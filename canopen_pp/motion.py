"""One synchronous absolute Profile Position handshake."""
from math import isfinite
from time import monotonic, sleep

from . import config
from .drives import ControlError, state
from .move_test import prepare
from .pp import PPEncoder
from .trajectory import plan, position_error, standstill

ACK = 1 << 12
REACHED = 1 << 10
FOLLOWING_ERROR = 1 << 13


def ack_matches(node, word, *, expected):
    return node not in config.PP_ACK_NODES or bool(word & ACK) == expected


def reached_flag_ok(node, word):
    return node not in config.PP_TARGET_REACHED_NODES or bool(word & REACHED)


class PlanningError(ControlError):
    """No batch submitted. The operator can correct the targets and retry."""


class PPMotion:
    def __init__(self, controller):
        self.clock = controller.clock
        self.feedback = controller.feedback
        self.mechanical = controller.mechanical
        self.encoder = PPEncoder(self.mechanical)
        self.drives = controller.online_drives
        self.previous_positions = controller.completed_positions
        self.previous_triggers = controller.triggered_nodes
        self.report = controller.report
        self.trigger_stamp = None
        self.clear_stamp = None
        self.last_sequence = None
        self.started = {}
        self.in_window = {}
        self.confirmations = {}

    def validate(self, samples):
        for n, s in samples.items():
            if not s.fresh(config.FEEDBACK_TIMEOUT):
                raise ControlError(f"Node {n} TPDO1/TPDO2 反馈失效")
            if state(s.word) != "Operation Enabled" or s.mode != 1 or s.word & FOLLOWING_ERROR:
                raise ControlError(f"Node {n} PP 状态异常: {state(s.word)}, "
                                   f"6041=0x{s.word:04X}, 6061={s.mode}")

    def snapshot(self):
        samples = self.feedback.snapshot(self.drives)
        self.validate(samples)
        return samples

    def wait(self, stage, since, deadline, accepts, *, exact=False):
        pending = list(self.drives)
        while True:
            self.clock.check()
            samples = self.snapshot()
            stamp = self.feedback.sync_stamp
            if exact and stamp > since:
                missing = ", ".join(f"Node {n}" for n in pending)
                raise ControlError(f"{stage} 缺少批次 SYNC 对应周期的完整反馈：{missing}")
            pending = [n for n, s in samples.items()
                       if not stamp or stamp < since or s.cycle1 != stamp or s.cycle2 != stamp]
            if not pending:
                self.observe(samples)
                pending = [n for n, s in samples.items() if not accepts(n, s)]
            if monotonic() >= deadline:
                raise ControlError(f"{stage} 超时，等待 Node {pending or list(samples)}")
            if not pending:
                return samples
            sleep(min(.005, self.clock.period / 8))

    def within(self, n, sample, target):
        return position_error(self.mechanical, n, target, sample.pos) <= config.PP_POSITION_TOLERANCES[n]

    def check_hold(self, samples, *, loaded=False):
        for n, s in samples.items():
            if not ack_matches(n, s.word, expected=False):
                raise ControlError(f"Node {n} 新设定点触发前 ACK 仍为 1")
            if self.previous_positions is not None:
                need_flag = n in self.previous_triggers and not (loaded and n in self.plan.moving)
                if ((need_flag and not reached_flag_ok(n, s.word))
                        or not self.within(n, s, self.previous_positions[n])):
                    raise ControlError(f"Node {n} 上次定位未保持到位")

    def ready(self, samples, *, loaded=False):
        self.check_hold(samples, loaded=loaded)
        self.plan.check_start(self.encoder, self.baseline, samples)

    def guard(self, *, clearing=False, loaded=False):
        samples = self.snapshot()
        for n, s in samples.items():
            if s.cycle1 != self.feedback.sync_stamp or s.cycle2 != s.cycle1:
                raise ControlError(f"Node {n} 上一 SYNC 周期反馈不完整")
            if clearing and n in self.plan.moving and not ack_matches(n, s.word, expected=True):
                raise ControlError(f"Node {n} 清除设定点前 ACK 已丢失")
        if clearing:
            self.observe(samples)
        else:
            self.ready(samples, loaded=loaded)
            if loaded:
                self.baseline = samples  # last pre-trigger position for observed start

    def observe(self, samples):
        if self.trigger_stamp is None:
            return
        sequence = next(iter(samples.values())).sequence
        if sequence == self.last_sequence:
            return
        if self.last_sequence is not None and sequence > self.last_sequence + 1:
            first = self.last_sequence + 1
            missed = self.feedback.closed_cycles(first, sequence - 1, self.drives)
            if len(missed) != sequence - first:
                raise ControlError("运动期间缺少连续的 SYNC 反馈记录")
            for seq, stamp, closed in missed:
                self.validate(closed)
                for n, s in closed.items():
                    if s.cycle1 != stamp or s.cycle2 != stamp:
                        raise ControlError(f"Node {n} 运动期间 SYNC 周期 {seq} 反馈不完整")
                self.observe_cycle(closed)
        self.observe_cycle(samples)
        self.last_sequence = sequence

    def observe_cycle(self, samples):
        for n, s in samples.items():
            quiet = standstill(self.mechanical, n, s.vel)
            inside = self.within(n, s, self.plan.positions[n]) and quiet
            if n not in self.plan.moving:
                if (not inside or not ack_matches(n, s.word, expected=False)
                        or (n in self.previous_triggers and not reached_flag_ok(n, s.word))):
                    raise ControlError(f"Node {n} 静止轴未保持到位")
                continue
            elapsed = max(s.tp1, s.tp2) - self.trigger_stamp
            if not quiet or not self.within(n, s, self.baseline[n].pos):
                self.started.setdefault(n, elapsed)
            if inside:
                self.in_window.setdefault(n, elapsed)
            else:
                self.in_window.pop(n, None)
            # Timestamp physical arrival independently of ACK and target-reached
            # latency. Completion still requires two consecutive post-clear
            # cycles satisfying all of the original handshake conditions.
            if (inside and self.clear_stamp is not None and s.cycle1 >= self.clear_stamp
                    and reached_flag_ok(n, s.word) and ack_matches(n, s.word, expected=False)):
                self.confirmations[n] = self.confirmations.get(n, 0) + 1
            else:
                self.confirmations[n] = 0

    def report_timing(self, duration):
        if self.trigger_stamp is None:
            return
        self.report(f"触发 SYNC {self.trigger_stamp:.6f}，T={duration:g} s，t={self.plan.ramp_s:g} s")
        for n, axis in self.plan.moving.items():
            start = f"{self.started[n]:.4f} s" if n in self.started else "未观测"
            arrival = "未确认"
            if self.confirmations.get(n, 0) >= 2:
                t = self.in_window[n]
                arrival = f"{t:.4f} s，相对 T 偏差 {t - duration:+.4f} s"
            self.report(f"Node {n} 起动 {start}，到位 {arrival}，编码预计 {axis.encoded_duration_s:.4f} s")
        if all(self.confirmations.get(n, 0) >= 2 for n in self.plan.moving):
            times = self.in_window.values()
            self.report(f"到位时间差 {max(times) - min(times):.4f} s")

    def run(self, targets):
        tolerance = config.PP_ARRIVAL_TOLERANCE_S
        if not isfinite(tolerance) or tolerance < 0:
            raise PlanningError("实测到位时间容差必须为非负有限秒数")
        self.baseline = self.wait("运动预检", 0, monotonic() + config.FEEDBACK_TIMEOUT,
                                  lambda n, s: True)
        self.check_hold(self.baseline)
        try:
            self.plan = plan(targets, self.baseline, self.encoder)
            axes = prepare(self.plan, self.drives, self.encoder, control=0x000F)
        except ValueError as exc:
            raise PlanningError(f"PP 规划失败: {exc}") from exc
        if not axes:
            self.report("在线节点均在目标窗口内，本次定位无需位移。")
            return self.plan
        trigger = [(a.frames[2][0], self.drives[a.command.node_id].control_data(31, a.torque_per_mille))
                   for a in axes]
        clear = [a.frames[2] for a in axes]
        frames = [a.frames[i] for i in range(3) for a in axes]
        loaded = self.clock.batch(frames, guard=self.guard)
        samples = self.wait("参数装载", loaded, monotonic() + config.PP_ACK_TIMEOUT,
                            lambda n, s: True, exact=True)
        self.ready(samples, loaded=True)
        self.trigger_stamp = self.clock.batch(trigger, guard=lambda: self.guard(loaded=True))
        deadline = monotonic() + targets.duration_s + config.PP_SETTLING_MARGIN + 2 * self.clock.period
        # Observe the trigger cycle even if the drive acknowledges in a later cycle.
        self.wait("触发反馈", self.trigger_stamp, monotonic() + config.FEEDBACK_TIMEOUT,
                  lambda n, s: True, exact=True)
        self.wait("设定点 ACK=1", self.trigger_stamp, monotonic() + config.PP_ACK_TIMEOUT,
                  lambda n, s: n not in self.plan.moving or ack_matches(n, s.word, expected=True))
        self.clear_stamp = self.clock.batch(clear, guard=lambda: self.guard(clearing=True))
        self.wait("设定点 ACK=0", self.clear_stamp, monotonic() + config.PP_ACK_TIMEOUT,
                  lambda n, s: ack_matches(n, s.word, expected=False))
        self.wait("在线节点到位", self.clear_stamp, deadline,
                  lambda n, s: n not in self.plan.moving or self.confirmations[n] >= 2)
        times = self.in_window.values()
        spread = max(times) - min(times)
        if spread > tolerance or any(abs(t - targets.duration_s) > tolerance for t in times):
            raise ControlError(f"实测到位时间超出容差 {tolerance:g} s")
        return self.plan
