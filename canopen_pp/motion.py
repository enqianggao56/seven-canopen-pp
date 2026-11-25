"""One synchronous absolute Profile Position handshake."""
from collections import deque
from time import monotonic, sleep

from . import config
from .drives import ControlError, state
from .move_test import prepare
from .pp import PPEncoder
from .trajectory import ReplanRequired, feedback_detail, plan, position_error, standstill

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
        self.held_positions = dict(controller.completed_positions or {})
        self.hold_errors = {}
        self.loaded_nodes = set()
        self.trigger_authorized = False
        self.stable_samples = None
        self.plan = None
        self.latest = {}
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
        self.latest = samples
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
            now = monotonic()
            if now >= deadline:
                if self.stable_samples is None or now >= deadline + self.clock.period:
                    raise ControlError(f"{stage} 超时，等待 Node {pending or list(samples)}")
                if self.last_sequence == self.feedback.sequence:
                    raise PlanningError(f"静止等待超时，{self.stability_reason}")
                # A SYNC may coincide with the deadline. Resolve its pending
                # TPDOs before distinguishing lost feedback from motion.
                pending = list(self.drives)
            if not pending:
                return samples
            sleep(min(.005, self.clock.period / 8))

    def check_hold(self, n, s):
        """Called once per complete cycle, including cycles recovered from history."""
        target = self.held_positions.get(n)
        if target is None:
            return
        position = position_error(self.mechanical, n, target, s.pos)
        speed = abs(self.mechanical.raw_to_velocity(n, s.vel))
        outside = position > config.PP_HOLD_POSITION[n] or speed > config.PP_HOLD_VELOCITY[n]
        count = self.hold_errors.get(n, 0) + 1 if outside else 0
        self.hold_errors[n] = count
        if (position > config.PP_HOLD_HARD_POSITION[n] or speed > config.PP_HOLD_HARD_VELOCITY[n]
                or count >= config.PP_HOLD_ERROR_CYCLES):
            detail = feedback_detail(self.mechanical, n, target, s, config.PP_HOLD_POSITION[n],
                                     config.PP_HOLD_VELOCITY[n])
            raise ControlError(f"Node {n} 静止轴未保持到位，连续 {count} 周期，{detail}，"
                               f"严重位置/速度阈值 {config.PP_HOLD_HARD_POSITION[n]:g}/"
                               f"{config.PP_HOLD_HARD_VELOCITY[n]:g}")

    def check_hold_flags(self, n, s):
        if not ack_matches(n, s.word, expected=False):
            raise ControlError(f"Node {n} 静止轴或触发前 ACK 仍为 1")
        loading = n in self.loaded_nodes and self.plan and n in self.plan.moving
        if n in self.previous_triggers and not loading and not reached_flag_ok(n, s.word):
            raise ControlError(f"Node {n} 上次定位未保持到位，6041=0x{s.word:04X}")

    def ready(self, samples):
        self.observe(samples)
        for n, s in samples.items():
            self.check_hold_flags(n, s)
        if any(count and n not in self.plan.moving for n, count in self.hold_errors.items()):
            raise ReplanRequired(f"静止轴保持窗口短时超差，超差周期 {self.hold_errors}")
        self.plan.check_start(self.encoder, self.baseline, samples)
        if any(self.hold_errors.values()):
            raise ReplanRequired(f"保持位置或速度尚未恢复，超差周期 {self.hold_errors}")

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
            self.ready(samples)
            if loaded:
                self.baseline = samples  # last pre-trigger position for observed start
                self.trigger_authorized = True  # next operation on SYNC thread sends the first edge

    def observe(self, samples):
        sequence = next(iter(samples.values())).sequence
        if sequence == self.last_sequence:
            return
        if self.last_sequence is not None and sequence > self.last_sequence + 1:
            first = self.last_sequence + 1
            missed = self.feedback.closed_cycles(first, sequence - 1, self.drives)
            if len(missed) != sequence - first:
                raise ControlError("运动期间缺少连续的 SYNC 反馈记录")
            for seq, stamp, closed in missed:
                self.latest = closed
                self.validate(closed)
                for n, s in closed.items():
                    if s.cycle1 != stamp or s.cycle2 != stamp:
                        raise ControlError(f"Node {n} 运动期间 SYNC 周期 {seq} 反馈不完整")
                self.observe_cycle(closed)
        self.latest = samples
        self.observe_cycle(samples)
        self.last_sequence = sequence

    def observe_cycle(self, samples):
        for n, s in samples.items():
            if self.trigger_stamp is None or n not in self.plan.moving:
                self.check_hold_flags(n, s)
                self.check_hold(n, s)
                continue
            speed = abs(self.mechanical.raw_to_velocity(n, s.vel))
            distance = position_error(self.mechanical, n, self.plan.positions[n], s.pos)
            inside = distance <= config.PP_POSITION_TOLERANCES[n] and speed <= config.PP_ARRIVAL_VELOCITY[n]
            elapsed = max(s.tp1, s.tp2) - self.trigger_stamp
            drift = position_error(self.mechanical, n, self.baseline[n].pos, s.pos)
            if speed > config.PP_START_VELOCITY[n] or drift > config.PP_START_POSITION[n]:
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

        if self.stable_samples is not None:
            self.record_stability(samples)

    def record_stability(self, samples):
        for n, s in samples.items():
            if self.hold_errors.get(n) or not standstill(self.mechanical, n, s.vel):
                self.stability_reason = f"Node {n} 规划起点未静止，606C={s.vel}"
                self.stable_samples.clear()
                return
        self.stable_samples.append(samples)
        for n in samples:
            while self.stable_samples:
                counts = [cycle[n].pos for cycle in self.stable_samples]
                span = position_error(self.mechanical, n, min(counts), max(counts))
                if span <= config.PP_STABLE_POSITION[n]:
                    break
                self.stability_reason = (f"Node {n} 规划起点未静止，位置变化 {span:g}，"
                                         f"阈值 {config.PP_STABLE_POSITION[n]:g}")
                # Retain the longest suffix stable for every axis.
                self.stable_samples.popleft()

    def stable_snapshot(self):
        self.stable_samples = deque(maxlen=config.PP_STABLE_CYCLES)
        self.stability_reason = "稳定周期不足"
        try:
            return self.wait("静止等待", 0, monotonic() + config.PP_STABLE_TIMEOUT_S,
                             lambda n, s: len(self.stable_samples) == config.PP_STABLE_CYCLES)
        finally:
            self.stable_samples = None

    def report_failure(self, targets):
        for axis in targets.axes:
            n = axis.node_id
            if n not in self.latest:
                continue
            try:
                target = self.mechanical.target_to_raw(n, axis.position)
            except ValueError:
                continue
            sample = self.latest[n]
            detail = feedback_detail(self.mechanical, n, target, sample, config.PP_POSITION_TOLERANCES[n],
                                     config.PP_ARRIVAL_VELOCITY[n] if self.trigger_stamp else
                                     config.PP_VELOCITY_TOLERANCES[n])
            profile = self.plan.moving.get(n) if self.plan else None
            text = profile.diagnostic(targets.duration_s) if profile else "无已规划运动参数"
            self.report(f"Node {n} 最近反馈，{detail}，保持参考 {self.held_positions.get(n)} inc，{text}")

    def report_timing(self, duration):
        if self.trigger_stamp is None:
            return
        self.report(f"触发 SYNC {self.trigger_stamp:.6f}，T={duration:g} s，t={self.plan.ramp_s:g} s")
        for n, axis in self.plan.moving.items():
            start = f"{self.started[n]:.4f} s" if n in self.started else "未观测"
            arrival = "未确认"
            if self.confirmations.get(n, 0) >= config.PP_ARRIVAL_CYCLES:
                t = self.in_window[n]
                arrival = f"{t:.4f} s，相对 T 偏差 {t - duration:+.4f} s"
            self.report(f"Node {n} 起动 {start}，到位 {arrival}，{axis.diagnostic(duration)}")
        if all(self.confirmations.get(n, 0) >= config.PP_ARRIVAL_CYCLES for n in self.plan.moving):
            times = self.in_window.values()
            self.report(f"到位时间差 {max(times) - min(times):.4f} s")

    def load_and_trigger(self, axes):
        if not axes:
            samples = self.wait("静止轴确认", 0, monotonic() + config.FEEDBACK_TIMEOUT,
                                lambda n, s: True)
            self.ready(samples)
            self.report("在线节点均在目标窗口内，本次定位无需位移。")
            return None
        clear = [a.frames[2] for a in axes]
        frames = [a.frames[i] for i in range(3) for a in axes]
        loaded = self.clock.batch(frames, guard=self.guard)
        self.loaded_nodes = set(self.plan.moving)
        samples = self.wait("参数装载", loaded, monotonic() + config.PP_ACK_TIMEOUT,
                            lambda n, s: True, exact=True)
        self.ready(samples)
        trigger = [(a.frames[2][0], self.drives[a.command.node_id].control_data(31, a.torque_per_mille))
                   for a in axes]
        self.trigger_stamp = self.clock.batch(trigger, guard=lambda: self.guard(loaded=True))
        return clear

    def run(self, targets):
        tolerance = config.PP_ARRIVAL_TOLERANCE_S
        try:
            config.validate_motion_settings(self.drives)
        except ValueError as exc:
            raise PlanningError(str(exc)) from exc
        for attempt in range(config.PP_MAX_REPLANS + 1):
            try:
                self.baseline = self.stable_snapshot()
                for n, s in self.baseline.items():
                    self.held_positions.setdefault(n, s.pos)
                self.plan = None
                self.plan = plan(targets, self.baseline, self.encoder)
                axes = prepare(self.plan, self.drives, self.encoder, control=0x000F)
            except (ValueError, PlanningError) as exc:
                if attempt:
                    raise ControlError(f"重新规划失败，{exc}") from exc
                raise PlanningError(f"PP 规划失败: {exc}") from exc

            try:
                clear = self.load_and_trigger(axes)
                if clear is None:
                    return self.plan
                break
            except ReplanRequired as exc:
                if self.trigger_authorized or attempt == config.PP_MAX_REPLANS:
                    raise ControlError(f"禁止继续运动，重规划次数 {attempt}，{exc}") from exc
                self.report(f"触发前整组重新规划 {attempt+1}/{config.PP_MAX_REPLANS}，{exc}")
        deadline = (monotonic() + targets.duration_s + config.PP_SETTLING_MARGIN
                    + config.PP_ARRIVAL_CYCLES * self.clock.period)
        # Observe the trigger cycle even if the drive acknowledges in a later cycle.
        self.wait("触发反馈", self.trigger_stamp, monotonic() + config.FEEDBACK_TIMEOUT,
                  lambda n, s: True, exact=True)
        self.wait("设定点 ACK=1", self.trigger_stamp, monotonic() + config.PP_ACK_TIMEOUT,
                  lambda n, s: n not in self.plan.moving or ack_matches(n, s.word, expected=True))
        self.clear_stamp = self.clock.batch(clear, guard=lambda: self.guard(clearing=True))
        self.wait("设定点 ACK=0", self.clear_stamp, monotonic() + config.PP_ACK_TIMEOUT,
                  lambda n, s: ack_matches(n, s.word, expected=False))
        self.wait("在线节点到位", self.clear_stamp, deadline,
                  lambda n, s: (self.confirmations[n] >= config.PP_ARRIVAL_CYCLES if n in self.plan.moving
                                else not self.hold_errors.get(n)))
        times = self.in_window.values()
        spread = max(times) - min(times)
        if spread > tolerance or any(abs(t - targets.duration_s) > tolerance for t in times):
            raise ControlError(f"实测到位时间超出容差 {tolerance:g} s")
        return self.plan
