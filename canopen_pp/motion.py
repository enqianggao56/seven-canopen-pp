"""One synchronous seven-axis absolute Profile Position handshake."""
from math import isfinite, sqrt
from time import monotonic, sleep

from . import config
from .drives import ControlError, state

ACK = 1 << 12
REACHED = 1 << 10
FOLLOWING_ERROR = 1 << 13


def ack_matches(node, word, *, expected):
    return node not in config.PP_ACK_NODES or bool(word & ACK) == expected


def reached_flag_ok(node, word):
    return node not in config.PP_TARGET_REACHED_NODES or bool(word & REACHED)


class PPMotion:
    def __init__(self, controller, axes):
        self.clock = controller.clock
        self.feedback = controller.feedback
        self.mechanical = controller.mechanical
        self.previous_positions = controller.completed_positions
        self.axes = {a.target.command.node_id: a for a in axes}
        self.position_tolerances = config.PP_POSITION_TOLERANCES
        for n in self.axes:
            tolerance = self.position_tolerances.get(n)
            if tolerance is None or not isfinite(tolerance) or tolerance <= 0:
                raise ControlError(f"Node {n} 到位误差阈值必须为有限正数")

    def snapshot(self):
        samples = self.feedback.snapshot()
        for n, s in samples.items():
            if not s.fresh(config.FEEDBACK_TIMEOUT):
                raise ControlError(f"Node {n} TPDO1/TPDO2 反馈失效")
            if state(s.word) != "Operation Enabled" or s.mode != 1 or s.word & FOLLOWING_ERROR:
                raise ControlError(f"Node {n} PP 状态异常: {state(s.word)}, "
                                   f"6041=0x{s.word:04X}, 6061={s.mode}")
        return samples

    def wait(self, stage, since, deadline, accepts, *, cycles=1, exact=False):
        """Require complete contemporaneous feedback after the specified wire barrier."""
        previous = None
        confirmations = 0
        while True:
            self.clock.check()
            samples = self.snapshot()
            stamp = self.feedback.sync_stamp
            if exact and stamp > since:
                raise ControlError(f"{stage} 缺少批次 SYNC 对应周期的完整七轴反馈")
            pending = []
            for n, s in samples.items():
                if s.cycle1 != stamp or s.cycle2 != stamp or stamp < since:
                    pending.append(n)
                elif not accepts(n, s):
                    pending.append(n)
                    confirmations = 0
            if monotonic() >= deadline:
                raise ControlError(f"{stage} 超时，等待 Node {pending or list(samples)}")
            if not pending and stamp != previous:
                previous = stamp
                confirmations += 1
                if confirmations >= cycles:
                    return samples
            sleep(min(.005, self.clock.period / 8))

    def error_distance(self, n, sample, target=None):
        # Compare the actual counts against the encoded target (including its rounding).
        cfg = self.mechanical.get_config(n)
        if target is None:
            target = self.axes[n].params.target_position
        error = target - sample.pos
        return abs(error) * cfg.travel_per_revolution / cfg.increments_per_revolution

    def check_ready_for_setpoint(self, samples, *, loaded=False):
        for n, s in samples.items():
            if not ack_matches(n, s.word, expected=False):
                raise ControlError(f"Node {n} 新设定点触发前 ACK 仍为 1")
            if self.previous_positions is not None:
                error = self.error_distance(n, s, self.previous_positions[n])
                # Loading 607Ah may change target-reached before the new edge.
                if (not loaded and not reached_flag_ok(n, s.word)) or error > self.position_tolerances[n]:
                    raise ControlError(f"Node {n} 上次定位未保持到位，位置误差 {error:.5f}")

    def guard(self, *, clearing=False, loaded=False):
        # Called by the sole writer immediately before it starts the next RPDO batch.
        samples = self.snapshot()
        for n, s in samples.items():
            if s.cycle1 != self.feedback.sync_stamp or s.cycle2 != s.cycle1:
                raise ControlError(f"Node {n} 上一 SYNC 周期反馈不完整")
            if clearing and not ack_matches(n, s.word, expected=True):
                raise ControlError(f"Node {n} 清除设定点前 ACK 已丢失")
        if not clearing:
            self.check_ready_for_setpoint(samples, loaded=loaded)

    def travel_timeout(self, samples):
        duration = 0.0
        for n, axis in self.axes.items():
            distance = self.error_distance(n, samples[n])
            if distance:
                cmd = axis.target.command
                ramps = 1 / cmd.acceleration + 1 / cmd.deceleration
                peak = min(cmd.velocity, sqrt(2 * distance / ramps))
                duration = max(duration, distance / peak + peak * ramps / 2)
        # Downward profile quantization, drive response, SYNC and settling time.
        return 2 * duration + config.PP_SETTLING_MARGIN + 4 * self.clock.period

    def run(self, trigger, clear):
        baseline = self.wait("运动预检", 0, monotonic() + config.FEEDBACK_TIMEOUT,
                             lambda n, s: True)
        self.check_ready_for_setpoint(baseline)
        frames = [a.frames[i] for i in range(3) for a in self.axes.values()]
        loaded = self.clock.batch(frames, guard=self.guard)
        baseline = self.wait("参数装载", loaded, monotonic() + config.PP_ACK_TIMEOUT,
                             lambda n, s: True, exact=True)
        self.check_ready_for_setpoint(baseline, loaded=True)
        timeout = self.travel_timeout(baseline)
        stamp = self.clock.batch(trigger, guard=lambda: self.guard(loaded=True))
        arrival_deadline = monotonic() + timeout
        self.wait("设定点 ACK=1", stamp, monotonic() + config.PP_ACK_TIMEOUT,
                  lambda n, s: ack_matches(n, s.word, expected=True))
        cleared = self.clock.batch(clear, guard=lambda: self.guard(clearing=True))
        self.wait("设定点 ACK=0", cleared, monotonic() + config.PP_ACK_TIMEOUT,
                  lambda n, s: ack_matches(n, s.word, expected=False))

        def arrived(n, s):
            return (reached_flag_ok(n, s.word) and ack_matches(n, s.word, expected=False)
                    and self.error_distance(n, s) <= self.position_tolerances[n])

        self.wait("七轴到位", cleared, arrival_deadline, arrived, cycles=2)
        return all(self.error_distance(n, s) <= self.position_tolerances[n]
                   for n, s in baseline.items())
