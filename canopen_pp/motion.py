"""Single-axis PP bench sequencer. One caller owns the session and polls during motion."""
from dataclasses import dataclass
from enum import StrEnum
from math import isfinite
from time import monotonic, sleep

from . import config
from .controller import DISABLED, ENABLED, READY, SWITCHED, Controller
from .drives import ControlError, Kinco, RobStride, state
from .pp import PPMotionCommand, PPParameterEncoder

# iSMD §5.4: absolute, change immediately, set-point edge; manufacturer bit 12 stays low.
KINCO_BASE = 0x002F
NEW_SETPOINT = 1 << 4
HALT = 1 << 8
TARGET_REACHED = 1 << 10
SETPOINT_ACK = 1 << 12
SETTLE_SAMPLES = 3


class MotionState(StrEnum):
    IDLE = "IDLE"
    PREPARING = "PREPARING"
    READY = "READY"
    TRIGGERING = "TRIGGERING"
    MOVING = "MOVING"
    COMPLETED = "COMPLETED"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    FAILED = "FAILED"
    TIMEOUT = "TIMEOUT"


@dataclass(frozen=True)
class MotionStatus:
    state: MotionState
    node_id: int
    trigger_sync: float
    stop_confirmed: bool
    detail: str


class MotionTimeout(ControlError):
    pass


class AxisMotion:
    """All motion writes use the initialized controller's sole SYNC writer."""
    def __init__(self, controller: Controller, node: int, report=print):
        if type(node) is not int or node not in config.AXIS_CONFIG or set(controller.drives) != {node}:
            raise ControlError("调试控制器必须只连接一个明确选定的节点")
        self.c, self.node, self.report = controller, node, report
        self.drive = controller.drives[node]
        self.kinco = isinstance(self.drive, Kinco)
        self.axis = controller.mechanical.get_config(node)
        self.limits = limits = config.DEBUG_LIMITS[node]
        if not all(type(v) in (int, float) and isfinite(v) and v > 0 for v in
                   (*vars(limits).values(), config.DEBUG_MOTION_TIMEOUT, config.DEBUG_STOP_TIMEOUT)):
            raise ControlError(f"Node {node} 调试限值和超时必须为有限正数")
        if not self.kinco and (not isinstance(self.drive, RobStride)
                              or self.drive.axis.pp_torque_per_mille > config.DEBUG_MAX_TORQUE_PER_MILLE):
            raise ControlError(f"Node {node} 调试力矩超过上限")
        bounds = config.PPWriteLimits(self.axis.reference_axis, (0, limits.velocity),
                                     (0, limits.acceleration), (0, limits.deceleration))
        self.debug_encoder = PPParameterEncoder(controller.drives, controller.mechanical, {node: bounds})
        self.phase = MotionState.IDLE
        self.detail = ""
        self.hold_raw = self.verified_rates = self.current_limit = self.halt_option = None
        self.enabled_here = self.stop_confirmed = False
        self.stop_record = None
        self.trigger_sync = self.barrier = self.last_cycle = 0.
        self.encoded = None
        self.origin = 0.
        self.settled_at = []
        self.ack_high = False

    def _transition(self, phase, detail=""):
        self.phase, self.detail = phase, detail
        self.report(f"Node {self.node} {phase.value} {detail}")

    def get_motion_status(self):
        return MotionStatus(self.phase, self.node, self.trigger_sync, self.stop_confirmed, self.detail)

    def position(self, raw):
        return self.c.mechanical.raw_to_actual(self.node, raw)

    def velocity(self, raw):
        return self.c.mechanical.raw_to_velocity(self.node, raw)

    def preview(self, command: PPMotionCommand, origin_raw: int):
        if type(command.node_id) is not int or command.node_id != self.node:
            raise ControlError("调试命令的 Node 与选定节点不一致")
        self.c.pp.encode(command, for_write=True)
        encoded = self.debug_encoder.encode(command, for_write=True)
        start, end = self.position(origin_raw), self.position(encoded.target_position)
        # An absolute single-axis PP segment traverses the interval between its endpoints.
        points = (start, command.target_position, end)
        if min(points) < self.axis.min_position or max(points) > self.axis.max_position:
            raise ControlError("当前点或量化后路径超出位置软限位")
        if max(points) - min(points) > self.limits.travel:
            raise ControlError("预期路径超过单次调试行程")
        if abs(end - start) <= 2 * self.limits.position_window:
            raise ControlError("位移太小，起点和终点的反馈窗口重叠")
        return encoded

    @staticmethod
    def _rates(encoded):
        return (encoded.profile_velocity, encoded.profile_acceleration, encoded.profile_deceleration)

    def _at_origin(self, raw, raw_velocity, origin):
        actual = self.position(raw)
        if (not self.axis.min_position <= actual <= self.axis.max_position
                or abs(actual - origin) > self.limits.position_window
                or abs(self.velocity(raw_velocity)) > self.limits.stopped_velocity):
            raise ControlError(f"Node {self.node} 位置或速度偏离已确认起点: {actual} {self.axis.unit}")

    def stage_disabled(self, command, confirmed_raw):
        """Pre-operational initialization hook. Read back a hold target and the three rates."""
        bus, n = self.c.bus, self.node
        bus.require_preop(n)
        self.hold_raw = self.verified_rates = None
        word = bus.read(n, 0x6041)
        if state(word) != DISABLED:
            raise ControlError(f"Node {n} 失能写入要求 Switch On Disabled，6041=0x{word:04X}")
        raw = bus.read(n, self.drive.position_index)
        self._at_origin(raw, bus.read(n, 0x606C), self.position(confirmed_raw))
        encoded = self.preview(command, raw)
        rates = self._rates(encoded)
        if self.kinco:
            self.halt_option = bus.read(n, 0x605D)
            self.current_limit = bus.read(n, 0x6073)
        else:
            bus.set(n, 0x6071, self.drive.axis.pp_torque_per_mille)
        fields = list(zip((0x6081, 0x6083, 0x6084), rates)) + [(0x607A, raw)]
        for index, value in fields:
            bus.set(n, index, value)
        for index, value in fields:
            bus.expect(n, index, value)
        if state(bus.read(n, 0x6041)) != DISABLED:
            raise ControlError("写入期间驱动状态发生变化")
        self._at_origin(bus.read(n, self.drive.position_index), bus.read(n, 0x606C), self.position(raw))
        self.hold_raw, self.verified_rates = raw, rates
        self.report(f"Node {n} 失能读回 607A={raw}, 6081/6083/6084={rates}")

    def check_stop_record(self):
        record = config.DEBUG_VERIFIED_STOPS.get(self.node)
        if (not isinstance(record, config.BenchStop) or type(record.word) is not int
                or not isinstance(record.evidence, str)
                or not record.evidence.strip()
                or type(record.effort_limit) is not int or record.effort_limit <= 0):
            raise ControlError(f"Node {self.node} 缺少已验证的停止记录和力矩/电流限值")
        if self.kinco:
            if (record.word != (KINCO_BASE | HALT) or record.final_state != ENABLED
                    or self.halt_option != 1 or not 0 < self.current_limit <= record.effort_limit):
                raise ControlError("Kinco 停止记录、605Dh=1 或 6073h 电流上限不符合台架条件")
        elif (record.word != 0x000B or record.final_state not in {"Quick Stop Active", DISABLED}
              or self.drive.axis.pp_torque_per_mille > record.effort_limit):
            raise ControlError("RobStride 000Bh 停止记录、停止状态或力矩上限不符合台架条件")
        return record

    def _sample(self):
        self.c.clock.check()
        s = self.c.feedback.snapshot()[self.node]
        if not s.fresh(config.FEEDBACK_TIMEOUT):
            raise ControlError(f"Node {self.node} TPDO 反馈过期，6041=0x{s.word:04X}")
        return s

    def _power(self, s):
        # iSMD §5.1: bits 11 and 13 indicate a limit or following error in position mode.
        if state(s.word) != ENABLED or s.mode != 1 or (self.kinco and s.word & ((1 << 11) | (1 << 13))):
            raise ControlError(f"Node {self.node} {state(s.word)} 6041=0x{s.word:04X}, 6061={s.mode}")

    def _guard(self, *, origin=None, powered=True):
        s = self._sample()
        if not s.cycle1 or s.cycle1 != self.c.feedback.sync_stamp or s.cycle1 != s.cycle2:
            raise ControlError("本周期 TPDO1/TPDO2 不完整")
        if powered:
            self._power(s)
        if origin is not None:
            self._at_origin(s.pos, s.vel, origin)
        return s

    def enable(self):
        """Operator calls only after supporting the load and confirming the stored hold position."""
        if not self.c.initialized or self.hold_raw is None or self.enabled_here or self.phase != MotionState.IDLE:
            raise ControlError("需要先完成失能写入检查")
        self.stop_record = self.check_stop_record()
        origin = self.position(self.hold_raw)
        s = self.c.wait_feedback([self.node], description="使能前位置确认", both=True)[self.node]
        if state(s.word) != DISABLED:
            raise ControlError("人工使能前要求 Switch On Disabled")
        self._at_origin(s.pos, s.vel, origin)
        try:
            self.c.enable(interlock=lambda: self._guard(origin=origin, powered=False))
            self.enabled_here = True
            s = self._wait_sample(0, lambda s: True, "使能后的保持位置")
            self._at_origin(s.pos, s.vel, origin)
        except (ControlError, ValueError, KeyboardInterrupt) as exc:
            # The final enable word may already be on the wire when feedback or input is lost.
            if self.c.needs_disable and not self.enabled_here:
                try:
                    current = self._sample()
                    if state(current.word) == ENABLED and current.mode == 1:
                        self.enabled_here = True
                    else:
                        def before_disable():
                            actual = state(self._guard(powered=False).word)
                            if actual not in (DISABLED, READY, SWITCHED):
                                raise ControlError(f"使能中断，当前状态 {actual} 不允许失能回退")
                        self.c.disable(interlock=before_disable)
                except ControlError as recovery:
                    self.report(f"使能中断，回退未确认。{recovery}")
            self.fail(exc)
            raise

    def _wait_sample(self, since, predicate, label):
        deadline = monotonic() + config.TRANSITION_TIMEOUT
        while monotonic() < deadline:
            s = self._sample()
            self._power(s)
            if s.cycle1 == s.cycle2 and s.cycle1 >= since and predicate(s):
                return s
            sleep(.005)
        raise MotionTimeout(f"等待 {label} 超时")

    def _origin_guard(self):
        s = self._guard(origin=self.origin)
        # Re-check the entire segment if the axis has drifted since preview.
        self.preview(self.command, s.pos)
        return s

    def _control(self, word):
        return self.c.frames([self.node], word)

    def prepare_motion(self, command, confirmed_raw):
        if not self.enabled_here or self.phase not in {MotionState.IDLE, MotionState.COMPLETED}:
            raise ControlError("当前阶段不能准备新的定位")
        self._transition(MotionState.PREPARING)
        try:
            self.command, self.origin = command, self.position(confirmed_raw)
            self.encoded = self.preview(command, confirmed_raw)
            if self._rates(self.encoded) != self.verified_rates:
                raise ControlError("轮廓参数与失能时读回值不同，需重新分步验证")
            s = self._wait_sample(0, lambda s: True, "PP 反馈")
            self._at_origin(s.pos, s.vel, self.origin)
            if self.kinco:
                stamp = self.c.clock.batch(self._control(KINCO_BASE), guard=self._origin_guard)
                self._wait_sample(stamp, lambda s: not s.word & SETPOINT_ACK, "Bit 12 清零")
                stamp = self.c.clock.batch(self.encoded.frames(), guard=self._kinco_ready)
                self._wait_sample(stamp, lambda s: not s.word & SETPOINT_ACK, "参数批次后反馈")
            self._transition(MotionState.READY)
        except (ControlError, ValueError) as exc:
            self.fail(exc)
            raise

    def _kinco_ready(self):
        if self._origin_guard().word & SETPOINT_ACK:
            raise ControlError("Kinco Bit 12 尚未复位")

    def execute_motion(self):
        """Consume READY once. Poll until completion, or call request_stop immediately."""
        if self.phase != MotionState.READY:
            raise ControlError("本次人工触发需要新的 READY 命令")
        self._transition(MotionState.TRIGGERING)
        self.ack_high = self.stop_confirmed = False
        self.settled_at.clear()
        self.last_cycle = self.trigger_sync = 0.
        try:
            if self.kinco:
                frames, guard = self._control(KINCO_BASE | NEW_SETPOINT), self._kinco_ready
            else:
                frames, guard = (self.encoded.frames()[0],), self._origin_guard
            self.trigger_sync = self.barrier = self.c.clock.batch(frames, guard=guard)
            self.deadline = monotonic() + config.DEBUG_MOTION_TIMEOUT
            self.ack_deadline = monotonic() + config.TRANSITION_TIMEOUT
            self.report(f"Node {self.node} 触发 SYNC={self.trigger_sync:.6f}")
        except (ControlError, ValueError) as exc:
            self.fail(exc)
            raise

    def _settled(self, condition, position):
        if not condition:
            self.settled_at.clear()
            return False
        self.settled_at.append(position)
        self.settled_at = self.settled_at[-SETTLE_SAMPLES:]
        return (len(self.settled_at) == SETTLE_SAMPLES
                and max(self.settled_at) - min(self.settled_at) <= self.limits.position_window)

    def poll_motion(self):
        """Advance the handshake and monitor feedback. Called continuously by the bench terminal."""
        if self.phase not in {MotionState.TRIGGERING, MotionState.MOVING}:
            return self.get_motion_status()
        try:
            if monotonic() >= self.deadline:
                raise MotionTimeout("定位超时")
            if self.kinco and self.phase == MotionState.TRIGGERING and monotonic() >= self.ack_deadline:
                raise MotionTimeout("Bit 4/12 应答超时")
            s = self._sample()
            self._power(s)
            if s.cycle1 != s.cycle2 or s.cycle1 < self.barrier or s.cycle1 <= self.last_cycle:
                return self.get_motion_status()
            self.last_cycle = s.cycle1
            pos, vel = self.position(s.pos), self.velocity(s.vel)
            target = self.position(self.encoded.target_position)
            window = self.limits.position_window
            if (not self.axis.min_position <= pos <= self.axis.max_position
                    or not min(self.origin, target)-window <= pos <= max(self.origin, target)+window):
                raise ControlError(f"实际位置 {pos} 偏离预期路径")
            if self.phase == MotionState.TRIGGERING:
                if self.kinco:
                    if not self.ack_high and s.word & SETPOINT_ACK:
                        self.ack_high = True
                        self.barrier = self.c.clock.batch(self._control(KINCO_BASE), guard=self._guard)
                        self.ack_deadline = monotonic() + config.TRANSITION_TIMEOUT
                        return self.get_motion_status()
                    if not self.ack_high or s.word & SETPOINT_ACK:
                        return self.get_motion_status()
                elif (pos-self.origin) * (1 if target > self.origin else -1) <= window:
                    return self.get_motion_status()
                self._transition(MotionState.MOVING)
            reached = not self.kinco or bool(s.word & TARGET_REACHED)
            if self._settled(reached and abs(pos-target) <= window and abs(vel) <= self.limits.stopped_velocity, pos):
                self._transition(MotionState.COMPLETED, f"实际位置={pos}，实际速度={vel}")
        except (ControlError, ValueError) as exc:
            self.fail(exc)
        return self.get_motion_status()

    def request_stop(self):
        if not self.enabled_here or self.stop_record is None:
            raise ControlError("本会话尚未完成使能和停止策略验证")
        if self.stop_confirmed:
            return
        self._transition(MotionState.STOPPING)
        self.settled_at.clear()
        try:
            # Do not issue an operation-enabled controlword to a node in an unknown state.
            def stop_guard():
                s = self.c.feedback.snapshot()[self.node]
                if (not s.cycle1 or s.cycle1 != self.c.feedback.sync_stamp
                        or monotonic() - s.received1 > config.FEEDBACK_TIMEOUT
                        or state(s.word) != ENABLED or s.mode != 1):
                    raise ControlError(f"无法核实 PP 运行状态，6041=0x{s.word:04X}")
            stamp = self.c.clock.batch(self._control(self.stop_record.word), guard=stop_guard)
            self.report(f"Node {self.node} 停止 SYNC={stamp:.6f}")
            last = 0.
            deadline = monotonic() + config.DEBUG_STOP_TIMEOUT
            while monotonic() < deadline:
                s = self._sample()
                if state(s.word) in {"Fault", "Fault Reaction Active", "Unknown"}:
                    raise ControlError(f"停止时 6041=0x{s.word:04X}")
                if s.cycle1 == s.cycle2 and s.cycle1 >= stamp and s.cycle1 > last:
                    last = s.cycle1
                    self.report(f"停止反馈 SYNC={last:.6f}，6041=0x{s.word:04X}，"
                                f"pos={s.pos}，vel={s.vel} ({self.velocity(s.vel):.4f} {self.axis.unit}/s)")
                    ok = (state(s.word) == self.stop_record.final_state
                          and (not self.kinco or (s.mode == 1 and bool(s.word & TARGET_REACHED)))
                          and abs(self.velocity(s.vel)) <= self.limits.stopped_velocity)
                    if self._settled(ok, self.position(s.pos)):
                        self.stop_confirmed = True
                        self._transition(MotionState.STOPPED, f"6041=0x{s.word:04X}, pos={s.pos}, vel={s.vel}")
                        return
                sleep(.005)
            raise MotionTimeout("等待停止状态和静止反馈超时")
        except (ControlError, ValueError) as exc:
            self._transition(MotionState.TIMEOUT if isinstance(exc, MotionTimeout) else MotionState.FAILED,
                             f"停止未确认，{exc}")
            raise

    def fail(self, exc):
        reason = str(exc)
        if self.enabled_here and not self.stop_confirmed:
            try:
                self.request_stop()
            except (ControlError, ValueError) as stop_exc:
                reason += f"。停止未确认，{stop_exc}"
        self._transition(MotionState.TIMEOUT if isinstance(exc, MotionTimeout) else MotionState.FAILED, reason)

    def disable_after_stop(self):
        if not self.stop_confirmed:
            raise ControlError("停止未确认，请检查外部安全回路")

        def still_stopped():
            s = self._guard(powered=False)
            if (state(s.word) != self.stop_record.final_state
                    or abs(self.velocity(s.vel)) > self.limits.stopped_velocity
                    or abs(self.position(s.pos) - self.settled_at[-1]) > self.limits.position_window):
                raise ControlError("失能前的状态或实际反馈已偏离已确认静止点")

        try:
            self.c.disable(interlock=still_stopped)
        except ControlError as exc:
            self.stop_confirmed = False
            self._transition(MotionState.FAILED, str(exc))
            raise
        self.enabled_here = False
