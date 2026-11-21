"""Initialization and synchronous CiA 402 commands. Seven-axis absolute Profile Position motion."""
from math import isfinite
from threading import Lock
from time import monotonic, sleep

from . import config
from .communication import SyncClock
from .drives import ControlError, state
from .mechanical import MechanicalConversionError, MechanicalConverter
from .motion import PPMotion
from .move_test import batch_report, prepare
from .pdo import Feedback, check_unused_slots, configure_slot, slots, verify_slot
from .pp import PPEncoder
from .pp_config import load_targets

DISABLED = "Switch On Disabled"
READY = "Ready to Switch On"
SWITCHED = "Switched On"
ENABLED = "Operation Enabled"
FAULTS = {"Fault", "Fault Reaction Active", "Unknown"}


class Controller:
    def __init__(self, bus, drives, period_ms, report=print):
        period = period_ms / 1000
        if not isfinite(period) or not 0 < period < config.FEEDBACK_TIMEOUT:
            raise ControlError("SYNC 周期必须大于零并短于反馈超时")
        self.bus, self.drives, self.report = bus, drives, report
        self.parameter_test_only = config.PARAMETER_TEST_ONLY
        # Display configuration errors belong to the affected row, even during init.
        self.mechanical = MechanicalConverter(config.MECHANICAL_CONFIG, validate_on_init=False)
        self.feedback = Feedback(bus.network, drives)
        self.bus.on_sync = self.feedback.sync
        self.clock = SyncClock(bus, period)
        self.initialized = False
        self.pp_lock = Lock()
        self.pp_enabled = False
        self.completed_positions = None
        self.pp_inhibited = False
        self.needs_disable = False

    def initialize(self):
        if self.initialized:
            return
        # Passive check before changing the nodes' NMT states.
        sleep(2 * self.clock.period)
        self.bus.check_producers()
        for n in self.drives:
            self.bus.nmt(n, 0x80)
        for n, drive in self.drives.items():
            self.bus.expect(n, 0x1018, drive.vendor, 1)
            word = self.bus.read(n, 0x6041)
            if state(word) in {ENABLED, "Fault Reaction Active", "Unknown"}:
                raise ControlError(f"Node {n} 6041=0x{word:04X} ({state(word)})，停止重映射")
        check_unused_slots(self.bus, self.drives)
        verified = []
        for n, drive in self.drives.items():
            for slot in slots(drive):
                desired = configure_slot(self.bus, n, slot)
                verified.append((n, slot, desired))
        for n, slot, desired in verified:
            verify_slot(self.bus, n, slot, desired)
        for n in self.drives:
            self.bus.nmt(n, 1)
        self.clock.thread.start()
        samples = self.wait_feedback(self.drives, description="初始 TPDO1/TPDO2", both=True)
        if self.parameter_test_only:
            for n, s in samples.items():
                if state(s.word) != DISABLED:
                    raise ControlError(f"Node {n} 初始状态 {state(s.word)}，需要 {DISABLED}")
        self.initialized = True

    def wait_feedback(self, nodes, *, description, since=0, target=None, pp=False,
                      both=False, enabling=False, exact_cycle=False):
        deadline = monotonic() + config.TRANSITION_TIMEOUT
        while True:
            self.clock.check()
            samples = self.feedback.snapshot()
            pending = []
            cycles = set()
            for n in nodes:
                s = samples[n]
                valid = monotonic() - s.received1 <= config.FEEDBACK_TIMEOUT
                actual = state(s.word)
                if exact_cycle:
                    if max(s.cycle1, s.cycle2) > since:
                        raise ControlError("未能在批次 SYNC 对应周期确认七轴 TPDO1/TPDO2")
                    if valid and s.cycle1 == since and actual != target:
                        raise ControlError(f"Node {n} {description}: {actual}, 6041=0x{s.word:04X}")
                if enabling and valid and s.cycle1 >= since and actual in FAULTS:
                    raise ControlError(f"Node {n} {description}: {actual}, 6041=0x{s.word:04X}")
                cycles.add(s.cycle1)
                if (not valid or not s.cycle1 or s.cycle1 < since
                        or (both and (not s.fresh(config.FEEDBACK_TIMEOUT) or s.cycle2 != s.cycle1))
                        or (target and actual != target) or (pp and s.mode != 1)):
                    pending.append(n)
            if not pending and len(cycles) == 1:
                return samples
            if monotonic() >= deadline:
                raise ControlError(f"{description} 超时，未确认 Node {pending or list(nodes)}\n{self.status()}")
            sleep(min(.005, self.clock.period / 8))

    def status(self):
        rows = [("Node", "Motor", "Statusword", "CiA402 State", "6061",
                 "Position", "Raw(inc)", "Velocity", "Vel(raw)")]
        errors = []

        def physical(n, field, raw, convert, suffix):
            try:
                value = convert(n, raw)
                unit = self.mechanical.get_config(n).unit
                return f"{value:.3f} {unit}{suffix}"
            except MechanicalConversionError as exc:
                errors.append(f"Node {n} {field}: {exc}")
                return "CONV ERROR"

        samples = self.feedback.snapshot()
        now = monotonic()
        for n, s in samples.items():
            tp1 = now - s.received1 <= config.FEEDBACK_TIMEOUT
            tp2 = now - s.received2 <= config.FEEDBACK_TIMEOUT
            word = f"0x{s.word:04X}" if s.tp1 else "----"
            name = state(s.word) if tp1 else "TPDO1 STALE"
            position = physical(n, "Position", s.pos, self.mechanical.raw_to_actual, "") if tp1 else "--"
            velocity = physical(n, "Velocity", s.vel, self.mechanical.raw_to_velocity, "/s") if tp2 else "--"
            rows.append((str(n), self.drives[n].axis.family, word, name, str(s.mode) if tp1 else "--",
                         position, str(s.pos) if tp1 else "--", velocity, str(s.vel) if tp2 else "TPDO2 STALE"))
        widths = [max(map(len, col)) for col in zip(*rows)]
        lines = ["  ".join(cell.ljust(w) for cell, w in zip(row, widths)).rstrip() for row in rows]
        return "\n".join(lines + errors)

    def guard(self, required):
        for n, s in self.feedback.snapshot().items():
            if (not s.fresh(config.FEEDBACK_TIMEOUT) or state(s.word) != required
                    or s.cycle1 != self.feedback.sync_stamp or s.cycle2 != s.cycle1):
                raise ControlError(f"Node {n} 阶段预检失败，6041=0x{s.word:04X} "
                                   f"({state(s.word)}) 或上一周期 TPDO 不完整")

    def frames(self, nodes, control=None, torques=None):
        frames = []
        for n in nodes:
            drive = self.drives[n]
            word = drive.disable_word if control is None else control
            frames.append((0x400+n, drive.control_data(word, torques[n] if torques else None)))
        return frames

    def enable(self, targets=None):
        if self.parameter_test_only:
            raise ControlError("PARAMETER TEST ONLY 模式禁止 enable")
        if not self.initialized:
            raise ControlError("请先 init")
        if self.pp_inhibited:
            raise ControlError("PP 运动已锁定，请排除异常并确认 disable 后重新使能")
        try:
            targets = load_targets() if targets is None else targets
            targets.encode(PPEncoder(self.mechanical))
        except (OSError, ValueError) as exc:
            raise ControlError(f"PP 配置错误: {exc}") from exc
        torques = {a.command.node_id: a.torque_per_mille for a in targets.axes}
        try:
            samples = self.wait_feedback(self.drives, description="使能前反馈", both=True)
            if self.pp_enabled and all(state(s.word) == ENABLED and s.mode == 1 for s in samples.values()):
                return
            # Normalize mixed powered states, including Quick Stop Active. Never reset faults.
            for n, s in samples.items():
                if state(s.word) in FAULTS | {"Not Ready"}:
                    raise ControlError(f"Node {n} 不允许使能，6041=0x{s.word:04X} ({state(s.word)})")
            self.needs_disable = True
            if any(state(s.word) != DISABLED for s in samples.values()):
                self.disable()
                self.needs_disable = True
            previous = DISABLED
            for cw, target in ((6, READY), (7, SWITCHED), (15, ENABLED)):
                stamp = self.clock.batch(self.frames(self.drives, cw, torques),
                                         guard=lambda: self.guard(previous))
                samples = self.wait_feedback(self.drives, since=stamp, target=target, pp=(cw == 15),
                                             both=True, enabling=True, description=f"0x{cw:04X} → {target}")
                words = " ".join(f"N{n}=0x{samples[n].word:04X}" for n in self.drives)
                self.report(f"0x{cw:04X} → {target}  {words}")
                previous = target
            self.pp_enabled = True
        except BaseException:
            self.report("使能中止，回退前的 TPDO 反馈\n" + self.status())
            self.try_disable()
            raise

    def pp_move(self, targets):
        if self.parameter_test_only:
            raise ControlError("PARAMETER TEST ONLY 模式禁止 pp-move")
        if not self.initialized:
            raise ControlError("请先 init")
        if self.pp_inhibited:
            raise ControlError("PP 运动已锁定，请排除异常并确认 disable 后重新使能")
        if not self.pp_enabled:
            raise ControlError("请先 enable")
        if not self.pp_lock.acquire(blocking=False):
            raise ControlError("上一批 PP 运动尚未完成")
        try:
            try:
                axes = prepare(targets, self.drives, PPEncoder(self.mechanical), control=0x000F)
            except ValueError as exc:
                raise ControlError(f"PP 配置错误: {exc}") from exc
            motion = PPMotion(self, axes)
            torques = {a.command.node_id: a.torque_per_mille for a in targets.axes}
            trigger = self.frames(self.drives, 0x001F, torques)
            clear = [a.frames[2] for a in axes]
            try:
                already_in_window = motion.run(trigger, clear)
            except BaseException:
                self.abort_motion()
                raise
            self.completed_positions = {a.target.command.node_id: a.params.target_position for a in axes}
            if already_in_window:
                self.report("七轴在触发前已位于本次到位窗口内，本次定位无需位移。")
            self.report("Seven Axis PP Motion Completed\n\n" + "\n".join(
                f"Node {n}: Completed" for n in self.drives)
                + "\n\nAll axes remain Operation Enabled.\nReady for next pp-move.")
        finally:
            self.pp_lock.release()

    def move_test(self, targets):
        if not self.initialized:
            raise ControlError("请先 init")
        try:
            axes = prepare(targets, self.drives, PPEncoder(self.mechanical))
        except ValueError as exc:
            raise ControlError(f"PP 配置错误: {exc}") from exc
        frames = tuple(axis.frames[i] for i in range(3) for axis in axes)
        echoes = []
        samples = None
        try:
            self.guard(DISABLED)
            self.needs_disable = True
            stamp = self.clock.batch(frames, guard=lambda: self.guard(DISABLED), echoes=echoes)
            samples = self.wait_feedback(self.drives, description="批次后失能确认", since=stamp,
                                         target=DISABLED, both=True, exact_cycle=True)
        except ControlError as exc:
            self.report(batch_report(axes, echoes, samples, exc))
            raise
        self.needs_disable = False
        self.report(batch_report(axes, echoes, samples, None))

    def abort_motion(self):
        self.pp_inhibited = True
        self.report("PP 异常，已锁定后续运动，执行整组失能。")
        try:
            self._disable_group()
        except ControlError as exc:
            self.report(f"{exc}\n{self.status()}\n无法确认整组失能，请使用现场安全回路并确保垂直负载受支撑。")

    def disable(self):
        self._disable_group()
        self.pp_inhibited = False

    def _disable_group(self):
        self.pp_enabled = False
        self.completed_positions = None
        if self.clock.thread.ident is None:
            raise ControlError("Disable not confirmed，SYNC 尚未运行")
        self.needs_disable = True
        try:
            stamp = self.clock.batch(self.frames(self.drives), disable=True)
            self.wait_feedback(self.drives, since=stamp, target=DISABLED, description="RPDO3 失能")
        except Exception as exc:
            raise ControlError(f"Disable not confirmed: {exc}") from exc
        self.needs_disable = False
        self.report("TPDO1 已确认全部节点 Switch On Disabled")

    def try_disable(self):
        try:
            self.disable()
            return True
        except ControlError as exc:
            self.report(f"{exc}\n{self.status()}")
            return False

    def close(self):
        if self.clock.done.is_set():
            return
        try:
            if self.clock.thread.ident is not None:
                uncertain = any(monotonic() - s.received1 > config.FEEDBACK_TIMEOUT
                                or state(s.word) != DISABLED for s in self.feedback.snapshot().values())
                if uncertain or self.needs_disable:
                    self.disable()
            elif self.bus.operational_started:
                raise ControlError("Disable not confirmed，NMT Start 未完成且无同步反馈")
        finally:
            self.clock.close()
            self.initialized = False
            self.bus.close()
