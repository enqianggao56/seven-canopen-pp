"""Initialization and synchronous CiA 402 commands. No trajectory or set-point writer."""
import struct
from math import isfinite
from time import monotonic, sleep

from . import config
from .communication import SyncClock
from .drives import ControlError, state
from .pdo import Feedback, check_unused_slots, configure_slot, slots, verify_slot

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
        self.feedback = Feedback(bus.network, drives)
        self.bus.on_sync = self.feedback.sync
        self.clock = SyncClock(bus, period)
        self.initialized = False
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
        self.wait_feedback(self.drives, description="初始 TPDO1/TPDO2", both=True)
        self.initialized = True

    def wait_feedback(self, nodes, *, description, since=0, target=None, pp=False,
                      both=False, enabling=False):
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
        rows = ["Node  Motor  Statusword  CiA402 State           6061  Position(inc)  Velocity(raw)"]
        for n, s in self.feedback.snapshot().items():
            tp1 = monotonic() - s.received1 <= config.FEEDBACK_TIMEOUT
            tp2 = monotonic() - s.received2 <= config.FEEDBACK_TIMEOUT
            word = f"0x{s.word:04X}" if s.tp1 else "----"
            name = state(s.word) if tp1 else "TPDO1 STALE"
            rows.append(f"{n:4}  {self.drives[n].axis.family:5}  {word:10}  {name:21}  "
                        f"{str(s.mode) if tp1 else '--':4}  {str(s.pos) if tp1 else '--':13}  "
                        f"{s.vel if tp2 else 'TPDO2 STALE'}")
        return "\n".join(rows)

    def guard(self, required):
        for n, s in self.feedback.snapshot().items():
            if (not s.fresh(config.FEEDBACK_TIMEOUT) or state(s.word) != required
                    or s.cycle1 != self.feedback.sync_stamp or s.cycle2 != s.cycle1):
                raise ControlError(f"Node {n} 阶段预检失败，6041=0x{s.word:04X} "
                                   f"({state(s.word)}) 或上一周期 TPDO 不完整")

    def frames(self, nodes, control=None):
        return [(0x400+n, struct.pack("<Hb", self.drives[n].disable_word if control is None
                                      else control, 1)) for n in nodes]

    def enable(self):
        if not self.initialized:
            raise ControlError("请先 init")
        try:
            samples = self.wait_feedback(self.drives, description="使能前反馈", both=True)
            if all(state(s.word) == ENABLED and s.mode == 1 for s in samples.values()):
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
                stamp = self.clock.batch(self.frames(self.drives, cw),
                                         guard=lambda: self.guard(previous))
                samples = self.wait_feedback(self.drives, since=stamp, target=target, pp=(cw == 15),
                                             both=True, enabling=True, description=f"0x{cw:04X} → {target}")
                words = " ".join(f"N{n}=0x{samples[n].word:04X}" for n in self.drives)
                self.report(f"0x{cw:04X} → {target}  {words}")
                previous = target
        except BaseException:
            self.report("使能中止，回退前的 TPDO 反馈\n" + self.status())
            self.try_disable()
            raise

    def disable(self):
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
