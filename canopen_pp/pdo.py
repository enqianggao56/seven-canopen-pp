from dataclasses import dataclass, replace
import struct
from threading import Lock
from time import monotonic, time

from canopen import SdoAbortedError

from .drives import ControlError


@dataclass(frozen=True)
class Slot:
    com: int
    mapping: int
    cob: int
    fields: tuple  # (index, EDS data type)
    fmt: str

    @property
    def words(self):
        widths = {2: 8, 3: 16, 4: 32, 6: 16, 7: 32}
        return tuple(index << 16 | widths[kind] for index, kind in self.fields)


def slots(drive):
    n = drive.id
    return (
        Slot(0x1400, 0x1600, 0x200+n, ((0x607a, 4), (0x6081, 7)), "<iI"),
        Slot(0x1401, 0x1601, 0x300+n, ((0x6083, 7), (0x6084, 7)), "<II"),
        Slot(0x1402, 0x1602, 0x400+n, drive.control_fields, drive.control_struct.format),
        Slot(0x1800, 0x1a00, 0x180+n, ((0x6041, 6), (drive.position_index, 4), (0x6061, 2)), "<Hib"),
        Slot(0x1801, 0x1a01, 0x280+n, ((0x606c, 4),), "<i"),
    )


def check_unused_slots(bus, drives):
    used = {slot.cob for d in drives.values() for slot in slots(d)}
    for n, drive in drives.items():
        od = bus.nodes[n].object_dictionary
        control_objects = {index << 8 for index, _ in drive.control_fields}
        for index in sorted(i for i in od if 0x1403 <= i < 0x1600 or 0x1802 <= i < 0x1a00):
            try:
                cob = bus.read(n, index, 1)
            except ControlError as exc:
                abort = exc.__cause__
                if isinstance(abort, SdoAbortedError) and abort.code in (0x06020000, 0x06090011):
                    continue  # optional PDO absent on this firmware
                raise
            if cob & 0x80000000:
                continue
            wire_id = cob & ~0x40000000  # preserve the standard/extended frame flag
            if wire_id in used:
                raise ControlError(f"Node {n} 额外 PDO {index:04X} 与目标 COB-ID {cob:#x} 冲突")
            if index >= 0x1800:
                continue
            mapping = index + 0x200
            count = bus.read(n, mapping)
            for sub in range(1, count + 1):
                word = bus.read(n, mapping, sub)
                if word >> 8 in control_objects:
                    # Keep one active RPDO route for each RPDO3 field.
                    bus.set(n, index, cob | 0x80000000, 1)
                    break


def desired_cob(current, slot):
    # Bit 30 and, for standard frames, reserved bits 11..28 are preserved.
    flags = current & (0x40000000 if current & 0x20000000 else 0x5ffff800)
    return flags | slot.cob


def verify_slot(bus, n, slot, desired):
    bus.expect(n, slot.com, desired, 1)
    bus.expect(n, slot.com, 1, 2)
    bus.expect(n, slot.mapping, len(slot.words))
    for sub, word in enumerate(slot.words, 1):
        bus.expect(n, slot.mapping, word, sub)


def configure_slot(bus, n, slot):
    od = bus.nodes[n].object_dictionary
    for index, kind in slot.fields:
        var = od.get_variable(index, 0)
        if (not var or var.data_type != kind or not var.pdo_mappable
                or (slot.com < 0x1800 and not var.writable) or not var.readable):
            raise ControlError(f"Node {n} EDS 不支持映射 {index:04X} 至 {slot.mapping:04X}")
    cob = bus.read(n, slot.com, 1)
    desired = desired_cob(cob, slot)
    try:
        if (cob != desired or bus.read(n, slot.com, 2) != 1
                or bus.read(n, slot.mapping) != len(slot.words)
                or any(bus.read(n, slot.mapping, j) != w for j, w in enumerate(slot.words, 1))):
            bus.set(n, slot.com, cob | 0x80000000, 1)
            bus.set(n, slot.mapping, 0)
            for j, word in enumerate(slot.words, 1):
                bus.set(n, slot.mapping, word, j)
            bus.set(n, slot.mapping, len(slot.words))
            if (cob | 0x80000000) != (desired | 0x80000000):
                bus.set(n, slot.com, desired | 0x80000000, 1)
            bus.set(n, slot.com, 1, 2)
            bus.set(n, slot.com, desired, 1)
        return desired
    except ControlError as exc:
        words = ", ".join(f"{w:08X}" for w in slot.words)
        raise ControlError(f"Node {n} PDO {slot.mapping:04X} [{words}]: {exc}") from exc


@dataclass
class Sample:
    word: int = 0
    mode: int = 0
    pos: int = 0
    vel: int = 0
    tp1: float = 0.0  # bus timestamps for command barriers
    tp2: float = 0.0
    cycle1: float = 0.0  # wire timestamp of the preceding master SYNC
    cycle2: float = 0.0
    received1: float = float("-inf")  # independent monotonic freshness
    received2: float = float("-inf")

    def fresh(self, timeout):
        return monotonic() - min(self.received1, self.received2) <= timeout


class Feedback:
    def __init__(self, network, drives):
        self.samples = {n: Sample() for n in drives}
        self.lock = Lock()
        self.sync_stamp = 0.0
        for n, drive in drives.items():
            for slot in slots(drive)[3:]:
                network.subscribe(slot.cob, self.receiver(n, slot))

    def sync(self, stamp):
        with self.lock:
            self.sync_stamp = stamp

    def receiver(self, n, slot):
        def update(cob, data, stamp):
            received = monotonic() - max(0, time() - stamp)
            with self.lock:
                s = self.samples[n]
                first = slot.com == 0x1800
                if not self.sync_stamp or stamp < self.sync_stamp:
                    return
                if len(data) != struct.calcsize(slot.fmt):
                    if first:
                        s.received1 = float("-inf")
                    else:
                        s.received2 = float("-inf")
                    return
                if first:
                    s.cycle1 = self.sync_stamp
                    s.word, s.pos, s.mode = struct.unpack(slot.fmt, data)
                    s.tp1, s.received1 = stamp, received
                else:
                    s.cycle2 = self.sync_stamp
                    s.vel, = struct.unpack(slot.fmt, data)
                    s.tp2, s.received2 = stamp, received
        return update

    def snapshot(self):
        with self.lock:
            return {n: replace(s) for n, s in self.samples.items()}
