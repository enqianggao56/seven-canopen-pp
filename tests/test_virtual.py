"""Real python-can virtual transport and seven LocalNode SDO servers; no physical I/O."""
import struct
from time import sleep
from uuid import uuid4

import canopen
import pytest

from canopen_pp import config
from canopen_pp.communication import Adapter, VENDOR
from canopen_pp.controller import Controller
from canopen_pp.drives import ControlError, make_drives
from canopen_pp.pdo import slots


class SimDrive:
    def __init__(self, net, drive):
        self.drive = drive
        self.n = drive.id
        self.node = net.create_node(self.n, str(VENDOR / drive.eds))
        self.word, self.mode = 0x0040, 0
        self.pending = None
        self.ignore = set()
        self.mode_ok = self.send_velocity = True
        self.node.set_data(0x6041, 0, struct.pack("<H", self.word))
        self.slots = slots(drive)
        for slot in self.slots:
            self.node.set_data(slot.com, 2, bytes([255]))
        self.frames = []
        net.subscribe(0x400+self.n, self.queue)
        net.subscribe(0x80, self.tick)

    def queue(self, cob, data, stamp):
        self.pending = bytes(data)

    def mapped(self, slot):
        def raw(index, sub):
            return int.from_bytes(self.node.get_data(index, sub), "little")
        return (raw(slot.com, 1) & ~0x40000000 == slot.cob and raw(slot.com, 2) == 1
                and raw(slot.mapping, 0) == len(slot.words)
                and all(raw(slot.mapping, j) == w for j, w in enumerate(slot.words, 1)))

    def tick(self, cob, data, stamp):
        if self.node.nmt.state != "OPERATIONAL":
            return
        if self.pending is not None and self.mapped(self.slots[2]):
            if self.n >= 4:
                cw, mode, torque = struct.unpack("<Hbh", self.pending)
                self.node.set_data(0x6071, 0, struct.pack("<h", torque))
            else:
                cw, mode = struct.unpack("<Hb", self.pending)
            self.pending = None
            self.frames.append((cw, mode, stamp))
            if cw not in self.ignore:
                if cw == self.drive.disable_word and self.word != 8:
                    self.word = 0x40
                elif (cw, self.word) == (6, 0x40):
                    self.word = 0x31
                elif (cw, self.word) == (7, 0x31):
                    self.word = 0x33
                elif (cw, self.word) == (15, 0x33):
                    self.word = 0x37
                    self.mode = mode if self.mode_ok else 0
        if self.mapped(self.slots[3]):
            self.node.network.send_message(0x180+self.n, struct.pack("<Hib", self.word, -3200, self.mode))
        if self.send_velocity and self.mapped(self.slots[4]):
            self.node.network.send_message(0x280+self.n, struct.pack("<i", -10))


@pytest.fixture
def system(monkeypatch):
    monkeypatch.setattr(config, "TRANSITION_TIMEOUT", .45)
    monkeypatch.setattr(config, "FEEDBACK_TIMEOUT", .20)
    settings = dict(interface="virtual", channel=str(uuid4()), bitrate=1000000)
    devices = canopen.Network()
    devices.NOTIFIER_CYCLE = .02
    drives = make_drives(config.AXIS_CONFIG)
    sims = {n: SimDrive(devices, d) for n, d in drives.items()}
    trace = []
    for cob in [0, 0x80] + [base+n for n in drives for base in (0x200, 0x300, 0x400, 0x600)]:
        devices.subscribe(cob, lambda cob, data, stamp: trace.append((cob, bytes(data))))
    devices.connect(**settings)
    bus = Adapter(drives, settings)
    log = []
    c = Controller(bus, drives, 35, log.append)
    try:
        yield c, sims, trace, log
    finally:
        try:
            c.close()
        finally:
            devices.disconnect()


def test_full_wire_sequence(system):
    c, sims, trace, _ = system
    # Start from the previously deployed two-entry RPDO3.
    for s in sims.values():
        s.node.set_data(0x1402, 2, bytes([1]))
        s.node.set_data(0x1402, 1, struct.pack("<I", 0x400+s.n))
        s.node.set_data(0x1602, 0, bytes([2]))
        s.node.set_data(0x1602, 1, struct.pack("<I", 0x60400010))
        s.node.set_data(0x1602, 2, struct.pack("<I", 0x60600008))
    c.initialize()
    assert not any(s.frames for s in sims.values())
    c.enable()
    for n in range(4, 8):
        assert sims[n].node.get_data(0x6071, 0) == bytes.fromhex("E8 03")
    c.disable()
    c.enable()
    c.close()
    assert all(s.word == 0x40 for s in sims.values())
    assert [x for cob, x in trace if cob == 0] == [
        bytes((cmd, n)) for cmd in (0x80, 1) for n in sims]
    cut = trace.index((0, bytes((1, 1))))
    assert all(cob != 0x80 for cob, _ in trace[:cut])
    for cob, data in trace[:cut]:
        if cob >= 0x600:
            index = int.from_bytes(data[1:3], "little")
            assert index not in (0x1005, 0x1006, 0x1016, 0x1017, 0x6040, 0x6060,
                                 0x6071, 0x607A, 0x6081, 0x6083, 0x6084)
    assert all(cob < 0x600 for cob, _ in trace[cut:])
    assert all(not 0x201 <= cob <= 0x307 for cob, _ in trace)
    for i, (cob, data) in enumerate(trace):
        if 0x401 <= cob <= 0x407:
            assert data[:3] == bytes((data[0], 0, 1))
            if cob <= 0x403:
                assert len(data) == 3
            else:
                assert data[3:] == (bytes(2) if data[0] == 1 else bytes.fromhex("E8 03"))
        if cob == 0x401:
            assert [item[0] for item in trace[i:i+8]] == [0x400+n for n in sims]+[0x80]
    assert all([cw for cw, _, _ in s.frames] == [6, 7, 15, s.drive.disable_word]*2
               for s in sims.values())
    assert all(mode == 1 for s in sims.values() for _, mode, _ in s.frames)
    assert all(len(set(col)) == 1 for col in zip(*[[t for _, _, t in s.frames] for s in sims.values()]))
    with pytest.raises(ControlError, match="Pre-operational"):
        c.bus.read(1, 0x6041)


@pytest.mark.parametrize("failure", ["stage", "mode", "fault", "stale"])
def test_failure_rolls_back(system, failure):
    c, sims, _, log = system
    c.initialize()
    if failure == "stage":
        sims[4].ignore.add(7)
    elif failure == "mode":
        sims[4].mode_ok = False
    elif failure == "fault":
        sims[4].word = 8
        sleep(.08)
    else:
        sims[4].send_velocity = False
        sleep(.25)
    with pytest.raises(ControlError, match="4"):
        c.enable()
    if failure == "fault":
        assert sims[4].word == 8 and any("Disable not confirmed" in msg for msg in log)
        sims[4].word = 0x40  # emulate operator clearing the fault before fixture teardown
    assert all(s.word == 0x40 for s in sims.values())
    assert any("回退前" in line and "0x" in line for line in log)
    if failure != "mode":
        assert all(15 not in [cw for cw, _, _ in s.frames] for s in sims.values())


def test_quickstop_disable_timeout_and_retry(system):
    c, sims, _, _ = system
    sims[4].word = 0x17
    c.initialize()
    c.enable()
    assert sims[4].frames[0][0] == 1
    sims[4].ignore.add(1)
    with pytest.raises(ControlError, match="Disable not confirmed"):
        c.disable()
    assert "Operation Enabled" in c.status()
    sims[4].ignore.clear()
    c.disable()


def test_rejected_mapping_never_starts_nmt_or_sync(system):
    c, sims, trace, _ = system
    def reject(index, subindex, **kwargs):
        if (index, subindex) == (0x1602, 3):
            raise canopen.SdoAbortedError(0x06040041)
    sims[4].node.add_write_callback(reject)
    with pytest.raises(ControlError, match="06040041"):
        c.initialize()
    assert not any(cob == 0x80 or (cob == 0 and data[0] == 1) for cob, data in trace)


def test_robstride_torque_has_one_active_rpdo(system):
    c, sims, _, _ = system
    node = sims[5].node
    node.set_data(0x1403, 1, struct.pack("<I", 0x505))
    node.set_data(0x1603, 0, bytes([1]))
    node.set_data(0x1603, 1, struct.pack("<I", 0x60710010))
    c.initialize()
    assert int.from_bytes(node.get_data(0x1403, 1), "little") == 0x80000505
    assert sims[5].mapped(sims[5].slots[2])


def test_partial_enable_batch_replaced_before_next_sync(system, monkeypatch):
    c, sims, trace, log = system
    c.initialize()
    original = c.bus.transmit

    def fail_once(cob, data, **kw):
        if cob == 0x404 and data == bytes.fromhex("0F 00 01 E8 03"):
            raise ControlError("injected CAN TX failure")
        return original(cob, data, **kw)

    monkeypatch.setattr(c.bus, "transmit", fail_once)
    with pytest.raises(ControlError, match="injected CAN TX failure"):
        c.enable()
    cut = trace.index((0x401, bytes((15, 0, 1))))
    ids = [cob for cob, _ in trace[cut:]]
    assert ids[:ids.index(0x80)] == [0x401, 0x402, 0x403] + list(range(0x401, 0x408))
    assert all(15 not in [cw for cw, _, _ in s.frames] and s.word == 0x40 for s in sims.values())
    assert any("Switch On Disabled" in msg for msg in log)


def test_final_verification_catches_lost_mapping(system):
    c, sims, trace, _ = system
    def corrupt_peer(index, subindex, data, **kwargs):
        if index == 0x1602 and subindex == 0 and data[0] == 3:
            sims[4].node.set_data(0x1602, 3, struct.pack("<I", 0x60400010))
    sims[7].node.add_write_callback(corrupt_peer)
    with pytest.raises(ControlError, match="Node 4 1602:03"):
        c.initialize()
    assert not any(cob == 0x80 or (cob == 0 and data[0] == 1) for cob, data in trace)
