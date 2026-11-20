"""Real python-can virtual transport and seven LocalNode SDO servers; no physical I/O."""
import struct
from dataclasses import replace
from time import sleep
from uuid import uuid4

import canopen
import pytest

from canopen_pp import config
from canopen_pp.communication import Adapter, VENDOR
from canopen_pp.controller import Controller
from canopen_pp.drives import ControlError, make_drives
from canopen_pp.pdo import slots
from canopen_pp.pp import PPMotionCommand, PPParameterError


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
        self.profile_data = {}
        for slot in self.slots[:2]:
            net.subscribe(slot.cob, self.queue_profile)
        net.subscribe(0x400+self.n, self.queue)
        net.subscribe(0x80, self.tick)

    def queue(self, cob, data, stamp):
        self.pending = bytes(data)

    def queue_profile(self, cob, data, stamp):
        self.profile_data[cob] = bytes(data)

    def mapped(self, slot):
        def raw(index, sub):
            return int.from_bytes(self.node.get_data(index, sub), "little")
        return (raw(slot.com, 1) & ~0x40000000 == slot.cob and raw(slot.com, 2) == 1
                and raw(slot.mapping, 0) == len(slot.words)
                and all(raw(slot.mapping, j) == w for j, w in enumerate(slot.words, 1)))

    def tick(self, cob, data, stamp):
        if self.node.nmt.state != "OPERATIONAL":
            return
        for slot in self.slots[:2]:
            payload = self.profile_data.pop(slot.cob, None)
            if payload is not None and self.mapped(slot):
                for (index, _), field in zip(slot.fields, (payload[:4], payload[4:])):
                    self.node.set_data(index, 0, field)
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
    trace = []
    for cob in [0, 0x80] + [base+n for n in drives for base in (0x200, 0x300, 0x400, 0x600)]:
        devices.subscribe(cob, lambda cob, data, stamp: trace.append((cob, bytes(data))))
    sims = {n: SimDrive(devices, d) for n, d in drives.items()}
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


def test_disabled_parameter_write_on_virtual_can(system, monkeypatch):
    c, sims, trace, _ = system
    command = PPMotionCommand(4, -45, 60, 120, 30)
    c.initialize()
    with monkeypatch.context() as missing_config:
        missing_config.delitem(config.PP_WRITE_LIMITS, 4)
        with pytest.raises(PPParameterError, match="Node 4.*PP_WRITE_LIMITS"):
            c.write_pp_parameters(command)
    with pytest.raises(PPParameterError, match="6084h"):
        c.write_pp_parameters(replace(command, profile_deceleration=721))
    assert not any(0x200 < cob < 0x480 for cob, _ in trace)
    stamp = c.write_pp_parameters(command)
    c.wait_feedback([4], description="simulated parameter SYNC", since=stamp, both=True)
    first = (0x204, bytes.fromhex("00 F8 FF FF 64 00 00 00"))
    second = (0x304, bytes.fromhex("C8 00 00 00 32 00 00 00"))
    i = trace.index(first)
    assert trace[i:i+3] == [first, second, (0x080, b"")]
    assert [item for item in trace if 0x200 < item[0] < 0x480] == [first, second]
    start = trace.index((0, bytes((1, 1))))
    assert all(cob < 0x600 for cob, _ in trace[start:])
    node = sims[4].node
    assert [int.from_bytes(node.get_data(index, 0), "little", signed=index == 0x607A)
            for index in (0x607A, 0x6081, 0x6083, 0x6084)] == [-2048, 100, 200, 50]
    assert all(sim.word == 0x40 and not sim.frames for sim in sims.values())


def test_pp_write_requires_successful_parameter_mapping_readback(system):
    c, sims, trace, _ = system
    command = PPMotionCommand(4, -45, 60, 120, 30)
    def corrupt_rpdo2(index, subindex, data, **kwargs):
        if index == 0x1602 and subindex == 0 and data[0] == 3:
            sims[4].node.set_data(0x1601, 2, struct.pack("<I", 0x60830020))
    sims[7].node.add_write_callback(corrupt_rpdo2)
    with pytest.raises(ControlError, match="Node 4 1601:02"):
        c.initialize()
    with pytest.raises(ControlError, match="PDO 映射读回"):
        c.write_pp_parameters(command)
    assert not any(0x200 < cob < 0x480 for cob, _ in trace)


def test_seven_independent_parameter_writes(system):
    c, sims, trace, _ = system
    c.initialize()
    positions = {1: (600, 3276800), 2: (120, 655360), 3: (95, 655360),
                 4: (-45, -2048), 5: (45, 2048), 6: (360, 16384), 7: (20, 163840)}
    for n, (position, _) in positions.items():
        axis = config.MECHANICAL_CONFIG[n]
        # One rev/s, acceleration 2 rev/s², independent deceleration 0.5 rev/s².
        k = axis.travel_per_revolution
        stamp = c.write_pp_parameters(PPMotionCommand(n, position, k, 2*k, k/2))
        c.wait_feedback([n], description="parameter SYNC", since=stamp)
    batches = [i for i, (cob, _) in enumerate(trace) if 0x201 <= cob <= 0x207]
    assert [[cob for cob, _ in trace[i:i+3]] for i in batches] == [
        [0x200+n, 0x300+n, 0x80] for n in positions]
    for n, (_, target) in positions.items():
        node = sims[n].node
        assert int.from_bytes(node.get_data(0x607A, 0), "little", signed=True) == target
        counts = [int.from_bytes(node.get_data(i, 0), "little") for i in (0x6081, 0x6083, 0x6084)]
        assert counts == ([1073742, 2147, 537] if n <= 3 else [600, 1200, 300])
        assert sims[n].word == 0x40 and not sims[n].frames


@pytest.mark.parametrize("feedback", ["enabled", "stale"])
def test_pp_write_requires_disabled_feedback_at_dispatch(system, feedback):
    c, sims, trace, _ = system
    c.initialize()
    if feedback == "enabled":
        sims[4].word = 0x37
        c.wait_feedback([4], description="simulated enable", target="Operation Enabled")
    else:
        # Stop the simulated node's TPDOs while the master clock continues.
        sims[4].node.nmt.state = "PRE-OPERATIONAL"
        sleep(config.FEEDBACK_TIMEOUT + c.clock.period)
    try:
        with pytest.raises(ControlError, match="Node 4.*Switch On Disabled"):
            c.write_pp_parameters(PPMotionCommand(4, -45, 60, 120, 30))
        assert not any(0x200 < cob < 0x480 for cob, _ in trace)
    finally:
        sims[4].word = 0x40
        sims[4].node.nmt.state = "OPERATIONAL"
        c.wait_feedback([4], description="simulated disable", target="Switch On Disabled")


def test_status_probe_after_operator_wait_accepts_delayed_response(system):
    c, sims, trace, _ = system
    for index in (0x6041, 0x6061, 0x6064, 0x606C):
        c.bus.probe(4, index)

    def delayed_status(index, subindex, **kwargs):
        if index == 0x6041:
            sleep(.45)  # longer than regular SDO timeout, shorter than startup timeout
    sims[4].node.add_read_callback(delayed_status)
    sims[4].node.set_data(0x6041, 0, struct.pack("<H", 0x27))
    assert c.bus.probe(4, 0x6041) == 0x27  # fresh value, even if it now prevents initialization
    assert len(trace) == 5 and all(cob == 0x604 and data[0] == 0x40 for cob, data in trace)
    assert c.bus.nodes[4].sdo.RESPONSE_TIMEOUT == config.SDO_TIMEOUT


@pytest.mark.parametrize("request_echo", [True, False])
def test_probe_timeout_reports_this_requests_echo(system, monkeypatch, request_echo):
    c, sims, trace, _ = system
    bus = c.bus
    assert bus.probe(4, 0x6041) == 0x40
    monkeypatch.setattr(config, "INITIAL_SDO_TIMEOUT", .12)
    monkeypatch.setattr(config, "SDO_TIMEOUT", .06)
    if request_echo:
        monkeypatch.setattr(sims[4].node.sdo, "send_response", lambda response: None)
    else:
        monkeypatch.setattr(bus.network, "send_message", lambda cob, data: None)
    echo = "已见" if request_echo else "未见"
    with pytest.raises(ControlError, match=f"6041:00.*0.12 s，{echo} 0x604.*0x584"):
        bus.probe(4, 0x6041)
    assert bus.nodes[4].sdo.RESPONSE_TIMEOUT == .06
    assert not bus.operational_started and not bus.preoperational
    assert all(cob == 0x604 and data[0] in (0x40, 0x80) for cob, data in trace)


def test_probe_preserves_remote_abort(system):
    c, sims, _, _ = system
    def abort_status(index, subindex, **kwargs):
        if index == 0x6041:
            raise canopen.SdoAbortedError(0x06020000)
    sims[4].node.add_read_callback(abort_status)
    with pytest.raises(ControlError, match="06020000") as failure:
        c.bus.probe(4, 0x6041)
    assert isinstance(failure.value.__cause__, canopen.SdoAbortedError)
    assert "发送回显" not in str(failure.value)


@pytest.mark.parametrize("case", [
    "read-only", "cancel-stage", "mistyped-stage", "recheck-offline", "recheck-enabled", "missing-stop",
])
def test_single_axis_bench_entry_logs_and_gates(system, monkeypatch, tmp_path, case):
    import json

    import can
    from canopen_pp import bench

    host, sims, trace, _ = system
    node = sims[4].node
    node.set_data(0x6064, 0, struct.pack("<i", -3200))
    node.set_data(0x606C, 0, bytes(4))
    monkeypatch.setattr(config, "DEBUG_VERIFIED_STOPS", {})
    monkeypatch.setitem(config.CAN_CONFIG, "interface", "virtual")
    monkeypatch.setattr(bench.sys.stdin, "isatty", lambda: True)
    answers = iter(["2", "1.2", "2.4", "1.2"])
    def operator_input(prompt):
        if "STAGE 4" not in prompt:
            return next(answers)
        if case == "recheck-offline":
            monkeypatch.setattr(node.sdo, "send_response", lambda response: None)
        if case == "recheck-enabled":
            node.set_data(0x6041, 0, struct.pack("<H", 0x27))
        return {"cancel-stage": "", "mistyped-stage": "STAGE 4 确认"}.get(case, "STAGE 4")
    monkeypatch.setattr("builtins.input", operator_input)
    args = ["--node", "4", "--channel", host.bus.network.bus.channel_id, "--log-dir", str(tmp_path)]
    args += ["--read-only"] if case == "read-only" else ["--torque-per-mille", "25"]
    assert bench.main(args) == (0 if case == "read-only" else 1)
    directory, = tmp_path.iterdir()
    rows = [json.loads(x) for x in (directory / "steps.jsonl").read_text().splitlines()]
    with can.ASCReader(directory / "bus.asc") as capture:
        assert any(m.arbitration_id == 0x584 for m in capture)
    passed = [r["step"] for r in rows if r["event"] == "passed"]
    assert rows[0]["node"] == 4 and not any(0x200 < cob < 0x480 for cob, _ in trace)
    if case == "missing-stop":
        assert passed == [1, 2] and "停止记录" in rows[-1]["message"]
        assert [data for cob, data in trace if cob == 0] == [bytes([0x80, 4]), bytes([1, 4])]
        assert node.get_data(0x607A, 0) == struct.pack("<i", -3200)
        # Configured, read-back slope differs from acceleration.
        assert node.get_data(0x6084, 0) == struct.pack("<I", 2)
        assert node.get_data(0x6083, 0) == struct.pack("<I", 4)
        assert node.get_data(0x6071, 0) == struct.pack("<h", 25)
        assert not any(cob == 0x604 for cob, _ in trace[trace.index((0, bytes([1, 4]))):])
    elif case.startswith("recheck-"):
        assert passed == [1] and rows[-2]["event"] == "failed"
        assert all(cob == 0x604 and data[0] in (0x40, 0x80) for cob, data in trace)
        assert [int.from_bytes(data[1:3], "little") for _, data in trace if data[0] == 0x40] == [
            0x6041, 0x6061, 0x6064, 0x606C, 0x6041]
        reason = rows[-2]["reason"]
        assert ("已见 0x604" if case == "recheck-offline" else "6041=0x0027") in reason
    else:
        assert passed == [1]
        if case == "mistyped-stage":
            assert rows[-2]["event"] == "failed" and "应输入「STAGE 4」" in rows[-2]["reason"]
        assert [(cob, data[0], int.from_bytes(data[1:3], "little")) for cob, data in trace] == [
            (0x604, 0x40, index) for index in (0x6041, 0x6061, 0x6064, 0x606C)]
