from dataclasses import replace
import struct
from time import sleep

import pytest

from canopen_pp import config
from canopen_pp.drives import ControlError
from test_virtual import system  # reuse the eight LocalNodes and the independent virtual listener


@pytest.fixture(autouse=True)
def parameter_test_mode(monkeypatch, reference_calibration, system):
    monkeypatch.setattr(config, "PARAMETER_TEST_ONLY", True)
    c, sims, _, _ = system
    for n, pos in enumerate([100, 100, 100, 0, 0, 45, 45, 0], 1):
        sims[n].pos = c.mechanical.target_to_raw(n, pos)
    c.parameter_test_only = True


@pytest.mark.usefixtures("reference_calibration")
def test_batch_on_virtual_bus(system, monkeypatch, wire_targets):
    c, sims, trace, log = system
    c.initialize()
    expected = [(1201493, 3579139, 3579, 3579), (218453, 1789569, 1789, 1789),
                (896808, 1130250, 1130, 1130), (-262144, 1431655, 1431, 1431),
                (2048, 25, 25, 25), (0, 25, 25, 25), (0, 25, 25, 25),
                (24576, 300, 300, 300)]
    wire = [(0x201+i, struct.pack("<iI", pos, vel)) for i, (pos, vel, _, _) in enumerate(expected)]
    wire += [(0x301+i, struct.pack("<II", acc, dec)) for i, (_, _, acc, dec) in enumerate(expected)]
    wire += [(0x400+n, bytes.fromhex("00 00 01" if n <= 4 else "01 00 01 00 00")) for n in sims]
    batch = c.clock.batch
    receipts = []

    def record(frames, **kwargs):
        receipts.append(kwargs["echoes"])
        assert list(frames) == wire
        return batch(frames, **kwargs)

    monkeypatch.setattr(c.clock, "batch", record)
    trace.clear()
    for _ in range(2):
        c.move_test(wire_targets)
    sleep(3 * c.clock.period)
    assert len(receipts) == 2
    for echoes in receipts:
        assert [cob for cob, _ in echoes] == [cob for cob, _ in wire] + [0x80]
        stamps = [stamp for _, stamp in echoes]
        assert stamps == sorted(stamps) and stamps[0] > 0
    for start in [i for i, frame in enumerate(trace) if frame == wire[0]]:
        assert trace[start:start+len(wire)+1] == wire + [(0x80, b"")]
    assert [frame for frame in trace if frame[0] != 0x80] == wire * 2
    assert trace[-2:] == [(0x80, b"")] * 2
    assert all(s.word == 0x40 for s in sims.values())
    for report in log:
        assert "Total RPDO: 24" in report and "SYNC Frames: 1" in report
        assert "Configured Torque: 600 ‰" in report and "RPDO3 Torque: 0" in report
        assert "SocketCAN capture review: manual (candump log)" in report
        assert "Result: TRANSMISSION COMPLETED" in report


def test_init_required_and_enable_blocked(system, wire_targets):
    c, _, trace, _ = system
    with pytest.raises(ControlError, match="init"):
        c.move_test(wire_targets)
    c.initialize()
    with pytest.raises(ControlError, match="禁止 enable"):
        c.enable()
    assert not any(0x201 <= cob <= 0x408 for cob, _ in trace)


def test_test_mode_init_requires_disabled(system):
    c, sims, _, _ = system
    sims[8].word = 0x17
    with pytest.raises(ControlError, match="Node 8.*Quick Stop"):
        c.initialize()
    assert not c.initialized


@pytest.mark.parametrize("fault", ["invalid", "state", "stale TPDO1", "stale TPDO2", "guard"])
def test_preflight_sends_no_rpdo(system, wire_targets, fault, monkeypatch):
    c, sims, trace, log = system
    c.initialize()
    targets = wire_targets
    if fault == "invalid":
        last = targets.axes[-1]
        targets = replace(targets, axes=(*targets.axes[:-1], replace(last, position=21)))
    elif fault == "state":
        sims[5].word = 0x23
        sleep(c.clock.period * 2)
    elif fault.startswith("stale"):
        # Silence just the requested TPDO at its source.
        index = 3 if fault == "stale TPDO1" else 4
        mapped = sims[5].mapped
        monkeypatch.setattr(sims[5], "mapped", lambda slot: slot != sims[5].slots[index] and mapped(slot))
        sleep(config.FEEDBACK_TIMEOUT + c.clock.period)
    else:
        original = c.clock.batch

        def changed_after_preflight(frames, **kw):
            monkeypatch.setattr(c, "guard", reject)
            return original(frames, **kw)

        def reject(required):
            raise ControlError("injected guard rejection")

        monkeypatch.setattr(c.clock, "batch", changed_after_preflight)
    with pytest.raises(ControlError):
        c.move_test(targets)
    assert not any(0x201 <= cob <= 0x408 for cob, _ in trace)
    assert not any("TRANSMISSION COMPLETED" in line for line in log)
    if fault.startswith("stale"):
        monkeypatch.setattr(sims[5], "mapped", mapped)


@pytest.mark.parametrize("failed_id,count", [(0x305, 12), (0x80, 24)])
def test_partial_batch_inhibits_sync_and_close_recovers(system, wire_targets, monkeypatch, failed_id, count):
    c, _, trace, log = system
    c.initialize()
    original = c.bus.transmit
    armed = False

    def transmit(cob, data, **kwargs):
        nonlocal armed
        if cob == 0x201:
            armed = True
        if armed and cob == failed_id:
            armed = False
            raise ControlError("injected CAN TX failure")
        return original(cob, data, **kwargs)

    monkeypatch.setattr(c.bus, "transmit", transmit)
    with pytest.raises(ControlError, match="injected CAN TX failure"):
        c.move_test(wire_targets)
    sleep(c.clock.period * 3)
    cut = next(i for i, (cob, _) in enumerate(trace) if cob == 0x201)
    assert len(trace[cut:]) == count and all(cob != 0x80 for cob, _ in trace[cut:])
    assert f"Total RPDO: {count}" in log[-1] and "SYNC Frames: 0" in log[-1]
    assert "Result: FAILED" in log[-1]
    c.close()
    assert [cob for cob, _ in trace[cut+count:]][:9] == list(range(0x401, 0x409)) + [0x80]


def test_fault_in_batch_cycle_fails_even_with_all_echoes(system, wire_targets, monkeypatch):
    c, sims, _, log = system
    c.initialize()
    original = c.bus.transmit

    def transmit(cob, data, **kwargs):
        if cob == 0x408:
            sims[5].word = 8
        return original(cob, data, **kwargs)

    with monkeypatch.context() as m:
        m.setattr(c.bus, "transmit", transmit)
        with pytest.raises(ControlError, match="Node 5.*Fault"):
            c.move_test(wire_targets)
    assert "Total RPDO: 24" in log[-1] and "SYNC Frames: 1" in log[-1]
    assert "Result: FAILED" in log[-1]
    sims[5].word = 0x40


def test_missing_batch_tpdo2_cannot_be_confirmed_by_a_later_cycle(system, wire_targets, monkeypatch):
    c, sims, _, log = system
    c.initialize()
    original = c.bus.transmit
    batch_started = False
    skipped = False

    def transmit(cob, data, **kwargs):
        nonlocal batch_started, skipped
        if cob == 0x201:
            batch_started = True
        if batch_started and cob == 0x80:
            sims[5].send_velocity = skipped
            skipped = True
        return original(cob, data, **kwargs)

    monkeypatch.setattr(c.bus, "transmit", transmit)
    with pytest.raises(ControlError, match="批次 SYNC"):
        c.move_test(wire_targets)
    assert "SYNC Frames: 1" in log[-1] and "Result: FAILED" in log[-1]


@pytest.mark.parametrize("raw_velocity", [-1, 2])
def test_disabled_batch_uses_same_noise_windows(system, wire_targets, monkeypatch, raw_velocity):
    c, sims, trace, _ = system
    monkeypatch.setitem(config.PP_VELOCITY_TOLERANCES, 5, .6)
    sims[5].vel = 1  # 0.6 deg/s at planning
    c.initialize()
    batch = c.clock.batch

    def jitter_before_submission(frames, **kwargs):
        sims[5].pos += 1
        sims[5].vel = raw_velocity
        sleep(2 * c.clock.period)
        return batch(frames, **kwargs)

    with monkeypatch.context() as m:
        m.setattr(c.clock, "batch", jitter_before_submission)
        if raw_velocity == -1:
            c.move_test(wire_targets)
            assert any(cob == 0x205 for cob, _ in trace)
        else:
            with pytest.raises(ControlError, match="Node 5.*速度超出静止窗口"):
                c.move_test(wire_targets)
            assert not any(0x201 <= cob <= 0x408 for cob, _ in trace)
    assert all(sim.word == 0x40 for sim in sims.values())
