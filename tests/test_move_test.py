from dataclasses import replace
import struct
from time import sleep

import pytest

from canopen_pp.drives import ControlError
from canopen_pp.pp_config import load_targets
from test_virtual import system  # reuse the seven LocalNodes and the independent virtual listener


def test_batch_on_virtual_bus(system, monkeypatch):
    c, sims, trace, log = system
    c.initialize()
    targets = load_targets()
    # Use a negative target and different acceleration/deceleration on one rotary axis.
    axes = list(targets.axes)
    axes[3] = replace(axes[3], command=replace(axes[3].command, target_position=-45,
                                               velocity=60, acceleration=12, deceleration=3))
    targets = replace(targets, axes=tuple(axes))
    expected = [(546133, 447392, 894, 894), (546133, 447392, 894, 894),
                (344926, 565127, 1130, 1130), (-2048, 100, 20, 5), (228, 8, 16, 16),
                (228, 8, 16, 16), (8192, 300, 600, 600)]
    wire = [(0x201+i, struct.pack("<iI", pos, vel)) for i, (pos, vel, _, _) in enumerate(expected)]
    wire += [(0x301+i, struct.pack("<II", acc, dec)) for i, (_, _, acc, dec) in enumerate(expected)]
    wire += [(0x400+n, bytes.fromhex("00 00 01" if n <= 3 else "01 00 01 00 00")) for n in sims]
    batch = c.clock.batch
    receipts = []

    def record(frames, **kwargs):
        receipts.append(kwargs["echoes"])
        assert list(frames) == wire
        return batch(frames, **kwargs)

    monkeypatch.setattr(c.clock, "batch", record)
    trace.clear()
    for _ in range(2):
        c.move_test(targets)
    sleep(3 * c.clock.period)
    assert len(receipts) == 2
    for echoes in receipts:
        assert [cob for cob, _ in echoes] == [cob for cob, _ in wire] + [0x80]
        stamps = [stamp for _, stamp in echoes]
        assert stamps == sorted(stamps) and stamps[0] > 0
    for start in [i for i, frame in enumerate(trace) if frame == wire[0]]:
        assert trace[start:start+22] == wire + [(0x80, b"")]
    assert [frame for frame in trace if frame[0] != 0x80] == wire * 2
    assert trace[-2:] == [(0x80, b"")] * 2
    assert all(s.word == 0x40 for s in sims.values())
    for report in log:
        assert "Total RPDO: 21" in report and "SYNC Frames: 1" in report
        assert "Configured Torque: 100 ‰" in report and "RPDO3 Torque: 0" in report
        assert "SocketCAN capture review: manual (candump log)" in report
        assert "Result: TRANSMISSION COMPLETED" in report


def test_init_required_and_enable_blocked(system):
    c, _, trace, _ = system
    with pytest.raises(ControlError, match="init"):
        c.move_test(load_targets())
    c.initialize()
    with pytest.raises(ControlError, match="禁止 enable"):
        c.enable()
    assert not any(0x201 <= cob <= 0x407 for cob, _ in trace)


def test_test_mode_init_requires_disabled(system):
    c, sims, _, _ = system
    sims[7].word = 0x17
    with pytest.raises(ControlError, match="Node 7.*Quick Stop"):
        c.initialize()
    assert not c.initialized


@pytest.mark.parametrize("fault", ["invalid", "state", "stale TPDO1", "stale TPDO2", "guard"])
def test_preflight_sends_no_rpdo(system, fault, monkeypatch):
    c, sims, trace, log = system
    c.initialize()
    targets = load_targets()
    if fault == "invalid":
        last = targets.axes[-1]
        targets = replace(targets, axes=(*targets.axes[:-1], replace(last, torque_per_mille=0)))
    elif fault == "state":
        sims[4].word = 0x23
        sleep(c.clock.period * 2)
    elif fault.startswith("stale"):
        # Silence just the requested TPDO at its source.
        index = 3 if fault == "stale TPDO1" else 4
        mapped = sims[4].mapped
        monkeypatch.setattr(sims[4], "mapped", lambda slot: slot != sims[4].slots[index] and mapped(slot))
        sleep(.25)
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
    assert not any(0x201 <= cob <= 0x407 for cob, _ in trace)
    assert not any("TRANSMISSION COMPLETED" in line for line in log)
    if fault.startswith("stale"):
        monkeypatch.setattr(sims[4], "mapped", mapped)


@pytest.mark.parametrize("failed_id,count", [(0x304, 10), (0x80, 21)])
def test_partial_batch_inhibits_sync_and_close_recovers(system, monkeypatch, failed_id, count):
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
        c.move_test(load_targets())
    sleep(c.clock.period * 3)
    cut = next(i for i, (cob, _) in enumerate(trace) if cob == 0x201)
    assert len(trace[cut:]) == count and all(cob != 0x80 for cob, _ in trace[cut:])
    assert f"Total RPDO: {count}" in log[-1] and "SYNC Frames: 0" in log[-1]
    assert "Result: FAILED" in log[-1]
    c.close()
    assert [cob for cob, _ in trace[cut+count:]][:8] == list(range(0x401, 0x408)) + [0x80]


def test_fault_in_batch_cycle_fails_even_with_all_echoes(system, monkeypatch):
    c, sims, _, log = system
    c.initialize()
    original = c.bus.transmit

    def transmit(cob, data, **kwargs):
        if cob == 0x407:
            sims[4].word = 8
        return original(cob, data, **kwargs)

    with monkeypatch.context() as m:
        m.setattr(c.bus, "transmit", transmit)
        with pytest.raises(ControlError, match="Node 4.*Fault"):
            c.move_test(load_targets())
    assert "Total RPDO: 21" in log[-1] and "SYNC Frames: 1" in log[-1]
    assert "Result: FAILED" in log[-1]
    sims[4].word = 0x40


def test_missing_batch_tpdo2_cannot_be_confirmed_by_a_later_cycle(system, monkeypatch):
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
            sims[4].send_velocity = skipped
            skipped = True
        return original(cob, data, **kwargs)

    monkeypatch.setattr(c.bus, "transmit", transmit)
    with pytest.raises(ControlError, match="批次 SYNC"):
        c.move_test(load_targets())
    assert "SYNC Frames: 1" in log[-1] and "Result: FAILED" in log[-1]
