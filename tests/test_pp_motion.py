"""Exercise the terminal and production PDO/SYNC path on python-can virtual only."""
import json
import struct
from time import sleep

import pytest

from canopen_pp import config, main as terminal
from canopen_pp.drives import ControlError
from canopen_pp.pp import PPEncoder
from canopen_pp.pp_config import CONFIG_PATH, load_targets
from test_virtual import system


@pytest.fixture
def motion_system(system, monkeypatch, tmp_path):
    controller, sims, trace, log = system
    controller.parameter_test_only = False
    monkeypatch.setattr(config, "PARAMETER_TEST_ONLY", False)
    monkeypatch.setattr(config, "PP_ACK_TIMEOUT", .35)
    path = tmp_path / "targets.json"
    doc = json.loads(CONFIG_PATH.read_text())
    for n, sim in sims.items():
        sim.pos = sim.vel = 0
        doc["axes"][str(n)]["position"] = .1 if n == 7 else .5
    path.write_text(json.dumps(doc))
    return controller, sims, trace, log, path


def rpdo_batches(trace):
    batches, pending = [], []
    for cob, data in trace:
        if 0x200 < cob < 0x408:
            pending.append((cob, data))
        if cob == 0x80 and pending:
            batches.append(pending)
            pending = []
    assert not pending
    return batches


def verify_move(batches, targets):
    params = targets.encode(PPEncoder())
    expected = [(0x201+i, struct.pack("<iI", p.target_position, p.profile_velocity))
                for i, p in enumerate(params)]
    expected += [(0x301+i, struct.pack("<II", p.profile_acceleration, p.profile_deceleration))
                 for i, p in enumerate(params)]

    def controls(word):
        return [(0x400+a.command.node_id, struct.pack("<Hb", word, 1) +
                 (struct.pack("<h", a.torque_per_mille) if a.torque_per_mille is not None else b""))
                for a in targets.axes]

    assert batches == [expected + controls(15), controls(31), controls(15)]


def test_terminal_two_moves_reload_hold_and_reject_invalid(motion_system, monkeypatch, capsys):
    c, sims, trace, log, path = motion_system
    monkeypatch.setattr(terminal, "CONFIG_PATH", path)

    monkeypatch.setattr(terminal, "open_controller", lambda *args: c)

    def commands():
        yield "init"
        yield "enable"
        assert all([w for w, _, _ in s.frames] == [6, 7, 15] for s in sims.values())
        first = load_targets(path)
        doc = json.loads(path.read_text())
        transmit = c.bus.transmit

        def edit_during_trigger(cob, data, **kwargs):
            if cob == 0x401 and data[0] == 31:
                path.write_text("{")
            return transmit(cob, data, **kwargs)

        trace.clear()
        with monkeypatch.context() as m:
            m.setattr(c.bus, "transmit", edit_during_trigger)
            yield "pp-move"
        assert path.read_text() == "{"
        yield "status"
        verify_move(rpdo_batches(trace), first)
        assert c.status().count("Operation Enabled") == 7
        assert all(s.word & 0x1400 == 0x0400 for s in sims.values())
        trace.clear()
        path.unlink()
        yield "pp-move"
        path.write_text("{")
        yield "pp-move"
        doc["axes"]["2"]["position"] = 1101  # beyond mechanical travel
        path.write_text(json.dumps(doc))
        yield "pp-move"
        doc["axes"]["2"]["position"] = .5
        doc["axes"]["5"]["torque_per_mille"] = 1001
        path.write_text(json.dumps(doc))
        yield "pp-move"
        doc["axes"]["5"]["torque_per_mille"] = 100
        for field in ("velocity", "acceleration", "deceleration"):
            old = doc["axes"]["6"][field]
            doc["axes"]["6"][field] = 721  # beyond the axis design limit
            path.write_text(json.dumps(doc))
            yield "pp-move"
            doc["axes"]["6"][field] = old
        assert rpdo_batches(trace) == []
        assert c.status().count("Operation Enabled") == 7
        # Every axis exceeds its former commissioning caps within its design range.
        positions = [100, 100, 50, -5, 5, 5, 1]
        for n, row in doc["axes"].items():
            row.update(position=positions[int(n) - 1], velocity=8,
                       acceleration=12, deceleration=13)
            if "torque_per_mille" in row:
                row["torque_per_mille"] = 50 + int(n)
        path.write_text(json.dumps(doc))
        second = load_targets(path)
        yield "pp-move"
        verify_move(rpdo_batches(trace), second)
        assert first != second
        yield "status"
        assert c.status().count("Operation Enabled") == 7
        trace.clear()
        yield "pp-move"  # already at target, still leaves a clean handshake
        verify_move(rpdo_batches(trace), second)
        assert any("无需位移" in line for line in log)
        yield "disable"
        assert all(s.word == 0x40 for s in sims.values())
        yield "exit"

    script = commands()
    monkeypatch.setattr("builtins.input", lambda _: next(script))
    assert terminal.main() == 0
    assert capsys.readouterr().err.count("pp-move 失败") == 7
    assert sum("Seven Axis PP Motion Completed" in line for line in log) == 3


@pytest.mark.parametrize("ack_bit", [0, 1 << 12])
def test_repeated_moves_without_reached_flag_on_nodes_4_to_7(motion_system, monkeypatch, ack_bit):
    c, sims, trace, _, path = motion_system
    network = sims[4].node.network
    send = network.send_message

    def send_feedback(cob, data, *args, **kwargs):
        if 0x184 <= cob <= 0x187:
            word, pos, mode = struct.unpack("<Hib", data)
            data = struct.pack("<Hib", (word & ~0x1400) | ack_bit, pos, mode)
        return send(cob, data, *args, **kwargs)

    monkeypatch.setattr(network, "send_message", send_feedback)
    c.initialize()
    c.enable(load_targets(path))
    doc = json.loads(path.read_text())
    for position in (.5, 1):
        doc["axes"]["4"]["position"] = position
        path.write_text(json.dumps(doc))
        targets = load_targets(path)
        trace.clear()
        c.pp_move(targets)
        verify_move(rpdo_batches(trace), targets)
        samples = c.feedback.snapshot()
        assert all(samples[n].word & 0x1400 == ack_bit for n in range(4, 8))


@pytest.mark.parametrize("failure", ["partial_ack", "old_ack", "stuck_ack", "old_reached",
                                     "following_error", "fault", "lost_feedback", "partial_tx",
                                     "lost_hold", "lost_position", "no_reached", "mode"])
def test_second_motion_failure_latches_without_retrigger(motion_system, failure, monkeypatch):
    c, sims, trace, log, path = motion_system
    c.initialize()
    c.enable(load_targets(path))
    c.pp_move(load_targets(path))
    doc = json.loads(path.read_text())
    doc["axes"]["5"]["position"] = 1
    path.write_text(json.dumps(doc))
    trace.clear()
    sim = sims[2] if failure in ("partial_ack", "old_ack", "stuck_ack", "lost_hold", "no_reached") else sims[5]
    if failure == "partial_ack":
        sim.ignore.add(31)
    elif failure == "old_ack":
        sim.word |= 0x1000
        sleep(2 * c.clock.period)
    elif failure == "lost_hold":
        sim.word &= ~0x0400
        sleep(2 * c.clock.period)
    elif failure == "lost_position":
        sim.pos += 100  # node 5 moved outside the previous target's position window
        sleep(2 * c.clock.period)
    elif failure == "stuck_ack":
        sim.ignore.add(15)
    elif failure == "old_reached":
        sim.follow_target = False  # bit 10 remains high, actual position is still the old one
    else:
        original = c.bus.transmit

        def inject(cob, data, **kwargs):
            if cob == 0x405 and data[0] == 31:
                if failure == "following_error":
                    sim.word |= 1 << 13
                elif failure == "mode":
                    sim.mode = 3
                elif failure == "fault":
                    sim.word = 8
                elif failure == "lost_feedback":
                    sim.send_velocity = False
                elif failure == "partial_tx":
                    raise ControlError("partial trigger")
            if failure == "no_reached" and cob == 0x400 + sim.n and data[0] == 15 and sim.word & 0x1000:
                sim.word &= ~0x0400  # correct position alone must not complete this move
            return original(cob, data, **kwargs)

        monkeypatch.setattr(c.bus, "transmit", inject)
    monkeypatch.setattr(config, "PP_SETTLING_MARGIN", .1)
    with pytest.raises(ControlError) as error:
        c.pp_move(load_targets(path))
    if failure == "partial_ack":
        assert "设定点 ACK=1 超时" in str(error.value)
    elif failure == "stuck_ack":
        assert "设定点 ACK=0 超时" in str(error.value)
    elif failure == "no_reached":
        assert "七轴到位 超时" in str(error.value)
    assert c.pp_inhibited
    assert sum("Seven Axis PP Motion Completed" in line for line in log) == 1
    triggers = [data for cob, data in trace if cob == 0x401 and data[0] == 31]
    assert len(triggers) == (0 if failure in ("old_ack", "lost_hold", "lost_position") else 1)
    if failure == "partial_tx":
        assert all([w for w, _, _ in s.frames].count(31) == 1 for s in sims.values())
    with pytest.raises(ControlError, match="锁定"):
        c.pp_move(load_targets(path))
    with pytest.raises(ControlError, match="锁定"):
        c.enable(load_targets(path))
    sim.word, sim.vel, sim.send_velocity = 0x40, 0, True
    sim.ignore.clear()
    c.disable()
    assert not c.pp_inhibited


def test_missing_load_cycle_prevents_trigger(motion_system, monkeypatch):
    c, sims, trace, _, path = motion_system
    c.initialize()
    c.enable(load_targets(path))
    original = c.bus.transmit
    loading = False

    def skip_load_feedback(cob, data, **kwargs):
        nonlocal loading
        if cob == 0x201:
            loading = True
        if cob == 0x80:
            sims[7].send_velocity = not loading
            loading = False
        return original(cob, data, **kwargs)

    monkeypatch.setattr(c.bus, "transmit", skip_load_feedback)
    trace.clear()
    with pytest.raises(ControlError, match="参数装载"):
        c.pp_move(load_targets(path))
    assert c.pp_inhibited
    assert not any(cob == 0x401 and data[0] == 31 for cob, data in trace)


def test_motion_with_nonzero_velocity_feedback_can_outlast_enable_timeout(motion_system, monkeypatch):
    c, sims, _, log, path = motion_system
    axis = sims[4]
    axis.vel = -12  # Report -7.2 deg/s before the trigger and throughout arrival.
    c.initialize()
    c.enable(load_targets(path))
    original = c.bus.transmit
    cycles = 0

    def delay_arrival(cob, data, **kwargs):
        nonlocal cycles
        if cob == 0x404 and data[0] == 31:
            axis.follow_target = False
            cycles = 1
        if cob == 0x80 and cycles:
            cycles += 1
            if cycles == 20:
                axis.pos = axis.target_position
                axis.word |= 0x0400
        return original(cob, data, **kwargs)

    monkeypatch.setattr(c.bus, "transmit", delay_arrival)
    c.pp_move(load_targets(path))
    assert cycles >= 20 and 18 * c.clock.period > config.TRANSITION_TIMEOUT
    assert c.feedback.snapshot()[4].vel == -12
    assert any("Seven Axis PP Motion Completed" in line for line in log)
