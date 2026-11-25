"""Exercise the terminal and production PDO/SYNC path on python-can virtual only."""
import json
import struct
from time import sleep

import pytest

from canopen_pp import config, main as terminal
from canopen_pp.drives import ControlError
from canopen_pp.pdo import Sample
from canopen_pp.pp import PPEncoder
from canopen_pp.trajectory import plan
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
    doc["duration_s"] = .8
    for n, sim in sims.items():
        sim.travel_s = doc["duration_s"]
        doc["axes"][str(n)] = .1 if n == 8 else 3.6
    path.write_text(json.dumps(doc))
    return controller, sims, trace, log, path


def rpdo_batches(trace):
    batches, pending = [], []
    for cob, data in trace:
        if 0x200 < cob <= 0x408:
            pending.append((cob, data))
        if cob == 0x80 and pending:
            batches.append(pending)
            pending = []
    assert not pending
    return batches


def planned(c, sims, targets):
    return plan(targets, {n: Sample(pos=s.pos) for n, s in sims.items()}, PPEncoder(c.mechanical))


def verify_move(batches, trajectory):
    axes = trajectory.moving
    expected = [(0x200+n, struct.pack("<iI", a.params.target_position, a.params.profile_velocity))
                for n, a in axes.items()]
    expected += [(0x300+n, struct.pack("<II", a.params.profile_acceleration, a.params.profile_deceleration))
                 for n, a in axes.items()]

    def controls(word):
        return [(0x400+n, struct.pack("<Hb", word, 1) +
                 (struct.pack("<h", config.PP_TORQUE_PER_MILLE[n]) if n >= 5 else b""))
                for n in axes]

    assert batches == [expected + controls(15), controls(31), controls(15)]


def test_terminal_two_moves_reload_hold_and_reject_invalid(motion_system, monkeypatch, capsys):
    c, sims, trace, log, path = motion_system
    monkeypatch.setattr(terminal, "CONFIG_PATH", path)

    monkeypatch.setattr(terminal, "open_controller", lambda *args: c)

    def commands():
        yield "init"
        saved = path.read_text()
        path.write_text("{")
        yield "enable"
        path.write_text(saved)
        assert all([w for w, _, _ in s.frames] == [6, 7, 15] for s in sims.values())
        first = planned(c, sims, load_targets(path))
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
        assert c.status().count("Operation Enabled") == 8
        assert all(s.word & 0x1400 == 0x0400 for s in sims.values())
        trace.clear()
        path.unlink()
        yield "pp-move"
        path.write_text("{")
        yield "pp-move"
        doc["axes"]["2"] = 1101  # beyond mechanical travel
        path.write_text(json.dumps(doc))
        yield "pp-move"
        doc["duration_s"] = 0
        doc["axes"]["2"] = 3.6
        path.write_text(json.dumps(doc))
        yield "pp-move"
        assert rpdo_batches(trace) == []
        assert c.status().count("Operation Enabled") == 8
        doc["duration_s"] = .8
        doc["axes"] = dict(zip(doc["axes"], [10, 20, 30, -3, -5, 5, 5, 1]))
        path.write_text(json.dumps(doc))
        second = planned(c, sims, load_targets(path))
        yield "pp-move"
        verify_move(rpdo_batches(trace), second)
        assert first != second
        yield "status"
        assert c.status().count("Operation Enabled") == 8
        trace.clear()
        yield "pp-move"  # all hold, no setpoint edge
        assert rpdo_batches(trace) == []
        assert any("无需位移" in line for line in log)
        yield "disable"
        assert all(s.word == 0x40 for s in sims.values())
        yield "exit"

    script = commands()
    monkeypatch.setattr("builtins.input", lambda _: next(script))
    assert terminal.main() == 0
    assert capsys.readouterr().err.count("pp-move 失败") == 4
    assert sum("PP Motion Completed" in line for line in log) == 2


@pytest.mark.parametrize("ack_bit", [0, 1 << 12])
def test_repeated_moves_without_reached_flag_on_nodes_5_to_8(motion_system, monkeypatch, ack_bit):
    c, sims, trace, _, path = motion_system
    network = sims[5].node.network
    send = network.send_message

    def send_feedback(cob, data, *args, **kwargs):
        if 0x185 <= cob <= 0x188:
            word, pos, mode = struct.unpack("<Hib", data)
            data = struct.pack("<Hib", (word & ~0x1400) | ack_bit, pos, mode)
        return send(cob, data, *args, **kwargs)

    monkeypatch.setattr(network, "send_message", send_feedback)
    c.initialize()
    c.enable(load_targets(path))
    doc = json.loads(path.read_text())
    for position in (3.6, 7.2):
        doc["axes"]["5"] = position
        path.write_text(json.dumps(doc))
        targets = load_targets(path)
        expected = planned(c, sims, targets)
        trace.clear()
        c.pp_move(targets)
        verify_move(rpdo_batches(trace), expected)
        samples = c.feedback.snapshot()
        assert all(samples[n].word & 0x1400 == ack_bit for n in range(5, 9))


@pytest.mark.parametrize("failure", ["partial_ack", "old_ack", "stuck_ack", "old_reached",
                                     "following_error", "fault", "lost_feedback", "partial_tx",
                                     "lost_hold", "lost_position", "no_reached", "mode"])
def test_second_motion_failure_latches_without_retrigger(motion_system, failure, monkeypatch):
    c, sims, trace, log, path = motion_system
    c.initialize()
    c.enable(load_targets(path))
    c.pp_move(load_targets(path))
    doc = json.loads(path.read_text())
    doc["axes"] = {n: 2 * x for n, x in doc["axes"].items()}
    path.write_text(json.dumps(doc))
    trace.clear()
    sim = sims[4] if failure in ("partial_ack", "old_ack", "stuck_ack", "lost_hold", "no_reached") else sims[6]
    if failure == "partial_ack":
        sim.ignore.add(31)
    elif failure == "old_ack":
        sim.word |= 0x1000
        sleep(2 * c.clock.period)
    elif failure == "lost_hold":
        sim.word &= ~0x0400
        sleep(2 * c.clock.period)
    elif failure == "lost_position":
        sim.pos += 100  # node 6 moved outside the previous target's position window
        sleep(2 * c.clock.period)
    elif failure == "stuck_ack":
        sim.ignore.add(15)
    elif failure == "old_reached":
        sim.follow_target = False  # bit 10 remains high, actual position is still the old one
    else:
        original = c.bus.transmit

        def inject(cob, data, **kwargs):
            if cob == 0x406 and data[0] == 31:
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
                sim.trigger_stamp = None
                sim.pos, sim.vel = sim.target_position, 0
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
        assert "在线节点到位 超时" in str(error.value)
    assert c.pp_inhibited
    assert sum("PP Motion Completed" in line for line in log) == 1
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
            sims[8].send_velocity = not loading
            loading = False
        return original(cob, data, **kwargs)

    monkeypatch.setattr(c.bus, "transmit", skip_load_feedback)
    trace.clear()
    with pytest.raises(ControlError, match="参数装载"):
        c.pp_move(load_targets(path))
    assert c.pp_inhibited
    assert not any(cob == 0x401 and data[0] == 31 for cob, data in trace)


def test_start_velocity_above_window_rejects_before_load(motion_system, monkeypatch):
    c, sims, trace, _, path = motion_system
    monkeypatch.setitem(config.PP_VELOCITY_TOLERANCES, 5, 6.0)
    sims[5].vel = -12  # 7.2 deg/s in magnitude
    c.initialize()
    c.enable()
    trace.clear()
    with pytest.raises(ControlError, match="Node 5.*未静止"):
        c.pp_move(load_targets(path))
    assert rpdo_batches(trace) == []


def test_arrival_requires_velocity_in_window_and_reports_measured_time(motion_system, monkeypatch):
    c, sims, trace, log, path = motion_system
    # Other axes settle on schedule. Axis 5 reports its final position early,
    # while its velocity remains nonzero until the delayed stop.
    monkeypatch.setitem(config.PP_VELOCITY_TOLERANCES, 5, .3)  # simulated running speed is 0.6 deg/s
    sims[5].travel_s = 1.2
    c.initialize()
    c.enable()
    original = c.bus.transmit

    def enter_window(cob, data, **kwargs):
        if cob == 0x405 and data[0] == 15 and sims[5].trigger_stamp is not None:
            sims[5].pos = sims[5].target_position
        return original(cob, data, **kwargs)

    monkeypatch.setattr(c.bus, "transmit", enter_window)
    with pytest.raises(ControlError, match="实测到位时间超出"):
        c.pp_move(load_targets(path))
    report = next(line for line in log if "Node 5 起动" in line)
    arrival = float(report.split("到位 ")[1].split()[0])
    assert arrival == pytest.approx(1.2, abs=.12)
    assert c.pp_inhibited


def test_start_drift_after_loading_prevents_trigger(motion_system, monkeypatch):
    c, sims, trace, _, path = motion_system
    c.initialize()
    c.enable()
    original = c.bus.transmit

    def drift(cob, data, **kwargs):
        if cob == 0x301:
            sims[1].pos += 110  # 0.02014 mm, just outside the 0.02 mm window
        return original(cob, data, **kwargs)

    monkeypatch.setattr(c.bus, "transmit", drift)
    trace.clear()
    with pytest.raises(ControlError, match="起始位置变化"):
        c.pp_move(load_targets(path))
    assert not any(cob == 0x401 and data[0] == 31 for cob, data in trace)


@pytest.mark.parametrize("system", [range(1, 9), (1, 2, 3, 4, 7, 8)], indirect=True)
def test_four_second_profile_on_wire(system, monkeypatch):
    from test_trajectory import START, END
    from canopen_pp.pp_config import PPTargets, AxisTarget

    c, sims, trace, log = system
    c.parameter_test_only = False
    monkeypatch.setattr(config, "INITIAL_SDO_TIMEOUT", .15)
    for n, sim in sims.items():
        sim.pos = c.mechanical.target_to_raw(n, START[n - 1])
    c.initialize()
    c.enable()
    targets = PPTargets(4, tuple(AxisTarget(n, pos) for n, pos in enumerate(END, 1)))
    expected = planned(c, sims, targets)
    trace.clear()
    c.pp_move(targets)
    verify_move(rpdo_batches(trace), expected)
    assert expected.moving.keys() == sims.keys() - {7}
    assert [cw for cw, _, _ in sims[7].frames] == [6, 7, 15]
    assert all(s.word & 0x37 == 0x37 for s in sims.values())
    assert sum("起动" in line for line in log) == len(expected.moving)


@pytest.mark.parametrize(("n", "pos", "duration", "reason"),
                         [(8, 14, 1, "最短时间 2 s"), (5, .1, 10, "分辨率")])
def test_unplannable_sends_no_motion_frames(motion_system, n, pos, duration, reason):
    c, sims, trace, _, path = motion_system
    path.write_text(json.dumps({"duration_s": duration, "axes":
                                {str(i): pos if i == n else 0 for i in sims}}))
    c.initialize()
    c.enable()
    trace.clear()
    with pytest.raises(ControlError, match=reason):
        c.pp_move(load_targets(path))
    assert c.pp_enabled and not c.pp_inhibited and not rpdo_batches(trace)


def test_early_arrival_is_timed_from_trigger_during_ack(motion_system):
    c, sims, _, log, path = motion_system
    for sim in sims.values():
        sim.travel_s = 0
    c.initialize()
    c.enable()
    with pytest.raises(ControlError, match="实测到位时间超出"):
        c.pp_move(load_targets(path))
    reports = [line for line in log if "起动" in line]
    assert len(reports) == len(sims)
    for line in reports:
        assert 0 <= float(line.split("到位 ")[1].split()[0]) < c.clock.period
    assert c.pp_inhibited


def test_inter_axis_spread_is_checked_independently(motion_system, monkeypatch):
    c, sims, _, log, path = motion_system
    monkeypatch.setattr(config, "PP_ARRIVAL_TOLERANCE_S", .28)
    sims[5].travel_s, sims[6].travel_s = .6, 1.0
    c.initialize()
    c.enable()
    with pytest.raises(ControlError, match="实测到位时间超出"):
        c.pp_move(load_targets(path))
    arrivals = [float(line.split("到位 ")[1].split()[0]) for line in log if "起动" in line]
    assert all(abs(t - .8) <= .28 for t in arrivals)
    assert max(arrivals) - min(arrivals) > .28


def test_stale_feedback_on_holding_axis_prevents_group_load(motion_system):
    c, sims, trace, _, path = motion_system
    doc = json.loads(path.read_text())
    doc["axes"]["7"] = 0
    path.write_text(json.dumps(doc))
    c.initialize()
    c.enable()
    sims[7].send_velocity = False
    sleep(config.FEEDBACK_TIMEOUT + c.clock.period)
    trace.clear()
    with pytest.raises(ControlError, match="Node 7.*反馈失效"):
        c.pp_move(load_targets(path))
    assert c.pp_inhibited
    assert rpdo_batches(trace) == [[(0x400+n, s.drive.control_data(s.drive.disable_word))
                                    for n, s in sims.items()]]


def test_skipped_poll_replays_wire_arrival(motion_system, monkeypatch):
    from canopen_pp.motion import PPMotion

    c, sims, _, log, path = motion_system
    c.initialize()
    c.enable()
    observe = PPMotion.observe
    delayed = False

    def skip_poll(self, samples):
        nonlocal delayed
        observe(self, samples)
        # Pause only the command thread, just before the programmed arrival.
        if self.clear_stamp and not delayed:
            elapsed = samples[1].tp1 - self.trigger_stamp
            if .5 <= elapsed <= .7:
                delayed = True
                sleep(.36)

    monkeypatch.setattr(PPMotion, "observe", skip_poll)
    c.pp_move(load_targets(path))
    assert delayed
    arrivals = [float(line.split("到位 ")[1].split()[0]) for line in log if "起动" in line]
    assert len(arrivals) == 8
    assert arrivals == pytest.approx([.8]*8, abs=.08)


def test_encoder_noise_during_load_start_and_arrival(motion_system, monkeypatch):
    c, sims, _, log, path = motion_system
    axis = sims[5]
    origin = axis.pos
    monkeypatch.setitem(config.PP_VELOCITY_TOLERANCES, 5, .6)
    transmit = c.bus.transmit
    send = axis.node.network.send_message

    def load_with_jitter(cob, data, **kwargs):
        if cob == 0x305:
            axis.pos += 1  # one count is about 0.022 deg, inside 0.05 deg
        return transmit(cob, data, **kwargs)

    jitter_cycles = 2

    def noisy_feedback(cob, data, *args, **kwargs):
        nonlocal jitter_cycles
        if cob == 0x285:
            data = struct.pack("<i", -1)  # exactly on the velocity threshold
        elif cob == 0x185 and axis.trigger_stamp is not None and jitter_cycles:
            # For the first two cycles there is still only encoder noise.
            jitter_cycles -= 1
            word, _, mode = struct.unpack("<Hib", data)
            data = struct.pack("<Hib", word, origin + 3, mode)
        return send(cob, data, *args, **kwargs)

    monkeypatch.setattr(c.bus, "transmit", load_with_jitter)
    monkeypatch.setattr(axis.node.network, "send_message", noisy_feedback)
    c.initialize()
    c.enable()
    c.pp_move(load_targets(path))
    report = next(line for line in log if "Node 5 起动" in line)
    start = float(report.split("起动 ")[1].split()[0])
    assert start == pytest.approx(.2, abs=.06)
    assert c.pp_enabled
