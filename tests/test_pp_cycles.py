"""Script complete feedback cycles at the production pp_move / guarded batch boundary."""
from dataclasses import replace
from math import floor
import struct
from time import monotonic
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from canopen_pp import config, motion
from canopen_pp.controller import Controller
from canopen_pp.drives import ControlError, make_drives
from canopen_pp.pdo import Sample
from canopen_pp.pp import PPEncoder
from canopen_pp.pp_config import AxisTarget, PPTargets
from canopen_pp.trajectory import plan


NODES = tuple(range(1, 9))


class Cycles:
    period = .125
    thread = SimpleNamespace(ident=1)

    def __init__(self, monkeypatch, online):
        drives = make_drives(config.AXIS_CONFIG)
        self.log = []
        c = Controller(SimpleNamespace(network=Mock()), drives, 125, self.log.append)
        c.parameter_test_only = False
        c.initialized = c.pp_enabled = True
        c.clock = c.feedback = self
        c.online_drives = {n: drives[n] for n in online}
        self.controller = c
        self.origin = {n: c.mechanical.target_to_raw(n, 2.) for n in online}
        self.samples = {n: Sample(pos=pos, word=0x437, mode=1) for n, pos in self.origin.items()}
        self.sequence = 0
        self.batches = []
        self.loads = []
        self.edges = []
        self.goal = {}
        self.arrivals = {}
        self.on_cycle = lambda: None
        self.tick()
        monkeypatch.setattr(motion, "monotonic", lambda: self.sync_stamp)
        monkeypatch.setattr(motion, "sleep", lambda _: self.tick())

    def check(self):
        pass

    def snapshot(self, nodes):
        return {n: replace(self.samples[n]) for n in nodes}

    def tick(self):
        self.sequence += 1
        self.sync_stamp = 100 + self.period * self.sequence
        self.on_cycle()
        for n, s in self.samples.items():
            if self.arrivals.get(n) == self.sequence:
                s.pos, s.vel = self.goal[n], 0
                s.word |= motion.REACHED
            s.tp1 = s.tp2 = s.cycle1 = s.cycle2 = self.sync_stamp
            s.received1 = s.received2 = monotonic()
            s.sequence = self.sequence

    def batch(self, frames, *, guard=None, disable=False):
        if guard:
            guard()
        frames = list(frames)
        self.batches.append(frames)
        load, edges = {}, set()
        for cob, data in frames:
            n = cob & 0x7f
            s = self.samples[n]
            if cob == 0x200+n:
                self.goal[n], speed = struct.unpack("<iI", data)
                load[n] = [self.goal[n], speed]
            elif cob == 0x300+n:
                load[n].extend(struct.unpack("<II", data))
            elif disable:
                s.word = 0x40
            elif data[0] == 31:
                edges.add(n)
                s.word = 0x1037
                self.arrivals[n] = self.sequence + 1 + round(self.duration/self.period)
            else:
                s.word &= ~motion.ACK
        if load:
            self.loads.append(load)
        if edges:
            self.edges.append(edges)
        self.tick()
        return self.sync_stamp

    def move(self, positions=None, duration=3):
        self.duration = duration
        positions = positions or {}
        targets = PPTargets(duration, tuple(AxisTarget(n, positions.get(n, 2.)) for n in NODES))
        self.controller.pp_move(targets)

    def unit(self, n):
        cfg = self.controller.mechanical.get_config(n)
        return cfg.travel_per_revolution / cfg.increments_per_revolution

    def count_window(self, n):
        return floor(config.PP_HOLD_POSITION[n]/self.unit(n))


@pytest.mark.parametrize("n", NODES)
@pytest.mark.parametrize("condition", ["spike", "persistent", "severe", "velocity"])
def test_hold_protection(monkeypatch, n, condition):
    bus = Cycles(monkeypatch, NODES)
    neighbor = 1 + n % 8
    pos = bus.origin[n]
    # Unsent target near one edge, feedback briefly on the opposite side.
    request = bus.controller.mechanical.raw_to_actual(n, pos-bus.count_window(n))
    excess = bus.count_window(n) + 1
    first = []

    def feedback():
        if bus.edges:
            if not first:
                first.append(bus.sequence)
            offset = bus.sequence-first[0]
            if condition == "severe":
                bus.samples[n].pos = pos + 3 * excess
            elif condition == "velocity":
                raw_unit = abs(bus.controller.mechanical.raw_to_velocity(n, 1))
                bus.samples[n].vel = floor(config.PP_HOLD_VELOCITY[n]/raw_unit)+1
            else:
                bus.samples[n].pos = pos + (excess if condition == "persistent" or offset == 0 else 1)

    bus.on_cycle = feedback
    goals = {n: request, neighbor: 3.5}
    if condition == "spike":
        bus.move(goals)
        assert bus.controller.completed_positions[n] == pos
        bus.on_cycle = lambda: None
        bus.move({n: 2., neighbor: 3.5})
        assert bus.controller.pp_enabled
    else:
        with pytest.raises(ControlError, match=f"Node {n} 静止轴未保持到位"):
            bus.move(goals)
        assert bus.controller.pp_inhibited
        expected = 1 if condition == "severe" else config.PP_HOLD_ERROR_CYCLES
        # One additional batch SYNC was used to disable.
        assert bus.sequence-first[0] == expected
    assert bus.edges == [{neighbor}]


@pytest.mark.parametrize("n", NODES)
@pytest.mark.parametrize("recovers", [True, False])
def test_wait_for_rest(monkeypatch, n, recovers):
    bus = Cycles(monkeypatch, NODES)
    unit = abs(bus.controller.mechanical.raw_to_velocity(n, 1))
    bus.samples[n].vel = floor(config.PP_VELOCITY_TOLERANCES[n]/unit)+1
    if recovers:
        bus.on_cycle = lambda: setattr(bus.samples[n], "vel", 0)
        bus.move({n: 3.5})
        assert bus.edges == [{n}]
        assert bus.arrivals[n] == 1 + config.PP_STABLE_CYCLES + 2 + round(bus.duration/bus.period)
    else:
        with pytest.raises(ControlError, match=f"Node {n}.*未静止"):
            bus.move({n: 3.5})
        assert not bus.batches


@pytest.mark.parametrize("n", NODES)
def test_time_drift_reloads_every_moving_axis(monkeypatch, n):
    bus = Cycles(monkeypatch, NODES)
    mechanical = bus.controller.mechanical
    neighbor = 1 + n % 8
    cfg = mechanical.get_config(n)
    if cfg.device_type == "Kinco":
        counts = 3 * bus.count_window(n)
        drift = bus.count_window(n) // 2
    else:
        counts, drift = 76, 2
    target = mechanical.raw_to_actual(n, bus.origin[n] + counts)
    goals = {n: target, neighbor: 3.5}

    def shifted():
        if len(bus.loads) == 1:
            bus.samples[n].pos = bus.origin[n] - drift

    bus.on_cycle = shifted
    bus.move(goals)
    assert len(bus.loads) == 2
    assert bus.edges == [{n, neighbor}]
    before, after = bus.loads
    assert before.keys() == after.keys() == {n, neighbor}
    assert any("编码轮廓时间误差" in line for line in bus.log)
    samples = {i: Sample(pos=p-drift if i == n else p) for i, p in bus.origin.items()}
    targets = PPTargets(3, tuple(AxisTarget(i, goals.get(i, 2.)) for i in NODES))
    expected = plan(targets, samples, PPEncoder(mechanical))
    for i, axis in expected.moving.items():
        p = axis.params
        assert after[i] == [p.target_position, p.profile_velocity, p.profile_acceleration, p.profile_deceleration]


@pytest.mark.parametrize("n", NODES)
def test_axis_enters_or_leaves_moving_group(monkeypatch, n):
    bus = Cycles(monkeypatch, NODES)
    mechanical = bus.controller.mechanical
    neighbor = 1 + n % 8
    window = bus.count_window(n)
    # Start just inside the window. A small stable shift makes this a moving axis.
    target = mechanical.raw_to_actual(n, bus.origin[n] + window)
    for entered in (True, False):
        start = bus.origin[n] if entered else bus.origin[n] - 1
        end = bus.origin[n] - 1 if entered else bus.origin[n]
        bus.samples[n].pos = start
        def shift():
            if len(bus.loads) == 1:
                bus.samples[n].pos = end
        bus.on_cycle = shift
        # Allow short moves of one fine encoder window on coarse drives.
        bus.move({n: target, neighbor: 2.5}, duration=.5)
        first, second = bus.loads
        assert (n in first) != entered and (n in second) == entered
        assert bus.edges == [set(second)]
        bus = Cycles(monkeypatch, NODES)


@pytest.mark.parametrize("offline", NODES)
def test_mixed_group_and_all_hold(monkeypatch, offline):
    online = tuple(n for n in NODES if n != offline)
    bus = Cycles(monkeypatch, online)
    goals = {n: 2. + ((n-offline) % 3 - 1) * 1.5 for n in online}
    bus.move(goals)
    expected = {n for n in online if goals[n] != 2.}
    assert bus.edges == [expected]
    assert set(bus.loads[0]) == expected
    for n in online:
        assert bus.samples[n].pos == bus.controller.mechanical.target_to_raw(n, goals[n])
    bus.move(goals)
    assert len(bus.batches) == 3
    assert bus.controller.pp_enabled


@pytest.mark.parametrize("n", NODES)
def test_repeated_drift_exhausts_reloads_before_any_edge(monkeypatch, n):
    bus = Cycles(monkeypatch, NODES)
    neighbor = 1 + n % 8
    target = bus.controller.mechanical.raw_to_actual(n, bus.origin[n] + bus.count_window(n))

    def oscillate_after_each_load():
        bus.samples[n].pos = bus.origin[n] - len(bus.loads) % 2

    bus.on_cycle = oscillate_after_each_load
    with pytest.raises(ControlError, match="重规划次数 3"):
        bus.move({n: target, neighbor: 2.5}, duration=.5)
    assert len(bus.loads) == 1 + config.PP_MAX_REPLANS
    assert not bus.edges and bus.controller.pp_inhibited


@pytest.mark.parametrize("n", NODES)
def test_position_span_rejects_moving_origin_with_zero_velocity(monkeypatch, n):
    bus = Cycles(monkeypatch, NODES)
    step = bus.count_window(n)//2 + 1
    bus.on_cycle = lambda: setattr(bus.samples[n], "pos", bus.origin[n] + (bus.sequence-1)*step)
    with pytest.raises(ControlError, match=f"Node {n} 规划起点未静止，位置变化"):
        bus.move()
    assert not bus.batches


@pytest.mark.parametrize("n", NODES)
def test_all_hold_preserves_last_triggered_target(monkeypatch, n):
    bus = Cycles(monkeypatch, NODES)
    bus.move({n: 3.5})
    latched = bus.controller.completed_positions[n]
    bus.samples[n].pos = latched + bus.count_window(n)
    request = bus.controller.mechanical.raw_to_actual(n, bus.samples[n].pos + 1)
    bus.move({n: request})
    assert bus.edges == [{n}]
    assert len(bus.batches) == 3
    assert bus.controller.completed_positions[n] == latched
    assert bus.controller.pp_enabled


def test_integer_failure_after_load_inhibits_without_an_edge(monkeypatch):
    bus = Cycles(monkeypatch, NODES)
    n = 5
    target = bus.controller.mechanical.raw_to_actual(n, bus.origin[n] + bus.count_window(n))

    def shift():
        if bus.loads:
            bus.samples[n].pos = bus.origin[n] - 1

    bus.on_cycle = shift
    with pytest.raises(ControlError, match="重新规划失败.*整数轮廓不可行"):
        bus.move({n: target, 1: 12.}, duration=10)
    assert len(bus.loads) == 1 and not bus.edges
    assert bus.controller.pp_inhibited


def test_guard_rejection_at_last_instant_can_reload_but_ambiguous_edge_cannot(monkeypatch):
    bus = Cycles(monkeypatch, NODES)
    n = 5
    target = bus.controller.mechanical.raw_to_actual(n, bus.origin[n] + 76)
    batch = bus.batch
    submissions = []

    def submit(frames, **kw):
        if any(data[0] == 31 for cob, data in frames if 0x400 < cob <= 0x408):
            submissions.append(frames)
            if len(submissions) == 1:
                # A new SYNC has completed between the load check and the guard.
                bus.tick()
                bus.samples[n].pos -= 2
            else:
                kw["guard"]()
                raise ControlError("触发 TX echo 丢失")
        return batch(frames, **kw)

    monkeypatch.setattr(bus, "batch", submit)
    with pytest.raises(ControlError, match="TX echo"):
        bus.move({n: target})
    assert len(submissions) == 2 and len(bus.loads) == 2
    assert bus.controller.pp_inhibited


@pytest.mark.parametrize("n", NODES)
def test_opposite_edges_of_known_hold_window_during_loading(monkeypatch, n):
    bus = Cycles(monkeypatch, NODES)
    bus.move({n: 3.5})
    latched = bus.controller.completed_positions[n]
    jitter = bus.count_window(n)
    bus.samples[n].pos = latched + jitter
    neighbor = 1 + n % 8

    def feedback():
        if len(bus.loads) == 2:
            bus.samples[n].pos = latched - jitter

    bus.on_cycle = feedback
    bus.move({n: 3.5, neighbor: 3.5})
    assert bus.edges == [{n}, {neighbor}]
    assert bus.controller.completed_positions[n] == latched
