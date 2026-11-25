"""Analytical profiles and integer encodings with the installed mechanical calibration."""
from dataclasses import replace
from math import sqrt

import pytest

from canopen_pp import config
from canopen_pp.pdo import Sample
from canopen_pp.pp import PPEncoder
from canopen_pp.pp_config import AxisTarget, PPTargets
from canopen_pp.trajectory import plan

START = [100, 100, 100, 0, 0, 45, 0, 0]
END = [220, 40, 130, -12, 45, 0, 0, 3]


def profile(starts, targets, duration, online=None):
    encoder = PPEncoder()
    axes = tuple(AxisTarget(n, x) for n, x in enumerate(targets, 1))
    samples = {n: Sample(pos=encoder.converter.target_to_raw(n, x))
               for n, x in enumerate(starts, 1) if online is None or n in online}
    return plan(PPTargets(duration, axes), samples, encoder)


@pytest.mark.parametrize("online", [set(range(1, 9)), {1, 2, 3, 4, 7, 8}])
def test_common_profile(online):
    trajectory = profile(START, END, 4, online)
    assert trajectory.ramp_s == 1
    assert trajectory.moving.keys() == online - {7}
    for n, axis in trajectory.moving.items():
        rate = [40, 20, 10, 4, 15, 15, None, 1][n-1]
        for actual in (axis.command.velocity, axis.command.acceleration, axis.command.deceleration):
            assert actual == pytest.approx(rate, abs=.0001)
        assert abs(axis.encoded_duration_s - 4) <= .1
        if n in (5, 8):
            raw = 25 if n == 5 else 300
            assert (axis.params.profile_velocity, axis.params.profile_acceleration,
                    axis.params.profile_deceleration) == (raw, raw, raw)


def test_triangle_at_exact_limit():
    result = profile([0]*8, [0]*7 + [14], 2)
    axis = result.moving[8]
    assert result.ramp_s == 1
    assert (axis.command.velocity, axis.command.acceleration, axis.command.deceleration) == (14, 14, 14)
    assert axis.encoded_duration_s == 2


def test_minimum_duration():
    with pytest.raises(ValueError, match="Node 8.*最短时间 2 s"):
        profile([0]*8, [0]*7 + [14], 1)


def test_limit_nodes_can_differ(monkeypatch):
    monkeypatch.setitem(config.PP_DESIGN_LIMITS, 1, replace(config.PP_DESIGN_LIMITS[1], velocity=10))
    # u >= 3 from node 1 and u*t >= 1 from node 8. Individual minima are lower.
    with pytest.raises(ValueError, match=r"速度限制 Node 1，加速度限制 Node 8，理论最短时间 3\.33333"):
        profile([0]*8, [30]+[0]*6+[14], 3.2)


def test_nearby_rate_codes_recover_duration():
    axis = profile([0]*8, [0]*4+[1]+[0]*3, 2).moving[5]
    v, a = axis.params.profile_velocity * .6, axis.params.profile_acceleration * .6
    distance = abs(axis.params.target_position - config.MECHANICAL_CONFIG[5].zero_offset) * 360 / 16384
    peak = min(v, sqrt(distance * a))
    assert axis.encoded_duration_s == pytest.approx(distance / peak + peak / a)
    assert abs(axis.encoded_duration_s - 2) <= .1
    assert (axis.params.profile_velocity, axis.params.profile_acceleration) != (1, 2)


def test_resolution_rejects_tiny_slow_move():
    with pytest.raises(ValueError, match="Node 5.*分辨率"):
        profile([0]*8, [0]*4+[.1]+[0]*3, 10)


def test_all_within_window():
    assert not profile(START, START, 4).moving


def test_small_start_drift_must_still_meet_encoded_time():
    from canopen_pp.drives import ControlError

    encoder = PPEncoder()
    targets = PPTargets(.8, tuple(AxisTarget(n, .04 if n == 1 else 0) for n in range(1, 9)))
    baseline = {1: Sample(pos=encoder.converter.target_to_raw(1, 0))}
    trajectory = plan(targets, baseline, encoder)
    # The axis still needs to move, but shortening this small move by 0.01 mm
    # makes the already encoded profile over 0.1 s too fast.
    changed = {1: Sample(pos=encoder.converter.target_to_raw(1, .01))}
    with pytest.raises(ControlError, match="编码轮廓时间误差"):
        trajectory.check_start(encoder, baseline, changed)


@pytest.mark.parametrize("node", range(5, 8))
def test_76_counts_in_three_seconds(node):
    axis = config.MECHANICAL_CONFIG[node]
    targets = [0.] * 8
    targets[node - 1] = 76 * axis.travel_per_revolution / axis.increments_per_revolution
    profile_axis = profile([0] * 8, targets, 3).moving[node]
    p = profile_axis.params
    assert abs(profile_axis.encoded_duration_s - 3) <= .1
    assert (p.profile_velocity, p.profile_acceleration, p.profile_deceleration) == (1, 4, 4)


@pytest.mark.parametrize(("node", "counts", "duration"), [
    (1, 110, 5), (2, 110, 5), (3, 138, 3), (4, 1093, 10),
    (5, 76, 3), (6, 76, 3), (7, 76, 3), (8, 76, 3),
])
def test_solution_outside_original_neighborhood(node, counts, duration):
    from math import floor

    cfg = config.MECHANICAL_CONFIG[node]
    distance = counts * cfg.travel_per_revolution / cfg.increments_per_revolution
    vs, acs = map(float, PPEncoder().profile_scales(node))
    v = distance / (.75 * duration)
    a = v / (.25 * duration)

    def elapsed(i, j):
        peak = min(i/vs, sqrt(distance*j/acs))
        return distance/peak + peak/(j/acs)

    for i in range(max(1, floor(v*vs)-1), floor(v*vs)+3):
        for j in range(max(1, floor(a*acs)-1), floor(a*acs)+3):
            assert abs(elapsed(i, j) - duration) > config.PP_ENCODED_TIME_TOLERANCE_S

    goals = [0.] * 8
    goals[node-1] = distance
    axis = profile([0]*8, goals, duration).moving[node]
    p = axis.params
    limit = config.PP_DESIGN_LIMITS[node]
    assert 1 <= p.profile_velocity <= limit.velocity*vs
    assert 1 <= p.profile_acceleration == p.profile_deceleration <= min(limit.acceleration, limit.deceleration)*acs
    assert axis.encoded_duration_s == pytest.approx(elapsed(p.profile_velocity, p.profile_acceleration))
    assert abs(axis.encoded_duration_s - duration) <= config.PP_ENCODED_TIME_TOLERANCE_S


@pytest.mark.parametrize("node", range(1, 9))
def test_time_above_slowest_integer_profile(node):
    goals = [0.] * 8
    goals[node-1] = .1
    with pytest.raises(ValueError, match=f"Node {node} 整数轮廓不可行.*最佳.*时间误差"):
        profile([0]*8, goals, 100_000)


@pytest.mark.parametrize("node", range(5, 8))
def test_point_one_degree_ten_seconds(node):
    goals = [0.] * 8
    goals[node-1] = .1
    with pytest.raises(ValueError, match=f"Node {node} 整数轮廓不可行"):
        profile([0]*8, goals, 10)


@pytest.mark.parametrize("node", range(1, 5))
def test_bounded_kinco_miss_is_not_reported_as_proven_infeasible(node, monkeypatch):
    from canopen_pp.trajectory import SearchMiss

    monkeypatch.setattr(config, "PP_ENCODED_TIME_TOLERANCE_S", 0)
    with pytest.raises(SearchMiss, match=f"Node {node} 有界搜索未找到解.*6081/6083/6084"):
        profile([0]*8, [.1]*8, 1.23456789, {node})


def test_group_refits_fine_axis_to_coarse_axis_ramp():
    targets = [4, 0, 0, 0, 76 * 360/16384, 0, 0, 0]
    trajectory = profile([0]*8, targets, 3)
    assert trajectory.ramp_s == pytest.approx(.25)
    assert abs(trajectory.moving[1].encoded_ramp_s - trajectory.moving[5].encoded_ramp_s) < .001
