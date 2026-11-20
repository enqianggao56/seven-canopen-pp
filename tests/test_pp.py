"""Offline PP encoding tests. Numeric examples in this file are test inputs."""
from dataclasses import replace
from math import inf, nan, nextafter

import pytest

from canopen_pp import config
from canopen_pp.drives import make_drives
from canopen_pp.mechanical import (MechanicalConverter, RawPositionOverflowError,
                                   TargetSoftLimitError, UnknownAxisError)
from canopen_pp.pp import PPMotionCommand, PPParameterEncoder, PPParameterError


CMD = PPMotionCommand(4, -45, 60, 120, 30)
LIMITS = config.PPWriteLimits("输出法兰", (0, 120), (0, 240), (0, 60))


@pytest.mark.parametrize("node, pos, raw, travel", [
    (1, 600, 3276800, 12), (2, 600, 3276800, 12), (3, 100, 689853, 9.5),
])
def test_kinco_pp_table_9_11(node, pos, raw, travel):
    # 200 rpm, 100 rps/s, 50 rps/s. The first two rate counts match the manual's CAN example.
    # The manual's example acceleration exceeds the installation's design limit.
    p = PPParameterEncoder(write_limits={}).encode(
        PPMotionCommand(node, pos, travel * 200 / 60, travel * 100, travel * 50))
    assert (p.target_position, p.profile_velocity, p.profile_acceleration, p.profile_deceleration) == (
        raw, 0x369D03, 0x1A36E, 53687)
    first, second = p.frames()
    assert first == (0x200 + node, raw.to_bytes(4, "little", signed=True) + bytes.fromhex("03 9D 36 00"))
    assert second == (0x300 + node, bytes.fromhex("6E A3 01 00 B7 D1 00 00"))


def test_robstride_negative_target_complete_wire_example():
    p = PPParameterEncoder().encode(CMD)
    assert (p.target_position, p.profile_velocity, p.profile_acceleration, p.profile_deceleration) == (
        -2048, 100, 200, 50)
    assert p.frames() == ((0x204, bytes.fromhex("00 F8 FF FF 64 00 00 00")),
                          (0x304, bytes.fromhex("C8 00 00 00 32 00 00 00")))


@pytest.mark.parametrize("node,pos,v,a,d,counts", [
    (4, 90, 60, 120, 30, (4096, 100, 200, 50)),
    (5, -45, 30, 6, 12, (-2048, 50, 10, 20)),
    (6, 360, 120, 60, 30, (16384, 200, 100, 50)),
    (7, 10, 1, 2, 0.5, (81920, 300, 600, 150)),
])
def test_robstride_rotary_and_screw(node, pos, v, a, d, counts):
    p = PPParameterEncoder().encode(PPMotionCommand(node, pos, v, a, d))
    assert (p.target_position, p.profile_velocity, p.profile_acceleration, p.profile_deceleration) == counts


def axis_encoder(node, *, write_limits=None, **changes):
    mechanical = MechanicalConverter({node: replace(config.MECHANICAL_CONFIG[node], **changes)})
    return PPParameterEncoder(mechanical=mechanical, write_limits=write_limits)


def test_rs06_one_revolution_scale_and_installed_limits():
    with pytest.raises(TargetSoftLimitError):
        PPParameterEncoder().encode(replace(CMD, target_position=360))
    # A detached test axis permits a full turn. The installed +90° limit still applies above.
    p = axis_encoder(4, max_position=360).encode(replace(CMD, target_position=360))
    assert p.target_position == 16384


@pytest.mark.parametrize("node,target,delta", [(1, 12, 65536), (4, -45, -2048)])
def test_direction_and_offset_affect_only_position(node, target, delta):
    cmd = replace(CMD, node_id=node, target_position=target)
    p = PPParameterEncoder().encode(cmd)
    reversed_p = axis_encoder(node, direction=-1, zero_offset=125).encode(cmd)
    assert p.target_position == delta
    assert reversed_p.target_position == 125 - delta
    assert reversed_p.frames()[0][1][4:] == p.frames()[0][1][4:]
    assert reversed_p.frames()[1] == p.frames()[1]


def test_kinco_profile_scale_independent_of_position_and_feedback_settings():
    cmd = replace(CMD, node_id=1, target_position=12)
    p = PPParameterEncoder().encode(cmd)
    custom = axis_encoder(1, increments_per_revolution=1000, velocity_unit="inc_per_second").encode(cmd)
    assert p.target_position == 65536 and custom.target_position == 1000
    assert custom.frames()[0][1][4:] == p.frames()[0][1][4:]
    assert custom.frames()[1] == p.frames()[1]


@pytest.mark.parametrize("field", ["profile_velocity", "profile_acceleration", "profile_deceleration"])
@pytest.mark.parametrize("bad", [0, -1, nan, inf, -inf, True, "1", 0.29, 10**100])
def test_invalid_unsigned_magnitude(field, bad):
    with pytest.raises(PPParameterError, match=f"Node 4.*{field}"):
        PPParameterEncoder().encode(replace(CMD, **{field: bad}))


def test_quantization_and_integer_boundaries():
    cmd = replace(CMD, target_position=0, profile_velocity=0.3, profile_acceleration=0.9)
    assert PPParameterEncoder().encode(cmd).profile_velocity == 1   # exactly 0.5 count
    assert PPParameterEncoder().encode(cmd).profile_acceleration == 2
    # With K=600 one engineering unit is one raw rate count.
    encoder = axis_encoder(4, travel_per_revolution=600, zero_offset=-(1 << 31), write_limits={})
    top = (1 << 32) - 1
    p = encoder.encode(replace(cmd, profile_velocity=top, profile_acceleration=top, profile_deceleration=top))
    assert p.frames() == ((0x204, bytes.fromhex("00 00 00 80 FF FF FF FF")), (0x304, b"\xff" * 8))
    edge_cmd = replace(cmd, profile_velocity=1, profile_acceleration=1)
    for field in ("profile_velocity", "profile_acceleration", "profile_deceleration"):
        with pytest.raises(PPParameterError, match=field):
            encoder.encode(replace(edge_cmd, **{field: top + 1}))
    with pytest.raises(RawPositionOverflowError):
        encoder.encode(replace(edge_cmd, target_position=-1))
    at_max = axis_encoder(4, zero_offset=(1 << 31) - 1)
    assert at_max.encode(cmd).frames()[0][1][:4] == bytes.fromhex("FF FF FF 7F")
    with pytest.raises(RawPositionOverflowError):
        at_max.encode(replace(cmd, target_position=1))


def test_node_type_reference_and_independent_deceleration():
    with pytest.raises(UnknownAxisError):
        PPParameterEncoder().encode(replace(CMD, node_id=9))
    for incorrect in (dict(device_type="RS00"), dict(reference_axis="电机轴")):
        with pytest.raises(PPParameterError, match="Node 4"):
            axis_encoder(4, **incorrect).encode(CMD)
    drives = make_drives({4: replace(config.AXIS_CONFIG[4], independent_deceleration=False)})
    with pytest.raises(PPParameterError, match="Node 4.*6084h"):
        PPParameterEncoder(drives).encode(CMD)


@pytest.mark.parametrize("name", ["velocity", "acceleration", "deceleration"])
def test_site_limits_apply_before_and_after_quantization(name):
    encoder = PPParameterEncoder(write_limits={4: replace(LIMITS, **{name: (1, 10)})})
    field = f"profile_{name}"
    base = PPMotionCommand(4, 0, 6, 6, 6)
    for value in (0.99, 10.01, 10):  # 10 rounds to 17 counts, or 10.2 deg/s (or deg/s²)
        with pytest.raises(PPParameterError, match=field):
            encoder.encode(replace(base, **{field: value}), for_write=True)
    assert getattr(encoder.encode(replace(base, **{field: 9.6}), for_write=True), field) == 16


@pytest.mark.parametrize("bad", [None, (1,), (nan, 10), (10, inf), (4, 1), (-1, 4), (0, 0)])
def test_invalid_limits(bad):
    encoder = PPParameterEncoder(write_limits={4: replace(LIMITS, deceleration=bad)})
    with pytest.raises(PPParameterError, match="Node 4.*deceleration"):
        encoder.encode(CMD, for_write=True)


def test_limits_and_commands_are_bound_to_the_node():
    encoder = PPParameterEncoder(write_limits={4: LIMITS})
    a = encoder.encode(CMD, for_write=True)
    with pytest.raises(PPParameterError, match="Node 5.*PP_WRITE_LIMITS"):
        encoder.encode(replace(CMD, node_id=5), for_write=True)
    b = encoder.encode(replace(CMD, node_id=5, target_position=45))
    assert b.frames()[0] == (0x205, bytes.fromhex("00 08 00 00 64 00 00 00"))
    assert a.frames()[0] == (0x204, bytes.fromhex("00 F8 FF FF 64 00 00 00"))
    with pytest.raises(PPParameterError, match="参考轴"):
        PPParameterEncoder(write_limits={4: replace(LIMITS, reference_axis="电机轴")}).encode(CMD, for_write=True)


@pytest.mark.parametrize("node, maximum, raw_velocity, raw_ramp", [
    (1, 600, 53687091, 53687), (2, 600, 53687091, 53687), (3, 600, 67815273, 67815),
    (4, 720, 1200, 1200), (5, 720, 1200, 1200), (6, 720, 1200, 1200), (7, 14, 4200, 4200),
])
def test_installed_design_limits(node, maximum, raw_velocity, raw_ramp):
    encoder = PPParameterEncoder()
    cmd = PPMotionCommand(node, 0, maximum, maximum, maximum)
    encoded = encoder.encode(cmd, for_write=True)
    assert (encoded.profile_velocity, encoded.profile_acceleration, encoded.profile_deceleration) == (
        raw_velocity, raw_ramp, raw_ramp)
    # This excess still rounds to the same count. Check the requested engineering value too.
    for field in ("profile_velocity", "profile_acceleration", "profile_deceleration"):
        with pytest.raises(PPParameterError, match=f"Node {node}.*{field}"):
            encoder.encode(replace(cmd, **{field: nextafter(maximum, inf)}), for_write=True)
