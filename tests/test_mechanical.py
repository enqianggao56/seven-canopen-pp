"""Mechanical conversions only; these tests use no CAN adapter or drive objects."""
from dataclasses import replace
from math import inf, nan, nextafter, ulp

import pytest

from canopen_pp.config import MECHANICAL_CONFIG
from canopen_pp.mechanical import (
    InvalidMechanicalConfigError, MechanicalConversionError, MechanicalConverter,
    RawPositionOverflowError, TargetSoftLimitError, UnknownAxisError,
)


@pytest.fixture
def converter():
    return MechanicalConverter()


def customized(node, **changes):
    return MechanicalConverter({node: replace(MECHANICAL_CONFIG[node], **changes)})


@pytest.mark.parametrize("node, target, raw", [
    (1, 600, 3276800), (2, 600, 3276800), (3, 100, 689853), (4, 3, 65536), (5, 90, 4096), (6, 45, 2048),
    (7, -360, -16384), (7, -180, -8192), (7, 0, 0), (7, 180, 8192), (7, 360, 16384),
    (8, 0, 0), (8, 1, 8192), (8, 10, 81920), (8, 20, 163840),
])
def test_target_counts(converter, node, target, raw):
    assert converter.target_to_raw(node, target) == raw


@pytest.mark.parametrize("node, low, high, raw_low, raw_high", [
    (1, 0, 1200, 0, 6553600), (2, 0, 1100, 0, 6007467), (3, 0, 700, 0, 4828968),
    (4, -55, 55, -1201493, 1201493), (5, -90, 90, -4096, 4096), (6, -45, 45, -2048, 2048),
    (7, -360, 360, -16384, 16384), (8, 0, 20, 0, 163840),
])
def test_limits_apply_before_rounding(converter, node, low, high, raw_low, raw_high):
    assert converter.target_to_raw(node, low) == raw_low
    assert converter.target_to_raw(node, high) == raw_high
    assert converter.validate_target(node, high) is None
    for outside in (low - 1, high + 1, nextafter(low, -inf), nextafter(high, inf)):
        for convert in (converter.target_to_raw, converter.validate_target):
            with pytest.raises(TargetSoftLimitError) as exc:
                convert(node, outside)
            message = str(exc.value)
            assert f"Node {node}" in message and str(outside) in message
            assert f"[{low}, {high}]" in message


@pytest.mark.parametrize("node, raw, actual", [
    (1, -65536, -12), (1, 3276800, 600), (1, 6553600, 1200), (1, 6619136, 1212),
    (1, 7100000, 1300.048828125), (4, 65536, 3), (5, 8192, 180), (7, -32768, -720), (8, 180224, 22),
])
def test_actual_feedback_including_outside_target_interval(converter, node, raw, actual):
    result = converter.raw_to_actual(node, raw)
    assert isinstance(result, float)
    assert result == actual


@pytest.mark.parametrize("direction", [1, -1])
def test_zero_offset_and_direction(direction):
    c = customized(7, zero_offset=-32000, direction=direction)
    for target, delta in ((0, 0), (-180, -8192), (360, 16384)):
        raw = -32000 + direction * delta
        assert c.target_to_raw(7, target) == raw
        assert c.raw_to_actual(7, raw) == target
    assert c.raw_to_actual(7, -32000 + direction * 32768) == 720


@pytest.mark.parametrize("zero, target, expected", [
    (0, 0.05, 1), (0, -0.05, -1), (0, 0.25, 3), (0, -0.25, -3),
    (-1, 0.05, -1), (1, -0.05, 1),
    (0, nextafter(0.05, 0.0), 0), (0, nextafter(-0.05, 0.0), 0),
])
def test_half_away_from_zero_on_complete_raw_coordinate(zero, target, expected):
    c = customized(5, increments_per_revolution=10, travel_per_revolution=1, zero_offset=zero)
    assert c.target_to_raw(5, target) == expected


@pytest.mark.parametrize("param, value", [
    ("travel_per_revolution", 0), ("travel_per_revolution", -1), ("travel_per_revolution", nan),
    ("travel_per_revolution", inf), ("increments_per_revolution", 0), ("increments_per_revolution", -1),
    ("increments_per_revolution", 2.5), ("increments_per_revolution", True),
    ("direction", 0), ("direction", 2), ("direction", 1.0), ("direction", True),
    ("zero_offset", 1.5), ("zero_offset", False), ("min_position", nan),
    ("max_position", inf), ("min_position", 91), ("unit", "rpm"), ("node_id", 6),
])
def test_invalid_config_names_node_and_field(param, value):
    with pytest.raises(InvalidMechanicalConfigError, match=f"Node 5.*{param}"):
        customized(5, **{param: value})


def test_single_allowed_target_and_config_snapshot():
    config = {5: replace(MECHANICAL_CONFIG[5], min_position=45, max_position=45)}
    c = MechanicalConverter(config)
    config.clear()
    assert c.get_config(5).unit == "deg"
    assert c.target_to_raw(5, 45) == 2048
    assert c.raw_to_actual(5, 0) == 0
    with pytest.raises(TargetSoftLimitError):
        c.target_to_raw(5, 44)


@pytest.mark.parametrize("node", [0, 9, True, 1.0, "1"])
def test_unknown_axis(converter, node):
    for operation in (converter.target_to_raw, converter.raw_to_actual, converter.validate_target,
                      converter.raw_to_velocity):
        with pytest.raises(UnknownAxisError, match="Node"):
            operation(node, 0)
    with pytest.raises(UnknownAxisError, match="Node"):
        converter.get_config(node)


@pytest.mark.parametrize("value", [nan, inf, -inf, "12", True, None])
def test_invalid_target(converter, value):
    with pytest.raises(MechanicalConversionError, match="Node 1.*target_position"):
        converter.target_to_raw(1, value)


@pytest.mark.parametrize("value", [1.0, False, "12", None, nan, inf])
def test_noninteger_feedback(converter, value):
    with pytest.raises(MechanicalConversionError, match="Node 1.*raw_position"):
        converter.raw_to_actual(1, value)


@pytest.mark.parametrize("edge, sign", [(-2147483648, -1), (2147483647, 1)])
def test_int32_edges_and_rounded_overflow(edge, sign):
    c = customized(5, zero_offset=edge, travel_per_revolution=1, increments_per_revolution=1)
    assert c.raw_to_actual(5, edge) == 0
    assert c.target_to_raw(5, sign * 0.49) == edge
    overflow = edge + sign
    for operation, value in ((c.raw_to_actual, overflow), (c.target_to_raw, sign * 0.5),
                             (c.validate_target, sign * 0.5)):
        with pytest.raises(RawPositionOverflowError) as exc:
            operation(5, value)
        assert str(overflow) in str(exc.value)
        assert "Node 5" in str(exc.value) and "[-2147483648, 2147483647]" in str(exc.value)


def test_extreme_scale_avoids_intermediate_float_overflow():
    c = customized(5, travel_per_revolution=1e308, increments_per_revolution=10**308)
    assert c.target_to_raw(5, 10) == 10
    assert c.raw_to_actual(5, -2147483648) == -2147483648.0
    c = customized(5, travel_per_revolution=1e308, increments_per_revolution=1)
    with pytest.raises(MechanicalConversionError, match="Node 5.*float"):
        c.raw_to_actual(5, 2)


def test_large_python_integers_report_domain_errors(converter):
    huge = 10**5000
    with pytest.raises(TargetSoftLimitError, match="Node 1"):
        converter.target_to_raw(1, huge)
    with pytest.raises(RawPositionOverflowError, match="Node 1"):
        converter.raw_to_actual(1, huge)


@pytest.mark.parametrize("node, targets", [
    (1, [0.00013, 951.325]), (2, [0.0017, 1100]), (3, [9.51, 699.91]),
    (4, [-54.99, 20.11]), (5, [-75.672, 20.11]), (6, [-44.039, 43.021]), (7, [-341.123, 359.99]),
    (8, [0.0063, 19.995, 1 / 16384]),
])
def test_round_trip_within_half_an_increment(converter, node, targets):
    cfg = converter.get_config(node)
    tolerance = cfg.travel_per_revolution / (2 * cfg.increments_per_revolution)
    for target in targets:
        actual = converter.raw_to_actual(node, converter.target_to_raw(node, target))
        assert abs(actual - target) <= tolerance + ulp(target)


@pytest.mark.parametrize("node, raw, expected", [
    (1, 65536, 12), (1, 32768, 6), (2, 65536, 12), (3, 65536, 9.5), (3, 32768, 4.75),
])
def test_explicit_inc_per_second(node, raw, expected):
    c = customized(node, velocity_unit="inc_per_second")
    assert c.raw_to_velocity(node, raw) == expected


@pytest.mark.parametrize("node, raw, expected", [
    (1, 65536, 0.732421875), (2, 32768, 0.3662109375), (3, 65536, 0.579833984375),
    (1, 1048576, 11.71875),   # 58.59375 rpm per the iSMD §7.2 DEC formula
    (4, 65536, 0.18310546875), (5, 100, 60), (5, 50, 30), (6, 100, 60), (7, 200, 120), (8, 300, 1), (8, 600, 2),
])
def test_feedback_velocity_sign_and_direction(converter, node, raw, expected):
    assert converter.raw_to_velocity(node, raw) == expected
    assert converter.raw_to_velocity(node, -raw) == -expected
    c = customized(node, zero_offset=100000, direction=-1)
    assert c.raw_to_velocity(node, 0) == 0
    assert c.raw_to_velocity(node, raw) == -expected
    assert c.raw_to_velocity(node, -raw) == expected


def test_velocity_rule_comes_from_config():
    c = customized(1, velocity_unit="rpm_0_1", min_position=0, max_position=0)
    assert c.raw_to_velocity(1, 100) == 2
    c = customized(5, velocity_unit="inc_per_second", zero_offset=100000, direction=-1)
    assert c.raw_to_velocity(5, 16384) == -360


def test_velocity_int32_edges(converter):
    assert converter.raw_to_velocity(1, -2147483648) == -24000
    assert converter.raw_to_velocity(7, 2147483647) == pytest.approx(1288490188.2)


@pytest.mark.parametrize("value", [True, None, 1.0, "123", nan, inf, -2147483649, 2147483648])
def test_invalid_raw_velocity(converter, value):
    with pytest.raises(MechanicalConversionError, match="Node 1.*raw_velocity"):
        converter.raw_to_velocity(1, value)


def test_unknown_velocity_unit_preserves_position():
    c = customized(5, velocity_unit="unknown")
    assert c.raw_to_actual(5, 8192) == 180
    with pytest.raises(InvalidMechanicalConfigError, match="Node 5.*velocity_unit.*unknown"):
        c.raw_to_velocity(5, 100)


def test_velocity_float_overflow():
    c = customized(5, travel_per_revolution=1e308)
    with pytest.raises(MechanicalConversionError, match="Node 5.*float"):
        c.raw_to_velocity(5, 2000)
