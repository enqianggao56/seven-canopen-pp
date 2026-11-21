from dataclasses import replace
from math import inf, nan, nextafter
from unittest.mock import Mock

import canopen
import pytest

from canopen_pp.config import MECHANICAL_CONFIG, PP_DESIGN_LIMITS, PPDesignLimits
from canopen_pp.mechanical import (
    InvalidMechanicalConfigError, MechanicalConverter, RawPositionOverflowError,
    TargetSoftLimitError, UnknownAxisError,
)
from canopen_pp.pp import PPCommand, PPEncoder, PPParameters


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    connect = Mock(side_effect=AssertionError("offline encoder opened CAN"))
    send = Mock(side_effect=AssertionError("offline encoder sent CAN"))
    monkeypatch.setattr(canopen.Network, "connect", connect)
    monkeypatch.setattr(canopen.Network, "send_message", send)
    yield
    connect.assert_not_called()
    send.assert_not_called()


def customized(node, **changes):
    cfg = replace(MECHANICAL_CONFIG[node], **changes)
    return PPEncoder(MechanicalConverter({node: cfg}))


@pytest.mark.parametrize("node, target, limit, raw", [
    (1, 1200, 600, PPParameters(6553600, 53687091, 53687, 53687)),
    (2, 1100, 600, PPParameters(6007467, 53687091, 53687, 53687)),
    (3, 700, 600, PPParameters(4828968, 67815273, 67815, 67815)),
    (4, -90, 720, PPParameters(-4096, 1200, 1200, 1200)),
    (5, -45, 720, PPParameters(-2048, 1200, 1200, 1200)),
    (6, -360, 720, PPParameters(-16384, 1200, 1200, 1200)),
    (7, 20, 14, PPParameters(163840, 4200, 4200, 4200)),
])
def test_all_axes_at_design_limits(node, target, limit, raw):
    encoder = PPEncoder()
    command = PPCommand(node, target, limit, limit, limit)
    assert encoder.encode(command) == raw
    for field in ("velocity", "acceleration", "deceleration"):
        with pytest.raises(ValueError, match=field):
            encoder.encode(replace(command, **{field: nextafter(limit, inf)}))


@pytest.mark.parametrize("command, expected", [
    (PPCommand(1, 12, 40, 120, 300), PPParameters(65536, 0x369D03, 10737, 26843)),
    (PPCommand(4, -45, 60, 12, 3), PPParameters(-2048, 100, 20, 5)),
    (PPCommand(7, 1, 2, 1, 0.5), PPParameters(8192, 600, 300, 150)),
])
def test_independent_profile_fields_ignore_feedback_units(command, expected):
    encoder = customized(command.node_id, velocity_unit=None)
    assert encoder.encode(command) == expected


def test_kinco_canopen_manual_example():
    # One mechanical unit per motor turn makes the manual's 100 rev/s²
    # example representable below the mechanical design limit in this fixture.
    encoder = customized(1, travel_per_revolution=1)
    params = encoder.encode(PPCommand(1, 0, 1, 100, 100))
    assert params.profile_acceleration == params.profile_deceleration == 0x1A36E


@pytest.mark.parametrize("node", [False, True, 1.0, "1", 0, 8, -1, None, []])
def test_invalid_node(node):
    with pytest.raises(UnknownAxisError):
        PPEncoder().encode(PPCommand(node, 0, 1, 1, 1))


@pytest.mark.parametrize("field", ["target_position", "velocity", "acceleration", "deceleration"])
@pytest.mark.parametrize("value", [False, True, "1", None, nan, inf, -inf])
def test_invalid_number(field, value):
    with pytest.raises(ValueError, match=field):
        PPEncoder().encode(replace(PPCommand(1, 0, 1, 1, 1), **{field: value}))


@pytest.mark.parametrize("field", ["velocity", "acceleration", "deceleration"])
def test_positive_and_quantized_rate_boundaries(field):
    encoder = PPEncoder()
    command = PPCommand(4, 0, 1, 1, 1)
    for value in (-1, 0, nextafter(0.6, 0), 10**5000):
        with pytest.raises(ValueError, match=field):
            encoder.encode(replace(command, **{field: value}))
    for value in (0.6, 0.9, nextafter(1.2, 0)):
        result = encoder.encode(replace(command, **{field: value}))
        assert getattr(result, "profile_" + field) == 1


def test_position_limits_direction_and_quantization():
    encoder = customized(6, direction=-1, zero_offset=8192)
    command = PPCommand(6, -360, 6, 12, 60)
    assert encoder.encode(command) == PPParameters(24576, 10, 20, 100)
    assert encoder.encode(replace(command, target_position=180)).target_position == 0
    assert encoder.encode(replace(command, target_position=0)).target_position == 8192
    with pytest.raises(TargetSoftLimitError):
        encoder.encode(replace(command, target_position=nextafter(-360, -inf)))
    with pytest.raises(ValueError, match="target_position"):
        encoder.encode(replace(command, target_position=0.0001))


def test_i32_position_overflow_propagates():
    encoder = customized(4, zero_offset=2**31 - 1)
    assert encoder.encode(PPCommand(4, 0, 1, 1, 1)).target_position == 2**31 - 1
    with pytest.raises(RawPositionOverflowError):
        encoder.encode(PPCommand(4, 1, 1, 1, 1))


@pytest.mark.parametrize("field", ["velocity", "acceleration", "deceleration"])
def test_u32_edge_and_overflow(monkeypatch, field):
    maximum = 2**32 - 1
    # One raw count per engineering unit, with a design range beyond U32.
    encoder = customized(4, travel_per_revolution=600)
    limits = replace(PPDesignLimits(1, 1, 1), **{field: maximum + 1})
    monkeypatch.setitem(PP_DESIGN_LIMITS, 4, limits)
    command = replace(PPCommand(4, 0, 1, 1, 1), **{field: maximum})
    assert getattr(encoder.encode(command), "profile_" + field) == maximum
    with pytest.raises(ValueError, match=field + ".*U32"):
        encoder.encode(replace(command, **{field: maximum + 1}))


@pytest.mark.parametrize("changes", [
    {"device_type": "Kinco"}, {"unit": "mm"}, {"reference_axis": "电机轴"},
])
def test_incompatible_mechanical_configuration(changes):
    with pytest.raises(InvalidMechanicalConfigError):
        customized(4, **changes).encode(PPCommand(4, 0, 1, 1, 1))
