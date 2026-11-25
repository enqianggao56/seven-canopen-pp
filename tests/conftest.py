from dataclasses import replace

import pytest

from canopen_pp.config import MECHANICAL_CONFIG
from canopen_pp.pp_config import AxisTarget, PPTargets


@pytest.fixture
def reference_calibration(monkeypatch):
    """Fixed coordinates for the original conversion and wire-format examples."""
    for node, axis in MECHANICAL_CONFIG.items():
        calibrated = replace(axis, zero_offset=0, direction=1)
        if node in (5, 6):
            calibrated = replace(calibrated, max_position=-axis.min_position)
        monkeypatch.setitem(MECHANICAL_CONFIG, node, calibrated)


@pytest.fixture
def wire_targets():
    positions = (220, 40, 130, -12, 45, 0, 0, 3)
    return PPTargets(4, tuple(AxisTarget(n, pos) for n, pos in enumerate(positions, 1)))
