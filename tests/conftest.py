from dataclasses import replace

import pytest

from canopen_pp.config import MECHANICAL_CONFIG


@pytest.fixture
def reference_calibration(monkeypatch):
    """Fixed coordinates for the original conversion and wire-format examples."""
    for node, axis in MECHANICAL_CONFIG.items():
        calibrated = replace(axis, zero_offset=0, direction=1)
        if node in (5, 6):
            calibrated = replace(calibrated, max_position=-axis.min_position)
        monkeypatch.setitem(MECHANICAL_CONFIG, node, calibrated)
