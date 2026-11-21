import json
from dataclasses import FrozenInstanceError

import pytest

from canopen_pp.pp_config import CONFIG_PATH, load_targets


def test_valid_snapshot(tmp_path):
    path = tmp_path / "targets.json"
    path.write_bytes(CONFIG_PATH.read_bytes())
    targets = load_targets(path)
    path.write_text("{}")
    assert targets.name == "seven_axis_pp_test"
    assert [axis.command.node_id for axis in targets.axes] == list(range(1, 8))
    assert targets.axes[-1].torque_per_mille == 100
    with pytest.raises(FrozenInstanceError):
        targets.axes[0].command.velocity = 5


@pytest.mark.parametrize(("location", "key", "value"), [
    ("axes", "7", None), ("axes", "8", {}), ("axes", "01", {}),
    ("1", "position", 1201), ("7", "velocity", 15), ("6", "acceleration", 721),
    ("2", "deceleration", 601), ("5", "velocity", 0.0001),
    ("4", "torque_per_mille", 0), ("4", "torque_per_mille", 1001),
    ("4", "torque_per_mille", 100.0), ("4", "torque_per_mille", True),
    ("7", "torque_per_mille", None), ("1", "torque_per_mille", 100),
    ("5", "position", float("nan")), ("2", "velocity", float("inf")),
    ("3", "acceleration", -float("inf")), ("4", "deceleration", True),
    ("3", "position", "12"), ("1", "velocity", None), ("6", "extra", 0),
    ("root", "position_mode", "relative"), ("root", "version", True),
    ("root", "name", " "), ("root", "extra", 1),
])
def test_invalid_config(tmp_path, location, key, value):
    doc = json.loads(CONFIG_PATH.read_text())
    obj = doc if location == "root" else doc["axes"]
    if location.isdigit():
        obj = obj[location]
    if value is None:
        del obj[key]
    else:
        obj[key] = value
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError):
        load_targets(path)


@pytest.mark.parametrize("content", ["{", "null", "[]",
    CONFIG_PATH.read_text().replace('"1":', '"1": {}, "1":', 1),
    CONFIG_PATH.read_text().replace('"position":', '"position": 0, "position":', 1)])
def test_malformed_config(tmp_path, content):
    path = tmp_path / "bad.json"
    path.write_text(content)
    with pytest.raises(ValueError):
        load_targets(path)
