import json
from dataclasses import FrozenInstanceError

import pytest

from canopen_pp.pp_config import CONFIG_PATH, load_targets


def test_valid_snapshot(tmp_path):
    path = tmp_path / "targets.json"
    path.write_bytes(CONFIG_PATH.read_bytes())
    targets = load_targets(path)
    path.write_text("{}")
    assert targets.name == "eight_axis_pp_test"
    assert [axis.command.node_id for axis in targets.axes] == list(range(1, 9))
    assert [axis.torque_per_mille for axis in targets.axes] == [None] * 4 + [100] * 4
    with pytest.raises(FrozenInstanceError):
        targets.axes[0].command.velocity = 5


@pytest.mark.parametrize(("location", "key", "value"), [
    ("axes", "8", None), ("axes", "9", {}), ("axes", "01", {}),
    ("1", "position", 1201), ("8", "velocity", 15), ("7", "acceleration", 721),
    ("2", "deceleration", 601), ("6", "velocity", 0.0001),
    ("5", "torque_per_mille", 0), ("5", "torque_per_mille", 1001),
    ("5", "torque_per_mille", 100.0), ("5", "torque_per_mille", True),
    ("8", "torque_per_mille", None), ("1", "torque_per_mille", 100), ("4", "torque_per_mille", 100),
    ("6", "position", float("nan")), ("2", "velocity", float("inf")),
    ("3", "acceleration", -float("inf")), ("5", "deceleration", True),
    ("3", "position", "12"), ("1", "velocity", None), ("7", "extra", 0),
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


def test_offline_torque_value_is_skipped_but_field_is_required(tmp_path):
    doc = json.loads(CONFIG_PATH.read_text())
    path = tmp_path / "targets.json"
    gripper = doc["axes"]["8"]
    gripper["torque_per_mille"] = 0
    path.write_text(json.dumps(doc))
    load_targets(path, nodes=[4])
    with pytest.raises(ValueError, match="Node 8.*torque_per_mille"):
        load_targets(path, nodes=[4, 8])
    del gripper["torque_per_mille"]
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="Node 8.*torque_per_mille"):
        load_targets(path, nodes=[4])


@pytest.mark.parametrize("content", ["{", "null", "[]",
    CONFIG_PATH.read_text().replace('"1":', '"1": {}, "1":', 1),
    CONFIG_PATH.read_text().replace('"position":', '"position": 0, "position":', 1)])
def test_malformed_config(tmp_path, content):
    path = tmp_path / "bad.json"
    path.write_text(content)
    with pytest.raises(ValueError):
        load_targets(path)
