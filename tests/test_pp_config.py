import json
from dataclasses import FrozenInstanceError

import pytest

from canopen_pp.pp_config import CONFIG_PATH, load_targets


def test_valid_snapshot(tmp_path):
    path = tmp_path / "targets.json"
    doc = json.loads(CONFIG_PATH.read_text())
    path.write_text(json.dumps(doc))
    targets = load_targets(path)
    path.write_text("{}")
    assert targets.duration_s == doc["duration_s"]
    assert [a.node_id for a in targets.axes] == list(range(1, 9))
    assert [a.position for a in targets.axes] == [doc["axes"][str(n)] for n in range(1, 9)]
    with pytest.raises(FrozenInstanceError):
        targets.axes[0].position = 5


@pytest.mark.parametrize(("location", "key", "value"), [
    ("axes", "8", None), ("axes", "9", 0), ("axes", "01", 0),
    ("axes", "1", 1201), ("axes", "8", {"position": 0}), ("axes", "5", True),
    ("axes", "6", float("nan")), ("axes", "2", float("inf")), ("axes", "3", "12"),
    ("root", "duration_s", None), ("root", "duration_s", True), ("root", "duration_s", 0),
    ("root", "duration_s", -1), ("root", "duration_s", float("inf")),
    ("root", "duration_s", float("nan")), ("root", "duration_s", "4"), ("root", "extra", 1),
])
def test_invalid_config(tmp_path, location, key, value):
    doc = json.loads(CONFIG_PATH.read_text())
    obj = doc if location == "root" else doc["axes"]
    if value is None:
        del obj[key]
    else:
        obj[key] = value
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError):
        load_targets(path)


def test_offline_position_skipped(tmp_path):
    doc = json.loads(CONFIG_PATH.read_text())
    doc["axes"]["8"] = 21
    path = tmp_path / "targets.json"
    path.write_text(json.dumps(doc))
    load_targets(path, nodes=[4])
    with pytest.raises(ValueError, match="Node 8"):
        load_targets(path, nodes=[4, 8])


@pytest.mark.parametrize("content", ["{", "null", "[]",
    CONFIG_PATH.read_text().replace('"1":', '"1": 0, "1":', 1),
    CONFIG_PATH.read_text().replace('"duration_s":', '"duration_s": 1, "duration_s":', 1)])
def test_malformed_config(tmp_path, content):
    path = tmp_path / "bad.json"
    path.write_text(content)
    with pytest.raises(ValueError):
        load_targets(path)
