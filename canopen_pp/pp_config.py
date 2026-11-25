"""Read and validate the immutable eight-axis absolute PP target set offline."""
import json
from dataclasses import dataclass
from pathlib import Path
from sys import float_info

from .config import AXIS_CONFIG
from .mechanical import MechanicalConverter

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config/pp_targets.json"
NODES = tuple(sorted(AXIS_CONFIG))


def validate_duration(value):
    if type(value) not in (float, int) or not 0 < value <= float_info.max:
        raise ValueError("duration_s 必须为有限正数")


@dataclass(frozen=True)
class AxisTarget:
    node_id: int
    position: float


@dataclass(frozen=True)
class PPTargets:
    duration_s: float
    axes: tuple[AxisTarget, ...]

    def selected(self, nodes):
        validate_duration(self.duration_s)
        if tuple(axis.node_id for axis in self.axes) != NODES:
            raise ValueError("需要按顺序提供完整配置 Node 1–8")
        nodes = frozenset(nodes)
        unknown = nodes.difference(NODES)
        if unknown:
            raise ValueError(f"在线节点 {sorted(unknown)} 缺少 PP 配置")
        return tuple(axis for axis in self.axes if axis.node_id in nodes)


def _unique_object(pairs):
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise ValueError(f"重复的 JSON 键 {key!r}")
        obj[key] = value
    return obj


def _fields(obj, expected, location):
    if not isinstance(obj, dict):
        raise ValueError(f"{location} 必须为 JSON 对象")
    if set(obj) != expected:
        raise ValueError(f"{location} 缺少 {sorted(expected - obj.keys())}，未知 {sorted(obj.keys() - expected)}")


def load_targets(path: Path = CONFIG_PATH, *, nodes=NODES) -> PPTargets:
    doc = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
    _fields(doc, {"duration_s", "axes"}, "PP 配置")
    _fields(doc["axes"], {str(n) for n in NODES}, "axes")
    targets = PPTargets(doc["duration_s"], tuple(AxisTarget(n, doc["axes"][str(n)]) for n in NODES))
    converter = MechanicalConverter(validate_on_init=False)
    for axis in targets.selected(nodes):
        converter.validate_target(axis.node_id, axis.position)
    return targets
