"""Read and validate the immutable seven-axis absolute PP target set offline."""
import json
from dataclasses import dataclass
from pathlib import Path

from .config import PP_DESIGN_LIMITS
from .pp import PPCommand, PPEncoder, PPParameters

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config/pp_targets.json"
NODES = tuple(range(1, 8))
RATES = ("velocity", "acceleration", "deceleration")


@dataclass(frozen=True)
class AxisTarget:
    command: PPCommand
    torque_per_mille: int | None


@dataclass(frozen=True)
class PPTargets:
    name: str
    axes: tuple[AxisTarget, ...]

    def encode(self, encoder: PPEncoder) -> tuple[PPParameters, ...]:
        if tuple(axis.command.node_id for axis in self.axes) != NODES:
            raise ValueError("需要按顺序提供完整配置 Node 1–7")
        for axis in self.axes:
            cmd = axis.command
            n = cmd.node_id
            encoder.converter.validate_target(n, cmd.target_position)
            for field in RATES:
                limit = getattr(PP_DESIGN_LIMITS[n], field)
                value = getattr(cmd, field)
                if type(value) not in (int, float) or not 0 < value <= limit:
                    raise ValueError(f"Node {n} 的 {field} 必须为 (0, {limit}] 内的数值")
            torque = axis.torque_per_mille
            if n >= 4 and (type(torque) is not int or not 1 <= torque <= 1000):
                raise ValueError(f"Node {n} torque_per_mille 必须为 1–1000 的整数")
            if n < 4 and torque is not None:
                raise ValueError(f"Node {n} 不支持 torque_per_mille")
        return tuple(encoder.encode(axis.command) for axis in self.axes)


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


def load_targets(path: Path = CONFIG_PATH) -> PPTargets:
    doc = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
    _fields(doc, {"version", "name", "position_mode", "axes"}, "PP 配置")
    if type(doc["version"]) is not int or doc["version"] != 1:
        raise ValueError("仅支持整数 version=1")
    if doc["position_mode"] != "absolute" or not isinstance(doc["name"], str) or not doc["name"].strip():
        raise ValueError("需要非空 name 和 position_mode=absolute")
    _fields(doc["axes"], {str(n) for n in NODES}, "axes")
    axes = []
    for n in NODES:
        row = doc["axes"][str(n)]
        fields = {"position", *RATES}
        if n >= 4:
            fields.add("torque_per_mille")
        _fields(row, fields, f"Node {n}")
        cmd = PPCommand(n, row["position"], row["velocity"], row["acceleration"], row["deceleration"])
        axes.append(AxisTarget(cmd, row.get("torque_per_mille")))
    targets = PPTargets(doc["name"], tuple(axes))
    targets.encode(PPEncoder())  # Reject unrepresentable values before returning this snapshot.
    return targets
