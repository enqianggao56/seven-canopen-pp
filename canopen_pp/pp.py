"""Offline encoding of absolute Profile Position commands in engineering units."""
from dataclasses import dataclass
from fractions import Fraction
from math import floor, isfinite

from .config import MECHANICAL_CONFIG, PP_DESIGN_LIMITS
from .mechanical import InvalidMechanicalConfigError, MechanicalConverter, UnknownAxisError


@dataclass(frozen=True)
class PPCommand:
    node_id: int
    target_position: float
    velocity: float
    acceleration: float
    deceleration: float


@dataclass(frozen=True)
class PPParameters:
    target_position: int
    profile_velocity: int
    profile_acceleration: int
    profile_deceleration: int


# Counts per rev/s and per rev/s² on the configured communication axis.
# iSMD §7.3 and table 8–2 specify the motor encoder resolution as 65536.
_KINCO_ENCODER_RESOLUTION = 65536
_KINCO_PROFILE = (Fraction(60 * 512 * _KINCO_ENCODER_RESOLUTION, 1875),
                  Fraction(65536 * _KINCO_ENCODER_RESOLUTION, 4_000_000))
# RobStride §5.6 specifies 0.1 rpm and 0.1 rpm/s. CiA 402 §12.3.9
# defines 6084h in the same acceleration units as 6083h.
_ROBSTRIDE_PROFILE = (600, 600)
_PROFILE_SCALES = {"Kinco": _KINCO_PROFILE, "RS06": _ROBSTRIDE_PROFILE,
                   "RS00": _ROBSTRIDE_PROFILE, "EL05": _ROBSTRIDE_PROFILE}
_U32_MAX = (1 << 32) - 1


def _encode_rate(node: int, field: str, value: float, limit: int, scale: Fraction) -> int:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or isinstance(value, float) and not isfinite(value)):
        raise ValueError(f"Node {node} 的 {field} 必须为有限数值")
    if not 0 < value <= limit:
        raise ValueError(f"Node {node} 的 {field} 必须大于 0 且不超过 {limit}")
    # Quantize downward so the encoded magnitude cannot exceed the input or its limit.
    exact = Fraction(str(value)) * scale
    if exact > _U32_MAX:
        raise ValueError(f"Node {node} 的 {field} 超出 U32 范围")
    raw = floor(exact)
    if raw == 0:
        raise ValueError(f"Node {node} 的 {field} 小于一个原始计数")
    return raw


class PPEncoder:
    def __init__(self, converter: MechanicalConverter | None = None) -> None:
        self.converter = MechanicalConverter() if converter is None else converter

    def encode(self, command: PPCommand) -> PPParameters:
        node = command.node_id
        if isinstance(node, bool) or not isinstance(node, int) or node not in PP_DESIGN_LIMITS:
            raise UnknownAxisError("PP 命令的 Node ID 必须为 1–8 的整数")
        cfg = self.converter.get_config(node)
        installed = MECHANICAL_CONFIG[node]
        if (cfg.device_type != installed.device_type or cfg.unit != installed.unit
                or cfg.reference_axis != installed.reference_axis):
            raise InvalidMechanicalConfigError(f"Node {node} 的设备、单位或参考轴与 PP 配置不符")
        target = self.converter.target_to_raw(node, command.target_position)
        if command.target_position != 0 and target == cfg.zero_offset:
            raise ValueError(f"Node {node} 的 target_position 量化后落在机械零点")

        k = cfg.travel_per_revolution
        travel = Fraction(str(k)) if isinstance(k, float) else Fraction(k)
        velocity_scale, acceleration_scale = _PROFILE_SCALES[cfg.device_type]
        limits = PP_DESIGN_LIMITS[node]
        velocity = _encode_rate(node, "velocity", command.velocity, limits.velocity,
                                velocity_scale / travel)
        acceleration = _encode_rate(node, "acceleration", command.acceleration,
                                    limits.acceleration, acceleration_scale / travel)
        deceleration = _encode_rate(node, "deceleration", command.deceleration,
                                    limits.deceleration, acceleration_scale / travel)
        return PPParameters(target, velocity, acceleration, deceleration)
