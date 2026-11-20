"""Single-node PP parameter encoding. Generating a frame does no CAN I/O."""
from collections.abc import Mapping
from dataclasses import dataclass
from fractions import Fraction
from math import isfinite
from struct import pack

from . import config
from .config import PPWriteLimits
from .drives import ControlError, Drive, Kinco, RobStride, make_drives
from .mechanical import MechanicalConverter
from .pdo import slots

UINT32_MAX = (1 << 32) - 1


@dataclass(frozen=True)
class PPMotionCommand:
    """Absolute target and three positive magnitudes in the axis's mechanical units."""
    node_id: int
    target_position: float
    profile_velocity: float
    profile_acceleration: float
    profile_deceleration: float


@dataclass(frozen=True)
class EncodedPPParameters:
    drive: Drive
    target_position: int
    profile_velocity: int
    profile_acceleration: int
    profile_deceleration: int

    def frames(self) -> tuple[tuple[int, bytes], tuple[int, bytes]]:
        """(standard COB-ID, payload) for RPDO1 and RPDO2 in the installed layout."""
        rp1, rp2 = slots(self.drive)[:2]
        return ((rp1.cob, pack(rp1.fmt, self.target_position, self.profile_velocity)),
                (rp2.cob, pack(rp2.fmt, self.profile_acceleration, self.profile_deceleration)))


class PPParameterError(ControlError):
    """The PP rate, device capability or commissioning configuration is invalid."""


def _number(node: int, field: str, value: object) -> Fraction:
    if type(value) is int:
        return Fraction(value)
    if type(value) is float and isfinite(value):
        return Fraction(str(value))
    raise PPParameterError(f"Node {node} {field} 必须为有限数值")


def _encode_rate(node: int, index: str, name: str, value: float, scale: Fraction,
                 limits: PPWriteLimits | None) -> int:
    field = f"{index}h profile_{name}"
    magnitude = _number(node, field, value)
    raw = magnitude * scale
    if raw <= 0 or raw > UINT32_MAX:
        raise PPParameterError(f"Node {node} {field} 原始幅值必须在 (0, {UINT32_MAX}] 内")
    # Round the exact rational once. Positive ties round up, with at most half an LSB error.
    count = int(raw + Fraction(1, 2))
    if count == 0:
        raise PPParameterError(f"Node {node} {field} 小于半个通信增量，量化后为零")
    if limits is not None:
        bounds = getattr(limits, name)
        if not isinstance(bounds, tuple) or len(bounds) != 2:
            raise PPParameterError(f"Node {node} {field} 需配置 (min, max)")
        low, high = (_number(node, f"{field} limits", x) for x in bounds)
        if not 0 <= low <= high or high <= 0:
            raise PPParameterError(f"Node {node} {field} 无效限值 {bounds}")
        if not low <= magnitude <= high or not low <= count / scale <= high:
            raise PPParameterError(f"Node {node} {field}={value} 请求值或量化值超出 {bounds}")
    return count


class PPParameterEncoder:
    def __init__(self, drives: Mapping[int, Drive] | None = None,
                 mechanical: MechanicalConverter | None = None,
                 write_limits: Mapping[int, PPWriteLimits] | None = None):
        self.drives = make_drives(config.AXIS_CONFIG) if drives is None else drives
        self.mechanical = MechanicalConverter() if mechanical is None else mechanical
        self.write_limits = config.PP_WRITE_LIMITS if write_limits is None else write_limits

    def encode(self, command: PPMotionCommand, *, for_write: bool = False) -> EncodedPPParameters:
        """Encode offline. Writing requires configured design limits for this node."""
        n = command.node_id
        axis = self.mechanical.get_config(n)
        drive = self.drives.get(n)
        if (not isinstance(drive, (Kinco, RobStride)) or drive.id != n
                or drive.axis.family != axis.device_type or drive.pp_reference_axis != axis.reference_axis):
            raise PPParameterError(f"Node {n} 驱动适配器与机械配置的设备类型或参考轴不匹配")
        if isinstance(drive, RobStride) and drive.axis.independent_deceleration is not True:
            raise PPParameterError(f"Node {n} 6084h 独立减速度未确认 (acc_status)")
        limits = self.write_limits.get(n)
        if limits is None and for_write:
            raise PPParameterError(f"Node {n} 缺少已确认的 PP_WRITE_LIMITS 和 PP 参考轴")
        if limits is not None and (not isinstance(limits, PPWriteLimits)
                                   or limits.reference_axis != axis.reference_axis):
            raise PPParameterError(f"Node {n} PP_WRITE_LIMITS 与机械参考轴不匹配")
        position = self.mechanical.target_to_raw(n, command.target_position)
        travel = _number(n, "travel_per_revolution", axis.travel_per_revolution)
        speed_scale = drive.pp_velocity_per_rps / travel
        ramp_scale = drive.pp_accel_per_rps2 / travel
        return EncodedPPParameters(
            drive, position,
            _encode_rate(n, "6081", "velocity", command.profile_velocity, speed_scale, limits),
            _encode_rate(n, "6083", "acceleration", command.profile_acceleration, ramp_scale, limits),
            _encode_rate(n, "6084", "deceleration", command.profile_deceleration, ramp_scale, limits),
        )
