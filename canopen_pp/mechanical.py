"""Single-axis position and feedback velocity conversion in engineering units."""
from collections.abc import Mapping
from fractions import Fraction
from math import ceil, floor, isfinite

from .config import INC_VELOCITY_TIME_BASE_SECONDS, MECHANICAL_CONFIG, AxisMechanicalConfig

INT32_MIN = -(1 << 31)
INT32_MAX = (1 << 31) - 1
_HALF = Fraction(1, 2)


class MechanicalConversionError(ValueError):
    """Base class for mechanical configuration and conversion errors."""


class InvalidMechanicalConfigError(MechanicalConversionError):
    """A node's mechanical parameters are inconsistent."""


class UnknownAxisError(MechanicalConversionError):
    """The requested node has no mechanical configuration."""


class TargetSoftLimitError(MechanicalConversionError):
    """The requested mechanical target is outside its inclusive interval."""


class RawPositionOverflowError(MechanicalConversionError):
    """The raw count cannot be represented by a signed 32-bit position object."""


def _is_integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_finite_number(value: object) -> bool:
    return _is_integer(value) or (isinstance(value, float) and isfinite(value))


def _ratio(value: int | float) -> Fraction:
    # Use the displayed decimal value of engineering inputs, including at half-count ties.
    return Fraction(value) if isinstance(value, int) else Fraction(str(value))


def _display(value: object) -> str:
    try:
        return repr(value)
    except ValueError:
        # Python 3.11 limits decimal formatting of very large integers. Hex preserves all bits.
        if isinstance(value, int):
            return hex(value)
        raise


def _check_raw_range(node_id: int, raw_position: int) -> None:
    if not INT32_MIN <= raw_position <= INT32_MAX:
        raise RawPositionOverflowError(
            f"Node {node_id} raw_position={_display(raw_position)} inc 超出 [{INT32_MIN}, {INT32_MAX}] inc"
        )


def _validate_config(node_id: int, cfg: AxisMechanicalConfig) -> None:
    if not _is_integer(node_id) or not 1 <= node_id <= 127:
        raise InvalidMechanicalConfigError(f"Node {_display(node_id)} 的 node_id 必须为 1–127 的整数")
    if not isinstance(cfg, AxisMechanicalConfig):
        raise InvalidMechanicalConfigError(f"Node {node_id} 的配置必须是 AxisMechanicalConfig")
    if not _is_integer(cfg.node_id) or cfg.node_id != node_id:
        raise InvalidMechanicalConfigError(f"Node {node_id} 与配置的 node_id={_display(cfg.node_id)} 不一致")
    if cfg.unit not in ("mm", "deg"):
        raise InvalidMechanicalConfigError(f"Node {node_id} 的 unit 必须为 mm 或 deg")
    if not _is_finite_number(cfg.travel_per_revolution) or cfg.travel_per_revolution <= 0:
        raise InvalidMechanicalConfigError(f"Node {node_id} 的 travel_per_revolution 必须为有限正数")
    if not _is_integer(cfg.increments_per_revolution) or cfg.increments_per_revolution <= 0:
        raise InvalidMechanicalConfigError(f"Node {node_id} 的 increments_per_revolution 必须为正整数")
    if not _is_integer(cfg.direction) or cfg.direction not in (-1, 1):
        raise InvalidMechanicalConfigError(f"Node {node_id} 的 direction 必须为整数 +1 或 -1")
    if not _is_integer(cfg.zero_offset):
        raise InvalidMechanicalConfigError(f"Node {node_id} 的 zero_offset 必须为整数 inc")
    if not _is_finite_number(cfg.min_position):
        raise InvalidMechanicalConfigError(f"Node {node_id} 的 min_position 必须为有限数值")
    if not _is_finite_number(cfg.max_position):
        raise InvalidMechanicalConfigError(f"Node {node_id} 的 max_position 必须为有限数值")
    if _ratio(cfg.min_position) > _ratio(cfg.max_position):
        raise InvalidMechanicalConfigError(f"Node {node_id} 的 min_position 大于 max_position")


class MechanicalConverter:
    """Convert a single axis using a snapshot of its mechanical configuration."""

    def __init__(self, configs: Mapping[int, AxisMechanicalConfig] | None = None, *,
                 validate_on_init: bool = True) -> None:
        self._configs = dict(MECHANICAL_CONFIG if configs is None else configs)
        self._validate_on_access = not validate_on_init
        if validate_on_init:
            for node_id, cfg in self._configs.items():
                _validate_config(node_id, cfg)

    def get_config(self, node_id: int) -> AxisMechanicalConfig:
        if not _is_integer(node_id) or node_id not in self._configs:
            raise UnknownAxisError(f"Node {_display(node_id)} 没有机械轴配置")
        cfg = self._configs[node_id]
        if self._validate_on_access:
            _validate_config(node_id, cfg)
        return cfg

    def target_to_raw(self, node_id: int, target_position: float) -> int:
        """Validate a mechanical target and round the complete raw coordinate half away from zero."""
        cfg = self.get_config(node_id)
        if not _is_finite_number(target_position):
            raise MechanicalConversionError(f"Node {node_id} 的 target_position 必须为有限数值")
        target = _ratio(target_position)
        if not _ratio(cfg.min_position) <= target <= _ratio(cfg.max_position):
            raise TargetSoftLimitError(
                f"Node {node_id} target_position={_display(target_position)} {cfg.unit} 超出"
                f" [{_display(cfg.min_position)}, {_display(cfg.max_position)}] {cfg.unit}"
            )
        delta = target * cfg.increments_per_revolution / _ratio(cfg.travel_per_revolution)
        raw = cfg.zero_offset + cfg.direction * delta
        rounded = floor(raw + _HALF) if raw >= 0 else ceil(raw - _HALF)
        _check_raw_range(node_id, rounded)
        return rounded

    def validate_target(self, node_id: int, target_position: float) -> None:
        """Check both the mechanical target limits and the representable raw position."""
        self.target_to_raw(node_id, target_position)

    def raw_to_actual(self, node_id: int, raw_position: int) -> float:
        """Convert measured counts, preserving travel beyond the target limits and full turns."""
        cfg = self.get_config(node_id)
        if not _is_integer(raw_position):
            raise MechanicalConversionError(f"Node {node_id} 的 raw_position 必须为整数 inc")
        _check_raw_range(node_id, raw_position)
        delta = (raw_position - cfg.zero_offset) * cfg.direction
        actual = delta * _ratio(cfg.travel_per_revolution) / cfg.increments_per_revolution
        try:
            return float(actual)
        except OverflowError as exc:
            raise MechanicalConversionError(f"Node {node_id} 的实际位置超出 float 范围 ({cfg.unit})") from exc

    def raw_to_velocity(self, node_id: int, raw_velocity: int) -> float:
        """Decode signed 606Ch feedback in mm/s or deg/s at the configured reference axis."""
        cfg = self.get_config(node_id)
        if not _is_integer(raw_velocity) or not INT32_MIN <= raw_velocity <= INT32_MAX:
            raise MechanicalConversionError(
                f"Node {node_id} raw_velocity={_display(raw_velocity)} 必须为"
                f" [{INT32_MIN}, {INT32_MAX}] 内的整数"
            )
        if cfg.velocity_unit == "rpm_0_1":
            rev_per_second = Fraction(raw_velocity, 10 * 60)
        else:
            if (not isinstance(cfg.velocity_unit, str)
                    or cfg.velocity_unit not in INC_VELOCITY_TIME_BASE_SECONDS):
                raise InvalidMechanicalConfigError(
                    f"Node {node_id} 不支持的 velocity_unit={cfg.velocity_unit!r}"
                )
            seconds = INC_VELOCITY_TIME_BASE_SECONDS[cfg.velocity_unit]
            if not _is_finite_number(seconds) or seconds <= 0:
                raise InvalidMechanicalConfigError(f"Node {node_id} 的速度时间基准必须为有限正秒数")
            rev_per_second = Fraction(raw_velocity, cfg.increments_per_revolution) / _ratio(seconds)
        try:
            return float(rev_per_second * _ratio(cfg.travel_per_revolution) * cfg.direction)
        except OverflowError as exc:
            raise MechanicalConversionError(f"Node {node_id} 的实际速度超出 float 范围 ({cfg.unit}/s)") from exc
