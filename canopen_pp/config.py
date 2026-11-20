"""Master timing, installed drives and mechanical coordinates keyed by node ID."""
from dataclasses import dataclass

CAN_CONFIG = dict(interface="socketcan", channel="can0", bitrate=1_000_000,
                  sync_period_ms=100)
INITIAL_SDO_TIMEOUT = 1.0
SDO_TIMEOUT = 0.30
FEEDBACK_TIMEOUT = 0.60
TRANSITION_TIMEOUT = 2.0

# Equivalent seconds in each inc/time-base unit of 606Ch (independent of SYNC).
# iSMD §7.2: DEC = RPM * 512 * encoder_counts_per_rev / 1875.
INC_VELOCITY_TIME_BASE_SECONDS = {"kinco_606c_dec": 16.384, "inc_per_second": 1}


@dataclass(frozen=True)
class Axis:
    family: str
    name: str
    pp_torque_per_mille: int | None = None  # RobStride 6071h, 1000 = 100%
    independent_deceleration: bool = False  # RobStride PP acc_status=1 on this node


AXIS_CONFIG = {
    1: Axis("Kinco", "横移"),
    2: Axis("Kinco", "上下"),
    3: Axis("Kinco", "伸缩"),
    4: Axis("RS06", "小臂上下旋转", pp_torque_per_mille=1000, independent_deceleration=True),
    5: Axis("RS00", "小臂左右旋转", pp_torque_per_mille=1000, independent_deceleration=True),
    6: Axis("EL05", "绕轴旋转", pp_torque_per_mille=1000, independent_deceleration=True),
    7: Axis("EL05", "夹爪", pp_torque_per_mille=1000, independent_deceleration=True),
}


@dataclass(frozen=True)
class PPWriteLimits:
    """Confirmed PP reference axis and mechanical design limits for one installation."""
    reference_axis: str  # confirmed reference of the drive's PP objects
    velocity: tuple[float, float]      # mm/s or deg/s
    acceleration: tuple[float, float]  # mm/s² or deg/s²
    deceleration: tuple[float, float]  # mm/s² or deg/s²


# Positive PP magnitudes, in mm or degrees per s (velocity) and per s² (ramps).
# For all four RobStride drives one PP revolution is one output-flange revolution.
PP_WRITE_LIMITS: dict[int, PPWriteLimits] = {
    1: PPWriteLimits("电机轴", (0, 600), (0, 600), (0, 600)),
    2: PPWriteLimits("电机轴", (0, 600), (0, 600), (0, 600)),
    3: PPWriteLimits("电机轴", (0, 600), (0, 600), (0, 600)),
    4: PPWriteLimits("输出法兰", (0, 720), (0, 720), (0, 720)),
    5: PPWriteLimits("输出法兰", (0, 720), (0, 720), (0, 720)),
    6: PPWriteLimits("输出法兰", (0, 720), (0, 720), (0, 720)),
    7: PPWriteLimits("输出法兰", (0, 14), (0, 14), (0, 14)),
}


@dataclass(frozen=True)
class AxisMechanicalConfig:
    node_id: int
    name: str
    device_type: str
    reference_axis: str
    unit: str
    travel_per_revolution: float
    increments_per_revolution: int
    min_position: float
    max_position: float
    zero_offset: int = 0
    direction: int = 1
    velocity_unit: str | None = None  # raw 606Ch unit, optional for position-only configurations

    @property
    def mechanism_type(self) -> str:
        return {"mm": "linear", "deg": "rotary"}[self.unit]


MECHANICAL_CONFIG = {
    n: AxisMechanicalConfig(n, AXIS_CONFIG[n].name, AXIS_CONFIG[n].family, reference, unit,
                            travel, increments, low, high, zero, direction, velocity_unit)
    for n, reference, unit, travel, increments, low, high, zero, direction, velocity_unit in (
        # ID, reference axis, unit, K/rev, C/rev, min, max, zero (inc), direction, 606Ch unit
        (1, "电机轴",   "mm",   12, 65536,    0, 1200, 0, 1, "kinco_606c_dec"),
        (2, "电机轴",   "mm",   12, 65536,    0, 1100, 0, 1, "kinco_606c_dec"),
        (3, "电机轴",   "mm",  9.5, 65536,    0,  700, 0, 1, "kinco_606c_dec"),
        (4, "输出法兰", "deg", 360, 16384,  -90,   90, 0, 1, "rpm_0_1"),
        (5, "输出法兰", "deg", 360, 16384,  -45,   45, 0, 1, "rpm_0_1"),
        (6, "输出法兰", "deg", 360, 16384, -360,  360, 0, 1, "rpm_0_1"),
        (7, "输出法兰", "mm",    2, 16384,    0,   20, 0, 1, "rpm_0_1"),
    )
}


@dataclass(frozen=True)
class DebugLimits:
    """Software ceilings in axis units, independent of the mechanical design limits."""
    travel: float
    velocity: float
    acceleration: float
    deceleration: float
    position_window: float
    stopped_velocity: float


DEBUG_LIMITS = {
    1: DebugLimits(1, 1, 2, 2, .02, .1),
    2: DebugLimits(1, 1, 2, 2, .02, .1),
    3: DebugLimits(1, 1, 2, 2, .02, .1),
    4: DebugLimits(3, 3, 6, 6, .10, .6),
    5: DebugLimits(3, 3, 6, 6, .10, .6),
    6: DebugLimits(3, 3, 6, 6, .10, .6),
    7: DebugLimits(.2, .2, .5, .5, .005, .01),
}
DEBUG_MAX_TORQUE_PER_MILLE = 100
DEBUG_MOTION_TIMEOUT = 20.0
DEBUG_STOP_TIMEOUT = 4.0


@dataclass(frozen=True)
class BenchStop:
    """Evidence for this installed axis, load, stop word and expected final 402 state."""
    word: int
    final_state: str
    evidence: str
    # Kinco: approved raw ceiling for 6073h. RobStride: approved 6071h ceiling in ‰.
    effort_limit: int


DEBUG_VERIFIED_STOPS: dict[int, BenchStop] = {}
