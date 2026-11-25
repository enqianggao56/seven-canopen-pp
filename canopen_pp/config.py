"""Master timing, installed drives and mechanical coordinates keyed by node ID."""
from dataclasses import dataclass
import os

PARAMETER_TEST_ONLY = os.environ.get("CANOPEN_PP_MOTION") != "1"
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


AXIS_CONFIG = {
    1: Axis("Kinco", "横移"),
    2: Axis("Kinco", "上下"),
    3: Axis("Kinco", "伸缩"),
    4: Axis("Kinco", "大臂旋转"),
    5: Axis("RS06", "小臂上下旋转"),
    6: Axis("RS00", "小臂左右旋转"),
    7: Axis("EL05", "绕轴旋转"),
    8: Axis("EL05", "夹爪"),
}


@dataclass(frozen=True)
class PPDesignLimits:
    velocity: int
    acceleration: int
    deceleration: int


# Engineering units on the mechanical axis (mm or deg, per s and per s²).
PP_DESIGN_LIMITS = {
    1: PPDesignLimits(600, 600, 600),
    2: PPDesignLimits(600, 600, 600),
    3: PPDesignLimits(600, 600, 600),
    4: PPDesignLimits(150, 150, 150),
    5: PPDesignLimits(720, 720, 720),
    6: PPDesignLimits(720, 720, 720),
    7: PPDesignLimits(720, 720, 720),
    8: PPDesignLimits(14, 14, 14),
}


# Allowed position error in the mechanical axis unit (mm or deg).
PP_POSITION_TOLERANCES = {
    1: .02, 2: .02, 3: .02,
    4: .05, 5: .05, 6: .05, 7: .05,
    8: .005,
}

# Statusword bits required in addition to actual position feedback.
PP_ACK_NODES = frozenset({1, 2, 3, 4})
PP_TARGET_REACHED_NODES = frozenset({1, 2, 3, 4})
PP_ACK_TIMEOUT = 2.0
PP_SETTLING_MARGIN = 3.0


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
        (1, "电机轴",   "mm",   12, 65536,    0, 1200, 2130340, -1, "kinco_606c_dec"),
        (2, "电机轴",   "mm",   12, 65536,    0, 1100, -327735, 1, "kinco_606c_dec"),
        (3, "电机轴",   "mm",  9.5, 65536,    0,  700, 4766881, -1, "kinco_606c_dec"),
        (4, "电机轴",   "deg",   3, 65536,  -55,   55, -4564877, -1, "kinco_606c_dec"),  # 120:1
        (5, "输出法兰", "deg", 360, 16384,  -90,   130, 0, -1, "rpm_0_1"),
        (6, "输出法兰", "deg", 360, 16384,  -45,   90, 0, -1, "rpm_0_1"),
        (7, "输出法兰", "deg", 360, 16384, -360,  360, 17472, 1, "rpm_0_1"),
        (8, "输出法兰", "mm",    2, 16384,    0,   20, 0, 1, "rpm_0_1"),
    )
}

# 6071h at enable and during PP. The disable word always carries zero torque.
PP_TORQUE_PER_MILLE = {5: 600, 6: 500, 7: 700, 8: 500}

# 各阶段共用的静止速度上限，按 |606Ch 换算后的机械速度| ≤ 阈值判断。
PP_VELOCITY_TOLERANCES = {
    1: 5.0, 2: 5.0, 3: 5.0,  # mm/s
    4: 5.0, 5: 5.0, 6: 5.0, 7: 5.0,  # °/s
    8: 5.0,  # mm/s
}
PP_ARRIVAL_TOLERANCE_S = 2.0  # both inter-axis spread and error relative to T
