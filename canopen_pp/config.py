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


AXIS_CONFIG = {
    1: Axis("Kinco", "横移"),
    2: Axis("Kinco", "上下"),
    3: Axis("Kinco", "伸缩"),
    4: Axis("RS06", "小臂上下旋转", pp_torque_per_mille=1000),
    5: Axis("RS00", "小臂左右旋转", pp_torque_per_mille=1000),
    6: Axis("EL05", "绕轴旋转", pp_torque_per_mille=1000),
    7: Axis("EL05", "夹爪", pp_torque_per_mille=1000),
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
