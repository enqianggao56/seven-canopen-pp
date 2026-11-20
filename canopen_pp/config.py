"""Master timing and installed drives. Node IDs select the model."""
from dataclasses import dataclass

CAN_CONFIG = dict(interface="socketcan", channel="can0", bitrate=1_000_000,
                  sync_period_ms=100)
INITIAL_SDO_TIMEOUT = 1.0
SDO_TIMEOUT = 0.30
FEEDBACK_TIMEOUT = 0.60
TRANSITION_TIMEOUT = 2.0


@dataclass(frozen=True)
class Axis:
    family: str
    name: str


AXIS_CONFIG = {
    1: Axis("Kinco", "横移"),
    2: Axis("Kinco", "上下"),
    3: Axis("Kinco", "伸缩"),
    4: Axis("RS06", "小臂上下旋转"),
    5: Axis("RS00", "小臂左右旋转"),
    6: Axis("EL05", "绕轴旋转"),
    7: Axis("EL05", "夹爪"),
}
