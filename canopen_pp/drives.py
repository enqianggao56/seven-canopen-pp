"""CiA 402 power-drive states and vendor-specific PDO layout / disable policy."""
from dataclasses import dataclass

from .config import Axis


class ControlError(RuntimeError):
    pass


CIA402_STATES = {
    "Not Ready": (0x004F, 0x0000),
    "Switch On Disabled": (0x004F, 0x0040),
    "Ready to Switch On": (0x006F, 0x0021),
    "Switched On": (0x006F, 0x0023),
    "Operation Enabled": (0x006F, 0x0027),
    "Quick Stop Active": (0x006F, 0x0007),
    "Fault Reaction Active": (0x004F, 0x000F),
    "Fault": (0x004F, 0x0008),
}


def state(word):
    return next((name for name, (mask, expected) in CIA402_STATES.items()
                 if word & mask == expected), "Unknown")


@dataclass(frozen=True)
class Drive:
    id: int
    axis: Axis


class Kinco(Drive):
    eds = "motor.eds"
    vendor = 0x300
    position_index = 0x6063
    disable_word = 0x0000  # Disable Voltage, CiA 402 transitions 7, 9, 10, 12


class RobStride(Drive):
    eds = "RS06.eds"
    vendor = 0x556
    position_index = 0x6064
    disable_word = 0x0001  # RS06 / RS00 / EL05 manuals §5.3


def make_drives(axes):
    adapters = {"Kinco": Kinco, "RS06": RobStride, "RS00": RobStride, "EL05": RobStride}
    return {n: adapters[a.family](n, a) for n, a in sorted(axes.items())}
