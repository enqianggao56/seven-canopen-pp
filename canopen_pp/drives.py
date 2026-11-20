"""CiA 402 power-drive states and vendor-specific PDO layout / disable policy."""
from dataclasses import dataclass
from fractions import Fraction
from struct import Struct

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
    control_fields = ((0x6040, 6), (0x6060, 2))
    control_struct = Struct("<Hb")

    def control_data(self, word):
        return self.control_struct.pack(word, 1)


class Kinco(Drive):
    eds = "motor.eds"
    vendor = 0x300
    position_index = 0x6063
    disable_word = 0x0000  # Disable Voltage, CiA 402 transitions 7, 9, 10, 12
    # iSMD §7.3 and table 8-2: encoder resolution for the DEC formulas.
    # This is distinct from the configurable mechanical position increment count.
    encoder_resolution = 65536
    pp_velocity_per_rps = Fraction(60 * 512 * encoder_resolution, 1875)
    pp_accel_per_rps2 = Fraction(65536 * encoder_resolution, 4_000_000)
    pp_reference_axis = "电机轴"


@dataclass(frozen=True)
class RobStride(Drive):
    eds = "RS06.eds"
    vendor = 0x556
    position_index = 0x6064
    disable_word = 0x0001  # RS06 / RS00 / EL05 manuals §5.3
    control_fields = Drive.control_fields + ((0x6071, 3),)
    control_struct = Struct("<Hbh")
    # PP §5.6: 0.1 rpm and 0.1 rpm/s. One rev/s corresponds to 600 counts.
    pp_velocity_per_rps = 60 * 10
    pp_accel_per_rps2 = 60 * 10
    pp_reference_axis = "输出法兰"  # installed PP reference is 1:1 with the output flange

    def __post_init__(self):
        limit = self.axis.pp_torque_per_mille
        if type(limit) is not int or not 1 <= limit <= 1000:
            raise ControlError(f"Node {self.id} 的 PP 力矩上限必须为 1–1000 的整数（‰）")

    def control_data(self, word):
        limit = 0 if word == self.disable_word else self.axis.pp_torque_per_mille
        return self.control_struct.pack(word, 1, limit)


def make_drives(axes):
    adapters = {"Kinco": Kinco, "RS06": RobStride, "RS00": RobStride, "EL05": RobStride}
    return {n: adapters[a.family](n, a) for n, a in sorted(axes.items())}
