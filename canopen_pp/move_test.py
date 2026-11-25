"""Pack PP parameters using the installed PDO layouts and report disabled test batches."""
from dataclasses import dataclass
import struct

from . import config
from .drives import state
from .pdo import slots
from .pp import PPParameters, PPCommand


@dataclass(frozen=True)
class PreparedAxis:
    command: PPCommand
    torque_per_mille: int | None
    unit: str
    params: PPParameters
    frames: tuple[tuple[int, bytes], ...]


def prepare(trajectory, drives, encoder, control=None):
    axes = []
    for n, profile in trajectory.moving.items():
        torque = config.PP_TORQUE_PER_MILLE.get(n)
        p = profile.params
        drive = drives[n]
        rp1, rp2, rp3 = slots(drive)[:3]
        word = drive.disable_word if control is None else control
        frames = (
            (rp1.cob, struct.pack(rp1.fmt, p.target_position, p.profile_velocity)),
            (rp2.cob, struct.pack(rp2.fmt, p.profile_acceleration, p.profile_deceleration)),
            (rp3.cob, drive.control_data(word, torque)),
        )
        axes.append(PreparedAxis(profile.command, torque, encoder.converter.get_config(n).unit, p, frames))
    return axes


def batch_report(axes, echoes, samples, error):
    lines = []
    stamps = dict(echoes)
    counts = [0, 0, 0]
    for axis in axes:
        cmd, p, unit = axis.command, axis.params, axis.unit
        lines.extend((
            f"Node {cmd.node_id}", "  Physical Parameters",
            f"    Position: {cmd.target_position} {unit}",
            f"    Velocity: {cmd.velocity} {unit}/s",
            f"    Acceleration: {cmd.acceleration} {unit}/s²",
            f"    Deceleration: {cmd.deceleration} {unit}/s²",
            "  Encoded Parameters",
            f"    607A: {p.target_position}    6081: {p.profile_velocity}",
            f"    6083: {p.profile_acceleration}    6084: {p.profile_deceleration}",
        ))
        for i, (cob, data) in enumerate(axis.frames):
            echo = f"{stamps[cob]:.6f}" if cob in stamps else "unconfirmed"
            counts[i] += int(cob in stamps)
            lines.append(f"  RPDO{i + 1}  CAN-ID: 0x{cob:03X}  DLC: {len(data)}  "
                         f"DATA: {data.hex(' ').upper()}  TX echo: {echo}")
        if axis.torque_per_mille is not None:
            cob, data = axis.frames[2]
            torque = int.from_bytes(data[3:5], "little", signed=True)
            lines.extend((f"  Configured Torque: {axis.torque_per_mille} ‰",
                          f"  RPDO3 Torque: {torque} ‰ ({'TX echo' if cob in stamps else 'prepared'})",
                          "  Reason: Drive Disabled"))
    lines.extend(("", "SYNC RPDO BATCH RESULT (TX echo counts)"))
    lines.extend(f"RPDO{i} Frames: {count}" for i, count in enumerate(counts, 1))
    total = sum(counts)
    lines.extend((f"Total RPDO: {total}", f"SYNC Frames: {int(0x80 in stamps)}",
                  f"TX Echo: {'Confirmed' if total == 3 * len(axes) else 'Incomplete'}",
                  f"SYNC Echo: {stamps[0x80]:.6f} (Confirmed)" if 0x80 in stamps else "SYNC Echo: Unconfirmed"))
    if samples is not None:
        lines.extend(f"Node {n}: {state(s.word)} (6041=0x{s.word:04X})" for n, s in samples.items())
    else:
        lines.append("Post-batch TPDO confirmation: unavailable")
    lines.extend(("SocketCAN capture review: manual (candump log)",
                  f"Result: FAILED — {error}" if error else "Result: TRANSMISSION COMPLETED"))
    return "\n".join(lines)
