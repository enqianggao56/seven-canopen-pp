"""Plan one symmetric, rest-to-rest PP profile from measured encoder counts."""
from dataclasses import dataclass
from itertools import product
from math import floor, isfinite, sqrt

from . import config
from .drives import ControlError
from .pp import BelowProfileResolution, PPCommand, PPParameters

ENCODED_TIME_TOLERANCE_S = 0.1


@dataclass(frozen=True)
class AxisProfile:
    command: PPCommand
    params: PPParameters
    encoded_duration_s: float


@dataclass(frozen=True)
class Trajectory:
    duration_s: float
    ramp_s: float
    positions: dict[int, int]  # includes axes holding their current setpoint
    moving: dict[int, AxisProfile]

    def check_start(self, encoder, baseline, samples):
        converter = encoder.converter
        for n, sample in samples.items():
            if not standstill(converter, n, sample.vel):
                raise ControlError(f"Node {n} 速度超出静止窗口")
            drift = position_error(converter, n, baseline[n].pos, sample.pos)
            if drift == 0:
                continue
            window = config.PP_POSITION_TOLERANCES[n]
            distance = position_error(converter, n, self.positions[n], sample.pos)
            if drift > window or (distance > window) != (n in self.moving):
                raise ControlError(f"Node {n} 规划起始位置变化，超出位置窗口或改变运动轴选择")
            if n in self.moving:
                p = self.moving[n].params
                elapsed = _encoded_duration(distance, p.profile_velocity, p.profile_acceleration,
                                            encoder.profile_scales(n))
                if abs(elapsed - self.duration_s) > ENCODED_TIME_TOLERANCE_S:
                    raise ControlError(f"Node {n} 规划起始位置变化，编码轮廓时间误差超过 0.1 s")


def position_error(converter, node, target_raw, actual_raw):
    cfg = converter.get_config(node)
    return abs(target_raw - actual_raw) * cfg.travel_per_revolution / cfg.increments_per_revolution


def standstill(converter, node, raw_velocity):
    return abs(converter.raw_to_velocity(node, raw_velocity)) <= config.PP_VELOCITY_TOLERANCES[node]


def _common_ramp(distances, duration):
    # Write u = T - t. The whole group requires u >= p and u*t >= q.
    speed_bounds, acceleration_bounds = {}, {}
    for n, distance in distances.items():
        limit = config.PP_DESIGN_LIMITS[n]
        speed_bounds[n] = distance / limit.velocity
        acceleration_bounds[n] = distance / min(limit.acceleration, limit.deceleration)
    velocity_node = max(speed_bounds, key=speed_bounds.get)
    accel_node = max(acceleration_bounds, key=acceleration_bounds.get)
    p, q = speed_bounds[velocity_node], acceleration_bounds[accel_node]
    u = max(p, sqrt(q))
    minimum = u + q / u
    if duration < minimum:
        raise ValueError(f"公共轮廓不可行，速度限制 Node {velocity_node}，加速度限制 Node {accel_node}，"
                         f"理论最短时间 {minimum:.9g} s")
    half = duration / 2
    # Rationalized smaller quadratic root avoids cancellation for long moves.
    lower = (q / half) / (1 + sqrt(max(0, 1 - (q / half) / half)))
    upper = min(half, duration - p)
    return max(lower, min(duration / 4, upper)), minimum


def _nearby(value, scale, limit):
    ideal = value * float(scale)
    center = floor(ideal)
    largest = min((1 << 32) - 1, floor(limit * scale))
    return range(max(1, center - 1), min(largest, center + 2) + 1)


def _encoded_duration(distance, v_raw, a_raw, scales):
    vs, acs = scales
    v, a = float(v_raw / vs), float(a_raw / acs)
    peak = min(v, sqrt(distance * a))
    return distance / peak + peak / a


def _encode_profile(encoder, cmd, distance, duration, minimum):
    n = cmd.node_id
    target_raw = encoder.target_count(n, cmd.target_position)
    vs, acs = encoder.profile_scales(n)
    limit = config.PP_DESIGN_LIMITS[n]

    # Preserve the encoder's standard downward quantization whenever it fits.
    # Below one count (or at a floating-point limit boundary), check the
    # adjacent representable rates against the same mechanical and U32 limits.
    ramp_limit = min(limit.acceleration, limit.deceleration)
    if 0 < cmd.velocity <= limit.velocity and 0 < cmd.acceleration <= ramp_limit:
        try:
            params = encoder.encode(cmd)
        except BelowProfileResolution:
            pass
        else:
            elapsed = _encoded_duration(distance, params.profile_velocity, params.profile_acceleration, (vs, acs))
            if abs(elapsed - duration) <= ENCODED_TIME_TOLERANCE_S:
                return AxisProfile(cmd, params, elapsed)

    velocities = _nearby(cmd.velocity, vs, limit.velocity)
    ramps = _nearby(cmd.acceleration, acs, ramp_limit)
    best = None
    for v_raw, a_raw in product(velocities, ramps):
        elapsed = _encoded_duration(distance, v_raw, a_raw, (vs, acs))
        error = abs(elapsed - duration)
        if error <= ENCODED_TIME_TOLERANCE_S:
            # Preserve the common shape among candidates that meet the time budget.
            shape = abs(v_raw - cmd.velocity * float(vs)) + abs(a_raw - cmd.acceleration * float(acs))
            rank = (shape, error)
            if best is None or rank < best[0]:
                best = (rank, PPParameters(target_raw, v_raw, a_raw, a_raw), elapsed)
    if best is None:
        raise ValueError(f"Node {n} 轮廓参数分辨率无法满足 {duration:g} s 的时间要求（误差 ≤0.1 s），"
                         f"连续轮廓理论最短时间 {minimum:.9g} s")
    return AxisProfile(cmd, best[1], best[2])


def plan(targets, samples, encoder):
    """Use a caller-verified coherent feedback snapshot. No bus access."""
    positions, distances = {}, {}
    selected = targets.selected(samples)
    for axis in selected:
        n = axis.node_id
        for windows in (config.PP_POSITION_TOLERANCES, config.PP_VELOCITY_TOLERANCES):
            window = windows.get(n)
            if window is None or not isfinite(window) or window < 0:
                raise ValueError(f"Node {n} 的反馈窗口必须为非负有限数")
        if not standstill(encoder.converter, n, samples[n].vel):
            raise ValueError(f"Node {n} 规划起点未静止")
        positions[n] = encoder.converter.target_to_raw(n, axis.position)
        distance = position_error(encoder.converter, n, positions[n], samples[n].pos)
        if distance > config.PP_POSITION_TOLERANCES[n]:
            distances[n] = distance
    if not distances:
        return Trajectory(targets.duration_s, 0, positions, {})
    duration = targets.duration_s
    ramp, minimum = _common_ramp(distances, duration)
    moving = {}
    for axis in selected:
        n = axis.node_id
        if n in distances:
            velocity = distances[n] / (duration - ramp)
            accel = velocity / ramp
            cmd = PPCommand(n, axis.position, velocity, accel, accel)
            moving[n] = _encode_profile(encoder, cmd, distances[n], duration, minimum)
    return Trajectory(duration, ramp, positions, moving)
