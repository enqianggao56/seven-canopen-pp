"""Plan symmetric rest-to-rest PP profiles in the installed drives' rate units."""
from dataclasses import dataclass
from math import ceil, floor, isfinite, sqrt

from . import config
from .drives import ControlError
from .pp import PPCommand, PPParameters


class ReplanRequired(ControlError):
    """A pre-transmission check found a recoverable change of origin."""


class ContinuousInfeasible(ValueError):
    def __init__(self, speed_node, accel_node, minimum, ramp):
        super().__init__(f"连续公共轮廓不可行，速度限制 Node {speed_node}，加速度限制 Node {accel_node}，"
                         f"理论最短时间 {minimum:.9g} s")
        self.minimum = minimum
        self.ramp = ramp


class IntegerInfeasible(ValueError):
    """The bounds or a complete time-window search exclude all legal codes."""


class SearchMiss(ValueError):
    """The bounded search did not prove infeasibility."""


@dataclass(frozen=True)
class AxisProfile:
    command: PPCommand
    params: PPParameters
    encoded_duration_s: float
    encoded_ramp_s: float

    def diagnostic(self, duration):
        c, p = self.command, self.params
        return (f"理论 V/A/B={c.velocity:.8g}/{c.acceleration:.8g}/{c.deceleration:.8g}，"
                f"6081/6083/6084={p.profile_velocity}/{p.profile_acceleration}/{p.profile_deceleration}，"
                f"预计 {self.encoded_duration_s:.8g} s，时间误差 {self.encoded_duration_s-duration:+.8g} s")


@dataclass(frozen=True)
class Trajectory:
    duration_s: float
    ramp_s: float
    positions: dict[int, int]  # requested absolute targets, including skipped axes
    moving: dict[int, AxisProfile]

    def check_start(self, encoder, baseline, samples):
        converter = encoder.converter
        for n, sample in samples.items():
            target = self.positions[n]
            detail = feedback_detail(converter, n, target, sample, config.PP_POSITION_TOLERANCES[n],
                                     config.PP_VELOCITY_TOLERANCES[n])
            speed = abs(converter.raw_to_velocity(n, sample.vel))
            if speed > config.PP_VELOCITY_TOLERANCES[n]:
                error = ControlError if speed > config.PP_HOLD_HARD_VELOCITY[n] else ReplanRequired
                raise error(f"Node {n} 速度超出静止窗口，{detail}")
            drift = position_error(converter, n, baseline[n].pos, sample.pos)
            # For an untriggered axis the hold monitor has its own reference.
            # Two samples at opposite sides of that window can be farther
            # apart than the planning window while both are valid hold samples.
            drift_limit = config.PP_STABLE_POSITION[n] if n in self.moving else config.PP_HOLD_HARD_POSITION[n]
            if drift > drift_limit:
                raise ControlError(f"Node {n} 规划起始位置变化 {drift:g} 超出 {drift_limit:g}，{detail}")
            distance = position_error(converter, n, target, sample.pos)
            if (distance > config.PP_POSITION_TOLERANCES[n]) != (n in self.moving):
                raise ReplanRequired(f"Node {n} 规划起始位置变化，运动轴选择变化，{detail}")
            if drift and n in self.moving:
                axis = self.moving[n]
                p = axis.params
                vs, acs = encoder.profile_scales(n)
                elapsed, _ = _timing(distance, float(p.profile_velocity/vs), float(p.profile_acceleration/acs))
                if abs(elapsed - self.duration_s) > config.PP_ENCODED_TIME_TOLERANCE_S:
                    raise ReplanRequired(f"Node {n} 规划起始位置变化，编码轮廓时间误差 "
                                         f"{elapsed-self.duration_s:+.8g} s，新预计 {elapsed:.8g} s，"
                                         f"{axis.diagnostic(self.duration_s)}，{detail}")


def position_error(converter, node, target_raw, actual_raw):
    cfg = converter.get_config(node)
    return abs(target_raw - actual_raw) * cfg.travel_per_revolution / cfg.increments_per_revolution


def feedback_detail(converter, n, target, sample, pos_window, vel_window):
    unit = converter.get_config(n).unit
    actual = converter.raw_to_actual(n, sample.pos)
    goal = converter.raw_to_actual(n, target)
    velocity = converter.raw_to_velocity(n, sample.vel)
    return (f"实际 {actual:.8g} {unit} ({sample.pos} inc)，目标 {goal:.8g} {unit} ({target} inc)，"
            f"误差 {actual-goal:+.8g} {unit}，位置阈值 {pos_window:g} {unit}，"
            f"606C={sample.vel}，机械速度 {velocity:.8g} {unit}/s，阈值 {vel_window:g} {unit}/s")


def standstill(converter, node, raw_velocity):
    return abs(converter.raw_to_velocity(node, raw_velocity)) <= config.PP_VELOCITY_TOLERANCES[node]


def _common_ramp(distances, duration):
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
        raise ContinuousInfeasible(velocity_node, accel_node, minimum, q/u)
    half = duration / 2
    lower = (q / half) / (1 + sqrt(max(0, 1 - (q / half) / half)))
    return lower, min(half, duration - p)


def _timing(distance, speed, accel):
    peak = min(speed, sqrt(distance * accel))
    ramp = peak / accel
    return distance / peak + ramp, ramp


def _adjacent(value, maximum):
    lo = floor(max(1, min(value, maximum)))
    return {lo, min(maximum, lo + 1)}


def _accel_for_time(d, v, time):
    coast = d / v
    if time <= coast:
        return float("inf")
    if time >= 2 * coast:
        return (4 * d / time) / time  # triangular, speed cap never reached
    return v / (time - coast)


def _speed_for_time(d, a, time):
    half = time / 2
    if half <= sqrt(d/a):
        return sqrt(d*a)
    return (d/half) / (1 + sqrt(max(0, 1 - (d/half)/a/half)))


def _kinco_speed_codes(distance, times, cmd, vs, acs, vmax, amax):
    # Speeds below D/(T+tol) are too slow even at infinite acceleration.
    # A triangular profile needs at most sqrt(D*Amax) peak speed.
    lo = max(1, floor(distance/times[-1]*vs))
    hi = min(vmax, ceil(sqrt(distance*amax/acs)*vs))
    if hi - lo <= 128:
        return range(lo, hi + 1), True

    # Invert T for V. Probe the acceleration limits, desired common shape,
    # triangular boundary and a fixed-size geometric grid over the legal codes.
    speeds = {lo, hi} | _adjacent(cmd.velocity*vs, vmax)
    accels = _adjacent(cmd.acceleration*acs, amax) | {1, amax}
    for t in times:
        if t > 0:
            accels.update(_adjacent((4*distance/t)/t*acs, amax))
    accels.update(round(amax ** (i/32)) for i in range(33))
    for code in accels:
        for t in times:
            speeds.update(_adjacent(_speed_for_time(distance, code/acs, t)*vs, vmax))
    return speeds, False


def _code_limits(encoder, n):
    vs, acs = encoder.profile_scales(n)
    limits = config.PP_DESIGN_LIMITS[n]
    vmax = min(0xffffffff, floor(limits.velocity * vs))
    amax = min(0xffffffff, floor(min(limits.acceleration, limits.deceleration) * acs))
    return float(vs), float(acs), vmax, amax


def _encode_profile(encoder, axis, distance, duration, ramp):
    n = axis.node_id
    velocity = distance / (duration - ramp)
    accel = velocity / ramp
    cmd = PPCommand(n, axis.position, velocity, accel, accel)
    target = encoder.target_count(n, axis.position)
    vs, acs, vmax, amax = _code_limits(encoder, n)
    tol = config.PP_ENCODED_TIME_TOLERANCE_S
    if min(vmax, amax) < 1:
        raise IntegerInfeasible(f"Node {n} 整数轮廓不可行，设计上限低于一个编码单位，理论 V/A/B="
                                f"{velocity:g}/{accel:g}/{accel:g}")
    best = None
    best_rank = None

    def consider(v, a):
        nonlocal best, best_rank
        elapsed, actual_ramp = _timing(distance, v/vs, a/acs)
        error = abs(elapsed-duration)
        shape = abs(actual_ramp-ramp)
        rank = (error > tol, shape if error <= tol else error, error, abs(v-velocity*vs), a)
        if best_rank is None or rank < best_rank:
            best_rank = rank
            best = AxisProfile(cmd, PPParameters(target, v, a, a), elapsed, actual_ramp)

    # Keep the standard downward encoding when it realizes the requested shape
    # to within one code and already meets the time budget.
    v, a = floor(velocity*vs), floor(accel*acs)
    if 1 <= v <= vmax and 1 <= a <= amax:
        consider(v, a)
        if not best_rank[0]:
            return best

    consider(1, 1)
    consider(vmax, amax)
    fastest, _ = _timing(distance, vmax/vs, amax/acs)
    slowest, _ = _timing(distance, 1/vs, 1/acs)
    exhaustive = True
    if fastest <= duration + tol and slowest >= duration - tol:
        times = (max(0., duration-tol), duration, duration+tol)
        # For fixed velocity the duration decreases monotonically with A.
        # These inverse images bracket the time window and the desired ramp.
        speeds = range(1, vmax + 1)
        if encoder.converter.get_config(n).device_type == "Kinco":
            speeds, exhaustive = _kinco_speed_codes(distance, times, cmd, vs, acs, vmax, amax)
        for code in speeds:
            v = code / vs
            desired = min(v/ramp, (distance/ramp)/ramp)
            candidates = _adjacent(desired*acs, amax)
            for t in times:
                candidates.update(_adjacent(_accel_for_time(distance, v, t)*acs, amax))
            for a in candidates:
                consider(code, a)
    if best_rank[0]:
        reason = "整数轮廓不可行（编码分辨率）" if exhaustive else "有界搜索未找到解"
        failure = IntegerInfeasible if exhaustive else SearchMiss
        raise failure(f"Node {n} {reason}，T={duration:g} s，容差 {tol:g} s，"
                      f"整数时间范围 [{fastest:.8g}, {slowest:.8g}] s，最佳 {best.diagnostic(duration)}")
    return best


def plan(targets, samples, encoder):
    """Use a caller-verified coherent feedback snapshot. No bus access."""
    if not isfinite(config.PP_ENCODED_TIME_TOLERANCE_S) or config.PP_ENCODED_TIME_TOLERANCE_S < 0:
        raise ValueError("编码时间容差必须为非负有限数")
    positions, distances = {}, {}
    converter = encoder.converter
    selected = targets.selected(samples)
    for axis in selected:
        n = axis.node_id
        for windows in (config.PP_POSITION_TOLERANCES, config.PP_VELOCITY_TOLERANCES):
            if n not in windows or not isfinite(windows[n]) or windows[n] < 0:
                raise ValueError(f"Node {n} 的反馈窗口必须为非负有限数")
        positions[n] = converter.target_to_raw(n, axis.position)
        if not standstill(converter, n, samples[n].vel):
            raise ValueError(f"Node {n} 规划起点未静止，" + feedback_detail(
                converter, n, positions[n], samples[n], config.PP_STABLE_POSITION[n],
                config.PP_VELOCITY_TOLERANCES[n]))
        distance = position_error(converter, n, positions[n], samples[n].pos)
        if distance > config.PP_POSITION_TOLERANCES[n]:
            distances[n] = distance
    duration = targets.duration_s
    if not distances:
        return Trajectory(duration, 0, positions, {})

    def encode_group(ramp):
        return {axis.node_id: _encode_profile(encoder, axis, distances[axis.node_id], duration, ramp)
                for axis in selected if axis.node_id in distances}

    try:
        lower, upper = _common_ramp(distances, duration)
        ramp = max(lower, min(duration/4, upper))
        # A Kinco search miss is not a proof. Try the ends and midpoint of
        # the continuous common-ramp interval before rejecting the request.
        for candidate in dict.fromkeys((ramp, upper, lower, (lower + upper)/2)):
            try:
                moving = encode_group(candidate)
            except SearchMiss as exc:
                miss = exc
                continue
            ramp = candidate
            break
        else:
            raise miss
        # If quantization forces a different ramp on one drive, also fit the
        # whole group to that physical ramp within the continuous limits.
        outlier = max(moving.values(), key=lambda p: abs(p.encoded_ramp_s-ramp))
        alternate = max(lower, min(outlier.encoded_ramp_s, upper))
        if len(moving) > 1 and abs(alternate-ramp) > config.PP_ENCODED_TIME_TOLERANCE_S:
            def spread(group):
                ramps = [p.encoded_ramp_s for p in group.values()]
                return max(ramps) - min(ramps)
            try:
                revised = encode_group(alternate)
            except SearchMiss:
                revised = None
            if revised and spread(revised) < spread(moving):
                ramp, moving = alternate, revised
    except (IntegerInfeasible, SearchMiss, ContinuousInfeasible) as exc:
        details = [f"Node {n} " + feedback_detail(converter, n, positions[n], samples[n],
                   config.PP_POSITION_TOLERANCES[n], config.PP_VELOCITY_TOLERANCES[n]) for n in distances]
        if isinstance(exc, ContinuousInfeasible):
            for n, distance in distances.items():
                vs, acs, vmax, amax = _code_limits(encoder, n)
                v = distance / (exc.minimum - exc.ramp)
                a = v / exc.ramp
                detail = f"Node {n} 最短公共轮廓理论 V/A/B={v:.8g}/{a:.8g}/{a:.8g}"
                if min(vmax, amax) > 0:
                    elapsed, _ = _timing(distance, vmax/vs, amax/acs)
                    detail += (f"，单轴最快 6081/6083/6084={vmax}/{amax}/{amax}，预计 {elapsed:.8g} s，"
                               f"时间误差 {elapsed-duration:+.8g} s")
                details.append(detail)
        exc.args = (f"{exc}\n" + "\n".join(details),)
        raise
    return Trajectory(duration, ramp, positions, moving)
