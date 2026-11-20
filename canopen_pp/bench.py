"""Operator-guided, single-axis SocketCAN PP commissioning. Run with python -m canopen_pp.bench."""
import argparse
from contextlib import ExitStack
from dataclasses import replace
from datetime import datetime, timezone
import json
from math import isfinite
from pathlib import Path
import select
import sys
from tempfile import mkdtemp
from threading import Lock
from time import monotonic, time

import can

from . import config
from .communication import Adapter
from .controller import Controller, DISABLED
from .drives import ControlError, make_drives, state
from .motion import AxisMotion, MotionState
from .pp import PPMotionCommand


class BenchLog:
    def __init__(self, stream, node):
        self.stream, self.node = stream, node
        self.step = 1
        self.lock = Lock()
        self.io_error = None

    def event(self, event, **values):
        row = dict(time=time(), node=self.node, step=self.step, event=event, **values)
        with self.lock:
            if self.io_error is None:
                try:
                    self.stream.write(json.dumps(row, ensure_ascii=False) + "\n")
                    self.stream.flush()
                except OSError as exc:
                    # Keep the stop/cleanup path usable if recording fails mid-motion.
                    self.io_error = exc
                    print(f"步骤日志写入失败，{exc}", file=sys.stderr)

    def check(self):
        if self.io_error is not None:
            raise ControlError(f"步骤日志不可用，{self.io_error}")

    def report(self, message):
        self.event("message", message=message)
        try:
            print(message, flush=True)
        except BrokenPipeError as exc:
            self.io_error = exc

    def confirm(self, token, prompt):
        self.check()
        reply = input(f"{prompt}\n请原样输入下一行的确认词并回车，直接回车取消。\n{token}\n> ").strip()
        if not reply:
            raise ControlError("操作人员取消")
        if reply != token:
            raise ControlError(f"确认词不匹配，已中止。应输入「{token}」")
        self.event("operator_confirmation", token=token)
        self.check()


def number(prompt):
    try:
        value = float(input(f"{prompt} > "))
    except ValueError as exc:
        raise ControlError("请输入有限数值") from exc
    if not isfinite(value):
        raise ControlError("请输入有限数值")
    return value


def probe(bus, drive, mechanical, log):
    n = drive.id
    indexes = (0x6041, 0x6061, drive.position_index, 0x606C)
    word, mode, pos, vel = (bus.probe(n, index) for index in indexes)
    unit = mechanical.get_config(n).unit
    actual = mechanical.raw_to_actual(n, pos)
    speed = mechanical.raw_to_velocity(n, vel)
    log.event("read_only_sdo", statusword=word, mode=mode, position_raw=pos,
              velocity_raw=vel, actual=actual, speed=speed, unit=unit)
    log.report(f"Node {n} {state(word)} 6041=0x{word:04X}, 6061={mode}, "
               f"位置 {actual:.4f} {unit} ({pos}), 速度 {speed:.4f} {unit}/s ({vel})")
    return word, pos


def show_plan(motion, command, raw, log):
    encoded = motion.preview(command, raw)
    origin = motion.position(raw)
    log.report(f"路径 {origin:.5f} → {command.target_position:.5f} {motion.axis.unit}，"
               f"量化目标 {motion.position(encoded.target_position):.5f}。")
    log.event("preview", origin_raw=raw, command=vars(command),
              frames=[(cob, data.hex(" ")) for cob, data in encoded.frames()])
    for cob, data in encoded.frames():
        print(f"0x{cob:03X} [{len(data)}] {data.hex(' ').upper()}")
    return encoded


def monitor(motion, log):
    """Keep polling deadlines and the handshake while accepting stop + Enter."""
    last = printed = 0.
    previous = None
    print("监控中。输入 stop 并回车可请求受控停止，Ctrl-C 中止。", flush=True)
    while motion.phase in {MotionState.TRIGGERING, MotionState.MOVING}:
        log.check()
        motion.poll_motion()
        s = motion.c.feedback.snapshot()[motion.node]
        if (s.fresh(config.FEEDBACK_TIMEOUT) and s.cycle1 == s.cycle2
                and s.cycle1 >= motion.trigger_sync and s.cycle1 > last):
            last = s.cycle1
            pos, vel = motion.position(s.pos), motion.velocity(s.vel)
            slope = None
            if previous and s.tp2 > previous[0]:
                slope = (abs(vel) - abs(previous[1])) / (s.tp2 - previous[0])
            previous = s.tp2, vel
            log.event("feedback", cycle=last, tpdo1=s.tp1, tpdo2=s.tp2, phase=motion.phase.value,
                      statusword=s.word, mode=s.mode, position_raw=s.pos, velocity_raw=s.vel,
                      position=pos, velocity=vel, speed_slope=slope)
            if monotonic() - printed >= .25:
                print(f"{motion.phase.value} 6041=0x{s.word:04X} 6061={s.mode} "
                      f"位置={pos:.4f} 速度={vel:.4f}", flush=True)
                printed = monotonic()
        if motion.phase not in {MotionState.TRIGGERING, MotionState.MOVING}:
            break
        if select.select([sys.stdin], [], [], .01)[0]:
            line = sys.stdin.readline()
            if not line:
                raise EOFError
            if line.strip().lower() == "stop":
                s = motion.c.feedback.snapshot()[motion.node]
                moving = (motion.phase == MotionState.MOVING and s.fresh(config.FEEDBACK_TIMEOUT) and s.cycle1 == s.cycle2
                          and s.cycle1 >= motion.trigger_sync
                          and abs(motion.velocity(s.vel)) > motion.limits.stopped_velocity)
                log.event("operator_stop", velocity_raw=s.vel, position_raw=s.pos, moving=moving)
                motion.request_stop()
                return moving
    if motion.phase in {MotionState.FAILED, MotionState.TIMEOUT}:
        raise ControlError(motion.detail)
    return False


def run_steps(motion, log, read_only):
    c, n = motion.c, motion.node
    word, anchor = probe(c.bus, motion.drive, c.mechanical, log)
    log.event("passed")
    log.check()
    if read_only:
        return
    log.step = 2
    if state(word) != DISABLED:
        raise ControlError("只读检查完成，写入要求已处于 Switch On Disabled")
    unit = motion.axis.unit
    lim = motion.limits
    delta = number(f"第一次相对本次实测起点的小位移 ({unit}，绝对值 ≤ {lim.travel})")
    command = PPMotionCommand(n, motion.position(anchor) + delta,
                              number(f"调试速度 ({unit}/s，上限 {lim.velocity})"),
                              number(f"调试加速度 ({unit}/s²，上限 {lim.acceleration})"),
                              number(f"调试减速度 ({unit}/s²，上限 {lim.deceleration})"))
    show_plan(motion, command, anchor, log)
    log.confirm(f"STAGE {n}", "核对机械零点、方向、当前实测位置及整段行程。"
                "确认隔离单轴、无其他主站，允许初始化映射并在失能状态写入当前位置及轮廓参数。")
    log.event("pre_init_status_request", index="6041:00")
    word = c.bus.probe(n, 0x6041)
    log.event("pre_init_status", statusword=word)
    if state(word) != DISABLED:
        raise ControlError(f"人工确认后 6041=0x{word:04X} ({state(word)})，已中止初始化")
    c.initialize(preop_setup=lambda: motion.stage_disabled(command, anchor))
    log.report(c.status())
    log.event("passed", hold_raw=motion.hold_raw, rates=motion.verified_rates)
    log.step = 3
    record = motion.check_stop_record()
    torque = motion.current_limit if motion.kinco else motion.drive.axis.pp_torque_per_mille
    log.report(f"停止凭据 {record.evidence}，控制字 0x{record.word:04X}，"
               f"预期 {record.final_state}，{'6073h raw' if motion.kinco else '6071h ‰'}={torque}")
    log.confirm(f"ENABLE {n}", "确认电机可靠固定、负载受到支撑、物理急停可用，"
                "停止验证工况与本次一致，当前位置保持目标安全，允许单轴使能。")
    motion.enable()
    log.report(c.status())
    log.event("passed")
    for run in (1, 2):
        log.step = 3 + run
        s = c.wait_feedback([n], description="定位前反馈", both=True)[n]
        if run == 2:
            delta = number(f"第二次独立定位，从当前实际位置 {motion.position(s.pos):.5f} {unit} 起的位移")
            command = replace(command, target_position=motion.position(s.pos) + delta)
        show_plan(motion, command, s.pos, log)
        motion.prepare_motion(command, s.pos)
        log.confirm(f"MOVE {run} NODE {n}", "核对本次起点、绝对目标和编码，确认只执行这一次定位。")
        motion.execute_motion()
        moving_at_stop = monitor(motion, log)
        if run == 1 and motion.phase != MotionState.COMPLETED:
            raise ControlError("第一次定位未正常完成，第二次定位已阻止")
        if run == 2 and not (motion.stop_confirmed and moving_at_stop):
            raise ControlError("第二次定位结束，未取得运动中请求停止并确认静止的记录")
        log.event("passed", result=motion.phase.value, trigger_sync=motion.trigger_sync)


def main(argv=None):
    parser = argparse.ArgumentParser(description="单轴 PP 分步台架验证")
    parser.add_argument("--node", required=True, type=int, choices=tuple(config.AXIS_CONFIG))
    parser.add_argument("--channel", default=config.CAN_CONFIG["channel"])
    parser.add_argument("--read-only", action="store_true", help="仅执行第一步 SDO 上传")
    parser.add_argument("--log-dir", type=Path, default=Path("bench-logs"))
    parser.add_argument("--torque-per-mille", type=int, help="RobStride 调试力矩上限，单位 ‰")
    args = parser.parse_args(argv)
    n = args.node
    axis = config.AXIS_CONFIG[n]
    if not args.read_only and not sys.stdin.isatty():
        parser.error("主动测试需要本机交互终端逐步确认")
    if args.torque_per_mille is not None and (
            axis.pp_torque_per_mille is None or not 1 <= args.torque_per_mille <=
            min(config.DEBUG_MAX_TORQUE_PER_MILLE, axis.pp_torque_per_mille)):
        parser.error("调试力矩参数仅适用于 RobStride，须在该轴调试上限内")
    if axis.pp_torque_per_mille is not None:
        if not args.read_only and args.torque_per_mille is None:
            parser.error("RobStride 主动测试须显式指定 --torque-per-mille")
        # The read-only drive descriptor never transmits a controlword.
        axis = replace(axis, pp_torque_per_mille=args.torque_per_mille or 1)
    drives = make_drives({n: axis})
    args.log_dir.mkdir(parents=True, exist_ok=True)
    prefix = datetime.now(timezone.utc).strftime(f"node{n}-%Y%m%dT%H%M%SZ-")
    directory = Path(mkdtemp(prefix=prefix, dir=args.log_dir))
    print(f"记录目录 {directory}", flush=True)
    result = 0
    with ExitStack() as stack:
        stream = stack.enter_context((directory / "steps.jsonl").open("w", encoding="utf-8"))
        log = BenchLog(stream, n)
        log.event("session", channel=args.channel, read_only=args.read_only, debug_limits=vars(config.DEBUG_LIMITS[n]))
        writer = can.ASCWriter(directory / "bus.asc")
        stack.callback(writer.stop)
        motion = None
        try:
            bus = Adapter(drives, dict(config.CAN_CONFIG, channel=args.channel), listener=writer)
            def disconnect():
                nonlocal result
                try:
                    bus.close()
                except ControlError as exc:
                    log.report(f"CAN 关闭异常，{exc}")
                    result = 1
            stack.callback(disconnect)
            c = Controller(bus, drives, config.CAN_CONFIG["sync_period_ms"], log.report)
            stack.callback(c.clock.close)
            def emcy(cob, data, stamp):
                if len(data) == 8:
                    log.report(f"EMCY 0x{cob:03X} t={stamp:.6f} code=0x{int.from_bytes(data[:2], 'little'):04X} "
                               f"register=0x{data[2]:02X} data={data.hex(' ')}")
            bus.network.subscribe(0x080+n, emcy)
            motion = AxisMotion(c, n, log.report)
            run_steps(motion, log, args.read_only)
        except (ControlError, ValueError, OSError, EOFError, KeyboardInterrupt) as exc:
            log.event("failed", reason=str(exc) or type(exc).__name__)
            log.report(f"步骤 {log.step} 中止，{exc or type(exc).__name__}")
            result = 1
        finally:
            if motion and motion.enabled_here:
                try:
                    if not motion.stop_confirmed and motion.phase not in {
                            MotionState.FAILED, MotionState.TIMEOUT, MotionState.STOPPING}:
                        motion.request_stop()
                    motion.disable_after_stop()
                    log.event("disable_confirmed")
                except (ControlError, KeyboardInterrupt) as exc:
                    log.report(f"退出时停止或失能未确认，请检查外部安全回路。{exc}")
                    result = 1
            elif motion and motion.c.needs_disable:
                log.report("使能阶段回退未确认，请检查外部安全回路")
                result = 1
    return result


if __name__ == "__main__":
    sys.exit(main())
