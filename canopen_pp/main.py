"""Terminal entry point: python -m canopen_pp.main"""
import sys

from . import config
from .communication import Adapter
from .controller import AllNodesOffline, Controller
from .drives import ControlError, make_drives
from .pp_config import CONFIG_PATH, load_targets


def open_controller(axes, settings, report=print):
    drives = make_drives(axes)
    bus = Adapter(drives, settings)
    try:
        return Controller(bus, drives, settings["sync_period_ms"], report)
    except BaseException:
        bus.close()
        raise


def connect(axes, settings, report=print):
    controller = open_controller(axes, settings, report)
    try:
        controller.initialize()
    except BaseException:
        try:
            controller.close()
        except ControlError as exc:
            report(str(exc))
        raise
    return controller


def main():
    controller = None
    result = 0
    print("=" * 40 + "\nEight Axis CANopen PP Controller\n" + "=" * 40)
    mode = "PARAMETER TEST ONLY" if config.PARAMETER_TEST_ONLY else "CiA 402"
    print(f"\nMode: {mode}\nConfiguration: config/pp_targets.json")
    if not config.PARAMETER_TEST_ONLY:
        print("运动异常或退出时会请求整组失能。请确认节点 2 垂直负载的抱闸与独立防坠保护。")
    commands = ["init", "enable", "pp-move", "move-test", "status", "disable", "exit"]
    print("\nCommands:\n  " + "\n  ".join(commands))
    try:
        while True:
            cmd = input("> ").strip().lower()
            try:
                if cmd == "exit":
                    break
                if cmd == "init":
                    if controller is None:
                        controller = open_controller(config.AXIS_CONFIG, config.CAN_CONFIG)
                    controller.initialize()
                    print(f"初始化完成，SYNC {config.CAN_CONFIG['sync_period_ms']} ms")
                    print(controller.status())
                elif cmd in ("enable", "pp-move") and config.PARAMETER_TEST_ONLY:
                    raise ControlError(f"PARAMETER TEST ONLY 模式禁止 {cmd}")
                elif cmd not in commands:
                    if cmd:
                        print("命令: " + "  ".join(commands))
                elif controller is None:
                    print("CAN 尚未初始化，请输入 init")
                elif cmd == "status":
                    print(controller.status())
                elif cmd == "disable":
                    controller.disable()
                elif cmd in ("enable", "pp-move", "move-test"):
                    try:
                        targets = load_targets(CONFIG_PATH, nodes=controller.online_nodes)
                    except (OSError, ValueError, RecursionError) as exc:
                        raise ControlError(f"配置加载失败: {exc}") from exc
                    if cmd == "pp-move":
                        print(f"已读取 {CONFIG_PATH}，使用本次在线节点定位参数")
                        controller.pp_move(targets)
                    elif cmd == "move-test":
                        controller.move_test(targets)
                    else:
                        controller.enable(targets)
                        print(f"使能成功，在线节点 {list(controller.online_nodes)}")
            except ControlError as exc:
                print(f"{cmd} 失败: {exc}", file=sys.stderr)
                if (cmd == "init" and controller is not None and not controller.initialized
                        and not isinstance(exc, AllNodesOffline)):
                    try:
                        controller.close()
                    except ControlError as close_error:
                        print(f"关闭 CAN 失败: {close_error}", file=sys.stderr)
                    controller = None
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        if controller:
            try:
                controller.close()
            except ControlError as exc:
                print(f"退出: {exc}", file=sys.stderr)
                result = 1
    return result


if __name__ == "__main__":
    sys.exit(main())
