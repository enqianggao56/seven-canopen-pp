"""CAN adapter, checked SDO access and the sole SYNC producer."""
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from queue import Empty
from threading import Condition, Event, Thread
from time import monotonic

import canopen

from . import config
from .drives import ControlError

VENDOR = Path(__file__).resolve().parents[1] / "vendor"


class Adapter:
    """Checked SDO in Pre-operational, plus four read-only bench measurements before NMT."""
    def __init__(self, drives, settings, *, listener=None):
        self.network = canopen.Network()
        self.network.NOTIFIER_CYCLE = 0.1
        self.condition = Condition()
        self.pending = None
        self.echo_time = None
        self.probe_request = None
        self.probe_echo_seen = False
        self.external_sync = False
        self.preoperational = set()
        self.operational_started = False
        self.on_sync = None
        self.rpdo_aliases = {base + n for n in drives for base in (0x200, 0x300, 0x400)}
        self.rpdo_conflict = None
        self.network.listeners = [self.receive]
        if listener is not None:
            self.network.listeners.append(listener)
        self.nodes = {}
        for n, drive in drives.items():
            node = self.network.add_node(n, str(VENDOR / drive.eds))
            node.sdo.RESPONSE_TIMEOUT = max(config.INITIAL_SDO_TIMEOUT, config.SDO_TIMEOUT)
            node.sdo.MAX_RETRIES = 1  # one attempt
            self.nodes[n] = node
        try:
            self.network.connect(**{k: v for k, v in settings.items() if k != "sync_period_ms"},
                                 receive_own_messages=True)
        except Exception as exc:
            self.close()
            raise ControlError(f"CAN 打开失败 {settings}: {exc}") from exc

    def receive(self, msg):
        if msg.is_remote_frame or msg.is_error_frame or msg.is_fd:
            return
        with self.condition:
            wire_id = msg.arbitration_id | (0x20000000 if msg.is_extended_id else 0)
            if not msg.is_rx and self.probe_request == (wire_id, msg.data):
                self.probe_echo_seen = True
            if not msg.is_rx and self.pending == (wire_id, msg.data):
                self.echo_time = msg.timestamp
                if wire_id == 0x80 and self.on_sync:
                    self.on_sync(msg.timestamp)
                self.condition.notify_all()
                return
            if wire_id in self.rpdo_aliases:
                self.rpdo_conflict = f"外部 RPDO 0x{wire_id:X} 与主站冲突"
            if msg.is_extended_id:
                return
            if msg.arbitration_id == 0x80:
                self.external_sync = True
        if msg.is_rx:
            self.network.notify(msg.arbitration_id, msg.data, msg.timestamp)

    def require_preop(self, n):
        if self.operational_started or n not in self.preoperational:
            raise ControlError(f"Node {n} SDO 仅允许在初始化 Pre-operational 阶段使用")

    def read(self, n, index, sub=0):
        self.require_preop(n)
        return self._upload(n, index, sub)

    def probe(self, n, index):
        """Addressed, read-only commissioning probe before this master issues any NMT."""
        if self.operational_started or self.preoperational or n not in self.nodes:
            raise ControlError("只读探测只允许在本次会话的 NMT 初始化之前")
        if index not in (0x6041, 0x6061, 0x6063, 0x6064, 0x606C):
            raise ControlError(f"只读探测未允许对象 {index:04X}")
        node = self.nodes[n]
        timeout = max(config.INITIAL_SDO_TIMEOUT, config.SDO_TIMEOUT)
        # A probe can follow an operator pause, even on a previously responsive node.
        node.sdo.RESPONSE_TIMEOUT = timeout
        with self.condition:
            self.probe_request = (node.sdo.rx_cobid, b"\x40" + index.to_bytes(2, "little") + bytes(5))
            self.probe_echo_seen = False
        try:
            return self._upload(n, index, 0)
        finally:
            with self.condition:
                self.probe_request = None

    def _upload(self, n, index, sub):
        node = self.nodes[n]
        try:
            var = node.object_dictionary.get_variable(index, sub)
            # Raw stream preserves the wire width (SdoClient.upload can truncate).
            with node.sdo.open(index, sub, buffering=0) as stream:
                data = stream.read()
            if len(data) * 8 != len(var):
                raise ValueError(f"期望 {len(var)} bit，收到 {len(data)*8} bit")
            return var.decode_raw(data)
        except Exception as exc:
            message = f"Node {n} SDO read {index:04X}:{sub:02X}: {exc}"
            if isinstance(exc, canopen.SdoCommunicationError) and isinstance(exc.__context__, Empty):
                with self.condition:
                    if self.probe_request is not None:
                        echo = "已见" if self.probe_echo_seen else "未见"
                        message += (f"（只读探测等待 {node.sdo.RESPONSE_TIMEOUT:g} s，"
                                    f"{echo} 0x{node.sdo.rx_cobid:03X} 请求的本机发送回显，"
                                    f"未收到 0x{node.sdo.tx_cobid:03X} 应答）")
                try:
                    self.network.check()
                except Exception as rx_error:
                    raise ControlError(f"{message}，CAN 接收线程异常: {rx_error}") from rx_error
            raise ControlError(message) from exc
        finally:
            node.sdo.RESPONSE_TIMEOUT = config.SDO_TIMEOUT

    def write(self, n, index, value, sub=0):
        self.require_preop(n)
        if index == 0x6040:
            raise ControlError("6040 控制字只能通过 RPDO3 发送")
        try:
            node = self.nodes[n]
            var = node.object_dictionary.get_variable(index, sub)
            node.sdo.download(index, sub, var.encode_raw(value))
        except Exception as exc:
            raise ControlError(f"Node {n} SDO write {index:04X}:{sub:02X}={value}: {exc}") from exc

    def expect(self, n, index, value, sub=0):
        actual = self.read(n, index, sub)
        if actual != value:
            raise ControlError(f"Node {n} {index:04X}:{sub:02X} 期望 {value}，读回 {actual}")

    def set(self, n, index, value, sub=0):
        self.write(n, index, value, sub)
        self.expect(n, index, value, sub)

    def nmt(self, n, command):
        if command == 1:
            self.operational_started = True
        try:
            self.transmit(0, bytes((command, n)), timeout=config.INITIAL_SDO_TIMEOUT)
        except ControlError as exc:
            raise ControlError(f"Node {n} NMT {command:#04x}: {exc}") from exc
        if command == 0x80:
            self.preoperational.add(n)

    def transmit(self, cob, data, *, timeout=None):
        with self.condition:
            self.check_producers()
            self.pending = (cob, bytes(data))
            self.echo_time = None
            try:
                self.network.send_message(cob, data)
                if not self.condition.wait_for(lambda: self.echo_time is not None, timeout or config.SDO_TIMEOUT):
                    raise ControlError("未收到适配器 TX echo")
                return self.echo_time
            except Exception as exc:
                raise ControlError(f"CAN TX 0x{cob:03X}: {exc}") from exc
            finally:
                self.pending = None

    def check_producers(self):
        try:
            self.network.check()
        except Exception as exc:
            raise ControlError(f"CAN 接收失败: {exc}") from exc
        if self.external_sync:
            raise ControlError("检测到其他 SYNC 生产者")
        if self.rpdo_conflict:
            raise ControlError(self.rpdo_conflict)

    def close(self):
        try:
            self.network.disconnect()
        except Exception as exc:
            raise ControlError(f"CAN 关闭失败: {exc}") from exc


@dataclass
class Batch:
    frames: tuple
    guard: Callable[[], None] | None = None
    stamp: float = 0.0
    error: Exception | None = None
    finished: bool = False


class SyncClock:
    """The only RPDO and SYNC writer. Each batch ends with the next SYNC."""
    def __init__(self, adapter, period):
        self.adapter = adapter
        self.period = period
        self.cv = Condition()
        self.done = Event()
        self.request = None
        self.error = None
        self.thread = Thread(target=self.run, name="canopen-sync")

    def check(self):
        self.adapter.check_producers()
        if self.done.is_set() or self.error:
            raise ControlError(f"SYNC 已停止: {self.error or 'closed'}")

    def run(self):
        due = monotonic()
        while not self.done.wait(max(0, due - monotonic())):
            with self.cv:
                batch = self.request
                try:
                    if self.error and batch is None:
                        self.cv.wait_for(lambda: self.done.is_set() or self.request is not None)
                        continue
                    if batch and batch.guard:
                        try:
                            batch.guard()
                        except ControlError as exc:
                            batch.error = exc
                    if batch and not batch.error:
                        for cob, data in batch.frames:
                            self.adapter.transmit(cob, data)
                    stamp = self.adapter.transmit(0x080, b"")
                    if batch:
                        batch.stamp = stamp
                    self.error = None
                except Exception as exc:
                    # A partial batch must not be followed by an idle SYNC.
                    self.error = ControlError(f"同步发送失败: {exc}")
                    if batch:
                        batch.error = self.error
                finally:
                    if batch:
                        batch.finished = True
                        self.request = None
                    self.cv.notify_all()
                due += self.period
                if due <= monotonic():
                    due = monotonic() + self.period

    def batch(self, frames, *, guard=None, disable=False):
        with self.cv:
            # Only a complete replacement batch of disable words can resume a
            # clock inhibited by a partial CAN transmission.
            if not disable or self.done.is_set():
                self.check()
            if not self.thread.is_alive() or self.request is not None:
                raise ControlError("SYNC 调度器无法接受批次")
            batch = Batch(tuple(frames), guard)
            self.request = batch
            self.cv.notify_all()
            try:
                self.cv.wait_for(lambda: batch.finished or self.done.is_set())
            except BaseException:
                # If the scheduler has not taken the batch, cancel it before
                # releasing the lock to the caller's disable/exit handler.
                if self.request is batch:
                    self.request = None
                raise
            if batch.error:
                raise batch.error
            if not batch.finished:
                raise ControlError("SYNC 批次已取消")
            return batch.stamp

    def close(self):
        self.done.set()
        with self.cv:
            self.cv.notify_all()
        if self.thread.ident is not None:
            self.thread.join()
