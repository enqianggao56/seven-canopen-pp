"""Script TPDO samples at individual SYNC boundaries; no CAN interfaces or real motors."""
import struct
from dataclasses import replace
from types import SimpleNamespace

import canopen
import pytest

from canopen_pp import config, motion as module
from canopen_pp import controller as controller_module, pdo
from canopen_pp.controller import Controller, DISABLED, ENABLED
from canopen_pp.drives import ControlError, make_drives
from canopen_pp.motion import AxisMotion, MotionState, SETPOINT_ACK, TARGET_REACHED
from canopen_pp.pdo import Sample
from canopen_pp.pp import PPMotionCommand


class BenchSamples:
    def __init__(self, n, monkeypatch):
        self.network = canopen.Network()
        self.n = n
        self.at = 10.
        self.thread = SimpleNamespace(ident=1)
        self.frames = []
        self.words = []
        self.raw = 65536 if n == 1 else 0
        self.objects = {0x6041: 0x40, 0x6063: self.raw, 0x6064: self.raw, 0x606C: 0,
                        0x605D: 1, 0x6073: 25}
        axis = config.AXIS_CONFIG[n]
        if n == 4:
            axis = replace(axis, pp_torque_per_mille=25)
        c = Controller(self, make_drives({n: axis}), 100, lambda text: None)
        self.c = c
        c.clock = self
        c.initialized = True
        self.s = Sample(word=0x40, pos=self.raw)
        self.tick()
        self.motion = AxisMotion(c, n, lambda text: None)
        self.command = PPMotionCommand(n, 12.5 if n == 1 else -2, .6, 1.2, 1.2)
        self.stop_word = 0x12F if n == 1 else 0x000B
        self.stop_state = ENABLED if n == 1 else "Quick Stop Active"
        self.coasting = False
        self.guard_race = None
        monkeypatch.setattr(config, "DEBUG_VERIFIED_STOPS", {
            n: config.BenchStop(self.stop_word, self.stop_state, "synthetic bench record", 25)})
        for mod in (module, pdo, controller_module):
            monkeypatch.setattr(mod, "monotonic", lambda: self.at)
        monkeypatch.setattr(module, "sleep", lambda _: self.tick())
        monkeypatch.setattr(config, "DEBUG_STOP_TIMEOUT", .6)

    def require_preop(self, n):
        assert n == self.n

    def read(self, n, index):
        assert n == self.n
        return self.objects[index]

    def set(self, n, index, value):
        assert n == self.n
        self.objects[index] = value

    def expect(self, n, index, value):
        assert self.read(n, index) == value

    def tick(self, **values):
        self.at += .1
        self.s = replace(self.s, cycle1=self.at, cycle2=self.at, tp1=self.at, tp2=self.at,
                         received1=self.at, received2=self.at, **values)
        self.c.feedback.sync(self.at)
        self.c.feedback.samples[self.n] = self.s

    def check(self):
        pass

    def batch(self, frames, *, guard=None, disable=False):
        if self.guard_race:
            self.guard_race()
            self.guard_race = None
        if guard:
            guard()
        values = {}
        for cob, data in frames:
            self.frames.append((cob, data))
            assert cob in (0x200+self.n, 0x300+self.n, 0x400+self.n)
            if cob == 0x400+self.n:
                word = int.from_bytes(data[:2], "little")
                self.words.append(word)
                if word in (6, 7, 15):
                    values.update(word={6: 0x21, 7: 0x23, 15: 0x27}[word], mode=1)
                if word == self.c.drives[self.n].disable_word:
                    values.update(word=0x40)
                if word == self.stop_word:
                    values.update(word=(0x27 | TARGET_REACHED) if self.n == 1 else 0x07, vel=self.s.vel if self.coasting else 0)
        self.tick(**values)
        return self.at

    def ready(self):
        self.motion.stage_disabled(self.command, self.raw)
        assert self.objects[0x607A] == self.raw
        self.motion.enable()
        self.motion.prepare_motion(self.command, self.raw)
        assert self.motion.phase == MotionState.READY

    def finish(self):
        word = (0x27 | TARGET_REACHED) if self.n == 1 else 0x27
        for _ in range(3):
            self.tick(pos=self.motion.encoded.target_position, vel=0, word=word)
            self.motion.poll_motion()
        assert self.motion.phase == MotionState.COMPLETED


def test_kinco_edge_ack_clear_position_and_second_command(monkeypatch):
    b = BenchSamples(1, monkeypatch)
    b.ready()
    m = b.motion
    for target in (12.5, 12):
        if target == 12:
            m.prepare_motion(replace(b.command, target_position=target), b.s.pos)
        m.execute_motion()
        assert b.words[-1] == 0x3F and (0x3F ^ 0x2F) == 1 << 4
        with pytest.raises(ControlError, match="READY"):
            m.execute_motion()
        b.tick(word=0x27 | TARGET_REACHED)
        assert m.poll_motion().state == MotionState.TRIGGERING
        b.tick(word=0x27 | SETPOINT_ACK)
        assert m.poll_motion().state == MotionState.TRIGGERING and b.words[-1] == 0x2F
        b.tick(word=0x27)
        assert m.poll_motion().state == MotionState.MOVING
        b.finish()
    assert b.words.count(0x3F) == 2


def test_rs_target_only_after_execute_and_no_bit12_required(monkeypatch):
    b = BenchSamples(4, monkeypatch)
    b.ready()
    m = b.motion
    assert b.words == [6, 7, 15] and b.objects[0x6071] == 25
    assert all(cob == 0x404 and struct.unpack("<Hbh", data)[2] == 25 for cob, data in b.frames)
    m.execute_motion()
    assert b.frames[-1] == m.encoded.frames()[0]
    b.tick(word=0x27 | TARGET_REACHED)
    assert m.poll_motion().state == MotionState.TRIGGERING
    b.tick(pos=-20, vel=-3, word=0x27)
    assert m.poll_motion().state == MotionState.MOVING
    for _ in range(4):
        m.poll_motion()   # one sample cannot confirm a stable position
    b.finish()
    assert b.words == [6, 7, 15]
    m.prepare_motion(replace(b.command, target_position=0), b.s.pos)
    m.execute_motion()
    m.request_stop()
    assert m.phase == MotionState.STOPPED and b.words[-1] == 0xB and m.stop_confirmed
    assert len([cob for cob, _ in b.frames if cob == 0x204]) == 2


@pytest.mark.parametrize("failure", ["ack-high", "ack-low", "deadline", "loss", "drive-fault", "path", "mode"])
def test_motion_failures_latch_and_do_not_retrigger(monkeypatch, failure):
    b = BenchSamples(1, monkeypatch)
    b.ready()
    m = b.motion
    m.execute_motion()
    if failure == "ack-low":
        b.tick(word=0x27 | SETPOINT_ACK)
        m.poll_motion()
    if failure in ("ack-high", "ack-low", "deadline"):
        b.at += config.DEBUG_MOTION_TIMEOUT + .1 if failure == "deadline" else config.TRANSITION_TIMEOUT + .1
        b.tick()
    elif failure == "loss":
        b.at += config.FEEDBACK_TIMEOUT + .1
    elif failure == "drive-fault":
        b.tick(word=8)
    elif failure == "path":
        b.tick(pos=-1)
    else:
        b.tick(mode=0)
    result = m.poll_motion()
    assert result.state == (MotionState.TIMEOUT if failure in ("ack-high", "ack-low", "deadline")
                            else MotionState.FAILED)
    if failure in ("loss", "drive-fault", "mode"):
        assert not result.stop_confirmed and "停止未确认" in result.detail
    else:
        assert result.stop_confirmed
    with pytest.raises(ControlError):
        m.execute_motion()
    with pytest.raises(ControlError):
        m.prepare_motion(b.command, b.raw)
    assert b.words.count(0x3F) == 1


def test_missing_stop_and_wrong_current_block_enable(monkeypatch):
    b = BenchSamples(1, monkeypatch)
    m = b.motion
    m.stage_disabled(b.command, b.raw)
    record = config.DEBUG_VERIFIED_STOPS.pop(1)
    with pytest.raises(ControlError, match="停止记录"):
        m.enable()
    config.DEBUG_VERIFIED_STOPS[1] = replace(record, effort_limit=24)
    with pytest.raises(ControlError, match="6073"):
        m.enable()
    config.DEBUG_VERIFIED_STOPS[1] = record
    m.halt_option = 2
    with pytest.raises(ControlError, match="605D"):
        m.enable()
    assert not b.frames


def test_stop_state_without_deceleration_is_not_stopped(monkeypatch):
    b = BenchSamples(4, monkeypatch)
    b.ready()
    b.motion.execute_motion()
    b.coasting = True
    b.tick(pos=-15, vel=-10)
    b.motion.poll_motion()
    with pytest.raises(module.MotionTimeout):
        b.motion.request_stop()
    assert b.motion.phase == MotionState.TIMEOUT and not b.motion.stop_confirmed


@pytest.mark.parametrize("node", [0, 1, 5, True])
def test_node_isolation(monkeypatch, node):
    b = BenchSamples(4, monkeypatch)
    with pytest.raises(ControlError):
        AxisMotion(b.c, node)
    with pytest.raises(ControlError):
        b.motion.preview(replace(b.command, node_id=node), b.raw)
    b.c.drives = make_drives(config.AXIS_CONFIG)
    with pytest.raises(ControlError):
        AxisMotion(b.c, 4)
    assert not b.frames


def test_limits_origin_and_quantized_endpoints(monkeypatch):
    b = BenchSamples(4, monkeypatch)
    for fields in (dict(profile_velocity=3.01), dict(profile_acceleration=6.01),
                   dict(profile_deceleration=6.01), dict(profile_velocity=float("nan")),
                   dict(target_position=-3.001), dict(target_position=-.05)):
        with pytest.raises(ControlError):
            b.motion.preview(replace(b.command, **fields), b.raw)
    with pytest.raises(ControlError, match="软限位"):
        b.motion.preview(replace(b.command, target_position=89), 4097)
    b.motion.stage_disabled(b.command, b.raw)
    b.tick(pos=400)
    with pytest.raises(ControlError, match="起点"):
        b.motion.enable()
    assert not b.frames


def test_dispatch_rechecks_drift_before_rs_target(monkeypatch):
    b = BenchSamples(4, monkeypatch)
    b.ready()
    b.guard_race = lambda: b.tick(pos=300)
    with pytest.raises(ControlError, match="起点"):
        b.motion.execute_motion()
    assert all(cob != 0x204 for cob, _ in b.frames)


@pytest.mark.parametrize("cancel", ["enable", "first", "second", "none"])
def test_operator_tokens_gate_each_step(monkeypatch, cancel):
    import io
    import json

    from canopen_pp import bench

    b = BenchSamples(4, monkeypatch)
    log_stream = io.StringIO()
    log = bench.BenchLog(log_stream, 4)
    monkeypatch.setattr(b, "probe", lambda n, i: 0 if i == 0x6061 else b.read(n, i), raising=False)
    monkeypatch.setattr(b.c, "initialize", lambda *, preop_setup: preop_setup())
    runs = []

    def monitor(m, log):
        runs.append(m.trigger_sync)
        b.tick(pos=-20 if len(runs) == 1 else b.s.pos+20, vel=-2 if len(runs) == 1 else 2)
        m.poll_motion()
        if len(runs) == 1:
            b.finish()
            return False
        m.request_stop()
        return True

    monkeypatch.setattr(bench, "monitor", monitor)
    answers = iter(["-2", "1.2", "1.2", "1.2", "STAGE 4",
                    "" if cancel == "enable" else "ENABLE 4",
                    "" if cancel == "first" else "MOVE 1 NODE 4", "2",
                    "MOVE 1 NODE 4" if cancel == "second" else "MOVE 2 NODE 4"])
    monkeypatch.setattr("builtins.input", lambda prompt: next(answers))
    if cancel == "none":
        bench.run_steps(b.motion, log, False)
    else:
        with pytest.raises(ControlError, match="确认词不匹配" if cancel == "second" else "取消"):
            bench.run_steps(b.motion, log, False)
    expected = {"enable": 0, "first": 0, "second": 1, "none": 2}[cancel]
    assert len(runs) == len([f for f in b.frames if f[0] == 0x204]) == expected
    if cancel == "enable":
        assert b.words == []
    rows = [json.loads(line) for line in log_stream.getvalue().splitlines()]
    assert [r["step"] for r in rows if r["event"] == "passed"] == list(
        range(1, {"enable": 3, "first": 4, "second": 5, "none": 6}[cancel]))


def test_partial_and_old_feedback_cannot_complete_motion(monkeypatch):
    b = BenchSamples(4, monkeypatch)
    b.ready()
    b.motion.execute_motion()
    b.tick(pos=b.motion.encoded.target_position)
    b.c.feedback.samples[4] = replace(b.s, cycle2=b.s.cycle2 - 1)
    assert b.motion.poll_motion().state == MotionState.TRIGGERING
    b.c.feedback.samples[4] = replace(b.s, cycle1=1, cycle2=1)
    assert b.motion.poll_motion().state == MotionState.TRIGGERING
    b.finish()


def test_default_robstride_torque_rejected_before_any_frame(monkeypatch):
    b = BenchSamples(4, monkeypatch)
    b.c.drives = make_drives({4: config.AXIS_CONFIG[4]})
    with pytest.raises(ControlError, match="调试力矩"):
        AxisMotion(b.c, 4)
    assert not b.frames


@pytest.mark.parametrize("fresh", [True, False])
def test_disable_after_verified_stop_rechecks_feedback_at_dispatch(monkeypatch, fresh):
    b = BenchSamples(1, monkeypatch)
    b.ready()
    m = b.motion
    m.execute_motion()
    m.request_stop()
    if fresh:
        m.disable_after_stop()
        assert b.words[-1] == 0 and not m.enabled_here and b.s.word == 0x40
    else:
        b.guard_race = lambda: b.tick(vel=65536)  # exceeds window after stop was confirmed
        with pytest.raises(ControlError, match="Disable not confirmed"):
            m.disable_after_stop()
        assert 0 not in b.words and not m.stop_confirmed
