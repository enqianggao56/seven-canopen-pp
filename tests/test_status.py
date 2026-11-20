"""Terminal display using decoded TPDOs; the CAN network is never connected."""
import struct
from dataclasses import replace
from types import SimpleNamespace

import canopen
import pytest

from canopen_pp import config, main
from canopen_pp.controller import Controller
from canopen_pp.drives import make_drives


HEADINGS = ("Node", "Motor", "Statusword", "CiA402 State", "6061",
            "Position", "Raw(inc)", "Velocity", "Vel(raw)")
TIME = 100.0


@pytest.fixture
def offline(monkeypatch):
    monkeypatch.setattr("canopen_pp.pdo.time", lambda: TIME)
    monkeypatch.setattr("canopen_pp.pdo.monotonic", lambda: TIME)
    monkeypatch.setattr("canopen_pp.controller.monotonic", lambda: TIME)

    def create():
        bus = SimpleNamespace(network=canopen.Network(), operational_started=False, close=lambda: None)
        c = Controller(bus, make_drives(config.AXIS_CONFIG), 100)
        c.feedback.sync(TIME - 2 * config.FEEDBACK_TIMEOUT)
        return c

    return create


def inject(c, node, pos, vel, *, pos_age=0, vel_age=0):
    word = 0x4637 if node <= 3 else 0x0037
    c.bus.network.notify(0x180 + node, struct.pack("<Hib", word, pos, 1), TIME - pos_age)
    c.bus.network.notify(0x280 + node, struct.pack("<i", vel), TIME - vel_age)


def cells(table, node):
    lines = table.splitlines()
    starts = [lines[0].index(label) for label in HEADINGS]
    line = lines[node]
    return [line[a:b].strip() for a, b in zip(starts, starts[1:] + [len(line)])]


def test_init_and_status_show_physical_and_raw_values(offline, monkeypatch, capsys):
    c = offline()
    cases = [
        (1, 3276800, 65536, "600.000 mm", "0.732 mm/s"),
        (2, 2730667, 32768, "500.000 mm", "0.366 mm/s"),
        (3, 1379705, 65536, "200.000 mm", "0.580 mm/s"),
        (4, 2048, 100, "45.000 deg", "60.000 deg/s"),
        (5, 1365, 50, "29.993 deg", "30.000 deg/s"),
        (6, 8192, 200, "180.000 deg", "120.000 deg/s"),
        (7, 81920, 300, "10.000 mm", "1.000 mm/s"),
    ]
    for node, pos, vel, _, _ in cases:
        inject(c, node, pos, vel)
    before = c.feedback.snapshot()
    table = c.status()
    for node, pos, vel, position, velocity in cases:
        assert cells(table, node) == [
            str(node), config.AXIS_CONFIG[node].family, f"0x{before[node].word:04X}",
            "Operation Enabled", "1", position, str(pos), velocity, str(vel)]
    commands = iter(["init", "status", "exit"])
    monkeypatch.setattr(main, "connect", lambda *args: c)
    monkeypatch.setattr("builtins.input", lambda prompt: next(commands))
    assert main.main() == 0
    assert capsys.readouterr().out.count(table) == 2
    assert c.feedback.snapshot() == before


def test_tpdo_freshness_is_independent(offline):
    c = offline()
    late = 1.5 * config.FEEDBACK_TIMEOUT
    inject(c, 1, 3276800, -65536, pos_age=late)
    inject(c, 2, -65536, 123, vel_age=late)
    inject(c, 4, 8192, -50)
    table = c.status()
    assert cells(table, 1)[2:] == ["0x4637", "TPDO1 STALE", "--", "--", "--", "-0.732 mm/s", "-65536"]
    assert cells(table, 2)[5:] == ["-12.000 mm", "-65536", "--", "TPDO2 STALE"]
    assert cells(table, 3)[2:] == ["----", "TPDO1 STALE", "--", "--", "--", "--", "TPDO2 STALE"]
    assert cells(table, 4)[5:] == ["180.000 deg", "8192", "-30.000 deg/s", "-50"]


def test_config_errors_are_local_and_preserve_raw_fields(offline, monkeypatch):
    monkeypatch.setitem(config.MECHANICAL_CONFIG, 1, replace(config.MECHANICAL_CONFIG[1], velocity_unit="bad"))
    monkeypatch.setitem(config.MECHANICAL_CONFIG, 2, replace(config.MECHANICAL_CONFIG[2], direction=0))
    # Valid configuration with an unrepresentable measured position but a valid zero speed.
    monkeypatch.setitem(config.MECHANICAL_CONFIG, 3, replace(config.MECHANICAL_CONFIG[3],
                        increments_per_revolution=1, travel_per_revolution=1e308))
    c = offline()
    for node in (1, 2, 3, 4):
        inject(c, node, 65536, 0)
    table = c.status()
    assert cells(table, 1)[5:] == ["12.000 mm", "65536", "CONV ERROR", "0"]
    assert cells(table, 2)[5:] == ["CONV ERROR", "65536", "CONV ERROR", "0"]
    assert cells(table, 3)[5:] == ["CONV ERROR", "65536", "0.000 mm/s", "0"]
    assert cells(table, 4)[5:] == ["1440.000 deg", "65536", "0.000 deg/s", "0"]
    assert "Node 1 Velocity" in table and "velocity_unit='bad'" in table
    assert "Node 2 Position" in table and "direction" in table
    assert "Node 3 Position" in table and "float" in table


def test_status_does_not_hide_programming_errors(offline, monkeypatch):
    c = offline()
    inject(c, 4, 42, 1)

    def fail(*args):
        raise RuntimeError("unexpected converter failure")

    monkeypatch.setattr(c.mechanical, "raw_to_velocity", fail)
    with pytest.raises(RuntimeError, match="unexpected converter failure"):
        c.status()
