"""Terminal command gating before initialization."""
import pytest

from canopen_pp import config
from canopen_pp.main import main


@pytest.fixture(autouse=True)
def parameter_test_mode(monkeypatch):
    monkeypatch.setattr(config, "PARAMETER_TEST_ONLY", True)


def test_terminal_before_init(monkeypatch, capsys):
    commands = iter(["status", "move-test", "enable", "start", "exit"])
    monkeypatch.setattr("builtins.input", lambda prompt: next(commands))
    assert main() == 0
    captured = capsys.readouterr()
    assert "禁止 enable" in captured.err
    output = captured.out
    assert output.count("CAN 尚未初始化") == 2
    assert "使能成功" not in output


def test_move_test_loads_valid_snapshot_after_repair(monkeypatch, tmp_path, capsys):
    import json
    from unittest.mock import Mock
    from canopen_pp import main as terminal
    from canopen_pp.pp_config import CONFIG_PATH

    path = tmp_path / "targets.json"
    repaired = json.loads(CONFIG_PATH.read_text())
    repaired["axes"]["1"]["position"] = 100
    path.write_text("{}")
    controller = Mock(online_nodes=range(1, 9))
    connect = Mock(return_value=controller)
    monkeypatch.setattr(terminal, "CONFIG_PATH", path)
    monkeypatch.setattr(terminal, "open_controller", connect)

    def commands():
        yield "init"
        yield "enable"
        yield "move-test"
        path.write_text(json.dumps(repaired))
        yield "move-test"
        yield "exit"

    script = commands()
    monkeypatch.setattr("builtins.input", lambda _: next(script))
    assert main() == 0
    assert "禁止 enable" in capsys.readouterr().err
    connect.assert_called_once()
    controller.enable.assert_not_called()
    controller.close.assert_called_once()
    controller.move_test.assert_called_once()
    targets, = controller.move_test.call_args.args
    assert len(targets.axes) == 8 and targets.axes[0].command.target_position == 100
