"""Terminal command gating before initialization."""
from canopen_pp.main import main


def test_terminal_before_init(monkeypatch, capsys):
    commands = iter(["status", "move-test", "enable", "start", "exit"])
    monkeypatch.setattr("builtins.input", lambda prompt: next(commands))
    assert main() == 0
    captured = capsys.readouterr()
    assert "禁止 enable" in captured.err
    output = captured.out
    assert output.count("CAN 尚未初始化") == 2
    assert "All 7 axes enabled successfully" not in output


def test_terminal_uses_startup_snapshot_after_file_changes(monkeypatch, tmp_path, capsys):
    from unittest.mock import Mock
    from canopen_pp import main as terminal
    from canopen_pp.pp_config import CONFIG_PATH

    path = tmp_path / "targets.json"
    path.write_bytes(CONFIG_PATH.read_bytes())
    controller = Mock()
    connect = Mock(return_value=controller)
    monkeypatch.setattr(terminal, "CONFIG_PATH", path)
    monkeypatch.setattr(terminal, "connect", connect)
    commands = iter(["init", "enable", "move-test", "exit"])

    def command(_):
        path.write_text("{}")
        return next(commands)

    monkeypatch.setattr("builtins.input", command)
    assert main() == 0
    assert "禁止 enable" in capsys.readouterr().err
    connect.assert_called_once()
    controller.enable.assert_not_called()
    controller.close.assert_called_once()
    controller.move_test.assert_called_once()
    targets, = controller.move_test.call_args.args
    assert len(targets.axes) == 7 and targets.axes[0].command.target_position == 100
