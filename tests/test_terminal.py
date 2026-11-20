"""Terminal command gating before initialization."""
from canopen_pp.main import main


def test_terminal_before_init(monkeypatch, capsys):
    commands = iter(["status", "enable", "start", "exit"])
    monkeypatch.setattr("builtins.input", lambda prompt: next(commands))
    assert main() == 0
    output = capsys.readouterr().out
    assert output.count("CAN 尚未初始化") == 2
    assert "All 7 axes enabled successfully" not in output
