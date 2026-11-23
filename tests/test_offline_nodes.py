"""Membership changes are confined to init. Check the actual frames on virtual CAN."""
from collections import defaultdict
import json
import struct

import pytest

from canopen_pp import config, main as terminal
from canopen_pp.controller import AllNodesOffline
from canopen_pp.drives import ControlError
from canopen_pp.pp_config import CONFIG_PATH, load_targets
from test_pp_motion import rpdo_batches
from test_virtual import SimDrive, system


@pytest.fixture(autouse=True)
def short_probe(monkeypatch):
    monkeypatch.setattr(config, "INITIAL_SDO_TIMEOUT", .15)
    monkeypatch.setattr(config, "SDO_TIMEOUT", .15)


@pytest.mark.parametrize("system", [
    range(1, 9), [1, 2, 3, 5, 6, 7, 8], range(1, 8), [1, 3, 6], [4], [5],
], ids=["all", "offline-4", "offline-8", "mixed", "only-4", "only-5"], indirect=True)
def test_only_online_nodes_participate(system, tmp_path):
    c, sims, trace, log = system
    c.parameter_test_only = False
    nodes = list(sims)
    missing = c.drives.keys() - sims.keys()
    feedback_by_sync = defaultdict(list)

    def record_tpdo(cob, data, stamp):
        feedback_by_sync[c.feedback.sync_stamp].append(cob)

    for n in c.drives:
        for base in (0x180, 0x280):
            c.bus.network.subscribe(base+n, record_tpdo)
    c.initialize()
    assert [data[1] for cob, data in trace if cob == 0 and data[0] == 0x80] == list(range(1, 9))
    assert c.online_nodes == sims.keys()
    assert c.offline_nodes == missing
    status = c.status().splitlines()
    assert status[-2:] == ["ONLINE: " + ", ".join(map(str, nodes)),
                           "OFFLINE: " + (", ".join(map(str, sorted(missing))) or "无")]
    for n in c.drives:
        assert status[n].split()[:2] == ["OFFLINE" if n in missing else "ONLINE", str(n)]
    for n in missing:
        assert any(f"Node {n} OFFLINE" in line for line in log)
        requests = [data[:4] for cob, data in trace if cob == 0x600+n and data[0] != 0x80]
        assert requests == [bytes.fromhex("40 18 10 01")]  # identity probe only
    assert [data[1] for cob, data in trace if cob == 0 and data[0] == 1] == nodes

    doc = json.loads(CONFIG_PATH.read_text())
    for n, row in doc["axes"].items():
        row["position"] = 12000 if int(n) in missing else 1
    path = tmp_path / "targets.json"
    path.write_text(json.dumps(doc))
    targets = load_targets(path, nodes=c.online_nodes)
    trace.clear()
    c.move_test(targets)
    c.enable(targets)
    c.pp_move(targets)
    assert set(c.completed_positions) == set(nodes)
    c.disable()
    c.close()

    controls = [0x400+n for n in nodes]
    load = [base+n for base in (0x200, 0x300, 0x400) for n in nodes]
    batches = rpdo_batches(trace)
    assert [[cob for cob, _ in batch] for batch in batches] == [
        load, controls, controls, controls, load, controls, controls, controls]
    assert all([data[0] for _, data in batch] == [cw]*len(nodes)
               for batch, cw in zip(batches[5:7], (31, 15), strict=True))
    assert all(s.pos == c.mechanical.target_to_raw(n, 1) for n, s in sims.items())
    assert all(s.word == 0x40 for s in sims.values())
    expected_tpdos = [base+n for base in (0x180, 0x280) for n in nodes]
    for _, _, stamp in sims[nodes[0]].frames[:-1]:  # exclude the final disable before close
        assert sorted(feedback_by_sync[stamp]) == expected_tpdos


@pytest.mark.parametrize("system", [()], indirect=True)
def test_all_offline(system):
    c, _, trace, _ = system
    with pytest.raises(AllNodesOffline, match="所有配置节点均 OFFLINE"):
        c.initialize()
    assert not c.initialized and not c.online_nodes and c.offline_nodes == set(c.drives)
    assert not c.clock.thread.is_alive()
    assert not any(cob == 0x80 or (cob == 0 and data[0] == 1) for cob, data in trace)


@pytest.mark.parametrize("system", [[2]], indirect=True)
def test_reconnect_requires_safe_reinit(system, monkeypatch):
    c, sims, trace, _ = system
    c.parameter_test_only = False
    c.initialize()
    old_clock = c.clock
    c.enable()
    net = sims[2].node.network
    recovered = SimDrive(net, c.drives[8])
    trace.clear()
    with pytest.raises(ControlError, match="disable"):
        c.initialize()
    send = c.bus.transmit
    attempts = []

    def rescan_during_move(cob, data, **kwargs):
        if cob == 0x402 and data[0] == 31:
            with pytest.raises(ControlError, match="执行中"):
                c.initialize()
            attempts.append(cob)
        return send(cob, data, **kwargs)

    with monkeypatch.context() as m:
        m.setattr(c.bus, "transmit", rescan_during_move)
        c.pp_move(load_targets())
    assert attempts == [0x402]
    assert c.online_nodes == {2} and not recovered.frames
    assert not any(cob == 0 or cob >= 0x600 for cob, _ in trace)
    c.disable()

    c.initialize()
    assert not old_clock.thread.is_alive() and c.clock.thread.is_alive()
    assert c.online_nodes == {2, 8} and 8 not in c.offline_nodes
    c.enable()
    c.pp_move(load_targets())
    assert [cw for cw, _, _ in recovered.frames] == [6, 7, 15, 15, 31, 15]


@pytest.mark.parametrize("system", [[2, 5]], indirect=True)
def test_loss_after_trigger_latches_group_fault(system, monkeypatch):
    c, sims, trace, _ = system
    c.parameter_test_only = False
    c.initialize()
    c.enable()
    send = c.bus.transmit

    def lose_tpdo(cob, data, **kwargs):
        if cob == 0x402 and data[0] == 31:
            sims[2].send_velocity = False
        return send(cob, data, **kwargs)

    monkeypatch.setattr(c.bus, "transmit", lose_tpdo)
    trace.clear()
    with pytest.raises(ControlError, match="Node 2"):
        c.pp_move(load_targets())
    assert c.pp_inhibited and c.online_nodes == {2, 5} and 2 not in c.offline_nodes
    assert rpdo_batches(trace)[-1] == [(0x402, bytes.fromhex("00 00 01")),
                                       (0x405, bytes.fromhex("01 00 01 00 00"))]
    with pytest.raises(ControlError, match="disable"):
        c.initialize()


@pytest.mark.parametrize("failure", ["wrong_vendor", "fault", "malformed_sdo"])
def test_response_errors_are_fatal(system, failure, monkeypatch):
    c, sims, trace, _ = system
    node = sims[5].node
    if failure == "wrong_vendor":
        node.set_data(0x1018, 1, struct.pack("<I", 0))
    elif failure == "fault":
        node.set_data(0x6041, 0, struct.pack("<H", 8))
    else:
        send = node.sdo.send_response
        monkeypatch.setattr(node.sdo, "send_response", lambda data: send(bytes([0x60]) + data[1:]))
    with pytest.raises(ControlError, match="Node 5"):
        c.initialize()
    assert 5 not in c.offline_nodes
    assert not any(cob == 0x80 or (cob == 0 and data[0] == 1) for cob, data in trace)


def test_no_response_during_remapping(system, monkeypatch):
    c, sims, trace, _ = system
    server = sims[3].node.sdo
    send = server.send_response

    def lose_write_response(data):
        if data[:4] != bytes.fromhex("60 00 16 00"):
            send(data)

    monkeypatch.setattr(server, "send_response", lose_write_response)
    c.initialize()
    assert c.offline_nodes == {3} and c.online_nodes == c.drives.keys() - {3}
    assert (0, bytes([1, 3])) not in trace


@pytest.mark.parametrize("system", [[5]], indirect=True)
def test_terminal_all_offline_status_and_retry(system, monkeypatch, capsys, tmp_path):
    c, sims, trace, _ = system
    c.parameter_test_only = False
    monkeypatch.setattr(config, "PARAMETER_TEST_ONLY", False)
    path = tmp_path / "targets.json"
    doc = json.loads(CONFIG_PATH.read_text())
    doc["axes"]["1"]["position"] = 12345  # outside the travel of this offline axis
    doc["axes"]["5"]["position"] = 1
    path.write_text(json.dumps(doc))
    monkeypatch.setattr(terminal, "CONFIG_PATH", path)
    monkeypatch.setattr(terminal, "open_controller", lambda *args: c)

    def commands():
        with monkeypatch.context() as m:
            m.setattr(sims[5].node.sdo, "send_response", lambda data: None)
            yield "init"
            yield "status"
        yield "init"
        yield "move-test"
        assert len(sims[5].frames) == 1
        yield "enable"
        yield "pp-move"
        assert sims[5].pos == c.mechanical.target_to_raw(5, 1)
        trace.clear()
        doc["axes"]["5"]["position"] = 12345
        path.write_text(json.dumps(doc))
        yield "pp-move"
        assert rpdo_batches(trace) == []
        doc["axes"]["5"]["position"] = 2
        path.write_text(json.dumps(doc))
        yield "pp-move"
        assert sims[5].pos == c.mechanical.target_to_raw(5, 2)
        yield "exit"

    script = commands()
    monkeypatch.setattr("builtins.input", lambda _: next(script))
    assert terminal.main() == 0
    output = capsys.readouterr()
    assert "所有配置节点均 OFFLINE" in output.err
    assert "OFFLINE: 1, 2, 3, 4, 5, 6, 7, 8" in output.out
    assert "ONLINE: 5" in output.out and "使能成功，在线节点 [5]" in output.out
    assert "pp-move 失败" in output.err and "Node 5" in output.err
