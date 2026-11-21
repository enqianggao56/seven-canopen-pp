"""Explicitly opt-in. Enables the selected real axes once and verifies disable."""
import os
import struct

import can
import pytest

from canopen_pp import config
from canopen_pp.main import connect


def verify_capture(trace, nodes):
    start = next(i for i, m in enumerate(trace) if m.arbitration_id == 0 and m.data[0] == 1)
    assert not any(m.arbitration_id == 0x80 for m in trace[:start])
    cycle, batches = [], []
    for m in trace[start:]:
        cob = m.arbitration_id
        assert not (0x601 <= cob <= 0x67F or 0x201 <= cob <= 0x27F or 0x301 <= cob <= 0x37F)
        if cob in [0x400+n for n in nodes]:
            n = cob - 0x400
            if n >= 4:
                cw, mode, torque = struct.unpack("<Hbh", m.data)
                assert torque == (0 if cw == 1 else config.AXIS_CONFIG[n].pp_torque_per_mille)
            else:
                cw, mode = struct.unpack("<Hb", m.data)
            assert mode == 1 and cw in (0, 1, 6, 7, 15)
            cycle.append((n, cw, m.timestamp))
        if cob == 0x80 and cycle:
            assert not m.data and [row[0] for row in cycle] == nodes
            gap = (m.timestamp-cycle[-1][2])*1000
            print(f"CAN 抓包 RPDO3 {[(n, w) for n, w, _ in cycle]} → SYNC，末帧间隔 {gap:.3f} ms")
            batches.append([row[1] for row in cycle])
            cycle = []
    assert not cycle
    assert batches[-4:] == [[cw]*len(nodes) for cw in (6, 7, 15)] + [
        [0 if n < 4 else 1 for n in nodes]]


@pytest.mark.parametrize("nodes,selection", [([4], "rs06"), (list(range(1, 8)), "seven")])
def test_hardware(nodes, selection):
    if config.PARAMETER_TEST_ONLY:
        pytest.skip("使能实机用例在 PARAMETER TEST ONLY 模式下停用")
    if os.getenv("CANOPEN_HARDWARE_TEST") != selection:
        pytest.skip(f"requires CANOPEN_HARDWARE_TEST={selection}")
    settings = {k: v for k, v in config.CAN_CONFIG.items() if k != "sync_period_ms"}
    with can.Bus(**settings) as monitor:
        trace = []
        listener = can.Notifier(monitor, [trace.append], timeout=.02)
        try:
            c = connect({n: config.AXIS_CONFIG[n] for n in nodes}, config.CAN_CONFIG)
            try:
                print("初始 TPDO\n" + c.status(), flush=True)
                c.enable()
                print("使能 TPDO\n" + c.status(), flush=True)
                c.disable()
                print("失能 TPDO\n" + c.status(), flush=True)
            finally:
                c.close()
        finally:
            listener.stop()
    verify_capture(trace, nodes)
