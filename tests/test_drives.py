import struct

import pytest

from canopen_pp import config
from canopen_pp.drives import ControlError, make_drives, state
from canopen_pp.pdo import Feedback, desired_cob, slots


@pytest.mark.parametrize(("word", "name"), [
    (0, "Not Ready"), (0x4270, "Switch On Disabled"), (0x4231, "Ready to Switch On"),
    (0x0233, "Switched On"), (0x0027, "Operation Enabled"), (0x0037, "Operation Enabled"),
    (0x0017, "Quick Stop Active"), (0x000F, "Fault Reaction Active"), (0x0008, "Fault"),
    (0x002A, "Unknown"),
])
def test_state(word, name):
    assert state(word) == name


def test_layout_and_flags():
    for n, drive in make_drives(config.AXIS_CONFIG).items():
        rp1, rp2, rp3, tp1, tp2 = slots(drive)
        assert rp1.words == (0x607A0020, 0x60810020)
        assert rp2.words == (0x60830020, 0x60840020)
        control = (0x60400010, 0x60600008)
        assert rp3.words == (control if n < 4 else control + (0x60710010,))
        assert tp1.words == (0x60410010, 0x60630020 if n < 4 else 0x60640020, 0x60610008)
        assert tp2.words == (0x606C0020,)
        assert [struct.calcsize(s.fmt) for s in slots(drive)] == [8, 8, 3 if n < 4 else 5, 7, 4]
        assert desired_cob(0xC0000000, tp1) == 0x40000180 + n


def test_configured_torque_per_axis():
    drives = make_drives(config.AXIS_CONFIG)
    assert drives[5].control_data(6, 250) == bytes.fromhex("06 00 01 FA 00")
    assert drives[4].control_data(6, 100) == bytes.fromhex("06 00 01 64 00")
    assert drives[5].control_data(1) == bytes.fromhex("01 00 01 00 00")
    for value in (None, -1, 0, 1001, 20.5, True):
        with pytest.raises(ControlError, match="Node 5"):
            drives[5].control_data(6, value)


def test_invalid_lengths_and_cycle_barrier():
    from time import time
    import canopen
    drives = make_drives({4: config.AXIS_CONFIG[4]})
    net = canopen.Network()
    fb = Feedback(net, drives)
    s1 = struct.pack("<Hib", 0x37, -64000, 1)
    stamp = time()
    fb.sync(stamp)
    net.notify(0x184, s1, stamp+.001)
    net.notify(0x284, struct.pack("<i", -123), stamp+.002)
    s = fb.snapshot()[4]
    assert s.fresh(.6) and (s.pos, s.vel, s.mode, s.cycle1) == (-64000, -123, 1, stamp)
    fb.sync(stamp+.01)
    net.notify(0x184, s1, stamp+.005)
    assert fb.snapshot()[4].cycle1 == stamp
    for cob, short in ((0x184, s1[:-1]), (0x284, b"\0"*5)):
        net.notify(cob, short, stamp+.015)
    s = fb.snapshot()[4]
    assert s.received1 == s.received2 == float("-inf") and s.word == 0x37
