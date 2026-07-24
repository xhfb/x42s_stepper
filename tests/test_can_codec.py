"""CAN codec 单元测试（无需硬件）."""

from x42s_stepper.transport.can_codec import (
    pack_logical_frame,
    reassemble_logical_frame,
)


def test_short_frame():
    frames = pack_logical_frame(bytes.fromhex("01 36 6B"))
    assert frames == [(0x100, bytes.fromhex("36 6B"))]


def test_enable_frame():
    frames = pack_logical_frame(bytes.fromhex("01 F3 AB 01 00 6B"))
    assert frames == [(0x100, bytes.fromhex("F3 AB 01 00 6B"))]


def test_long_position_manual_example():
    logical = bytes.fromhex("01 FD 01 0F A0 00 00 01 FA 00 00 00 6B")
    frames = pack_logical_frame(logical)
    assert len(frames) == 2
    assert frames[0] == (0x100, bytes.fromhex("FD 01 0F A0 00 00 01 FA"))
    assert frames[1] == (0x101, bytes.fromhex("FD 00 00 00 6B"))
    assert reassemble_logical_frame(1, frames) == logical


def test_addr2():
    frames = pack_logical_frame(bytes.fromhex("02 1F 6B"))
    assert frames == [(0x200, bytes.fromhex("1F 6B"))]
