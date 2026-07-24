"""串口逻辑帧 ↔ CAN 扩展帧分包/拼包（手册 V1.0.5 §4.2）."""

from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

# 每包: [Code] + 最多 7 字节载荷，总长 ≤ 8
_MAX_CHUNK = 7


def make_can_id(addr: int, packet: int) -> int:
    """扩展帧 ID = (Addr << 8) | Packet."""
    if not 0 <= addr <= 255:
        raise ValueError(f"地址越界: {addr}")
    if not 0 <= packet <= 255:
        raise ValueError(f"包序号越界: {packet}")
    return ((addr & 0xFF) << 8) | (packet & 0xFF)


def parse_can_id(can_id: int) -> Tuple[int, int]:
    """从扩展帧 ID 解析 (addr, packet)."""
    return (can_id >> 8) & 0xFF, can_id & 0xFF


def pack_logical_frame(logical_frame: bytes) -> List[Tuple[int, bytes]]:
    """将含地址的逻辑帧拆成 CAN 帧列表 [(can_id, data), ...].

    逻辑帧格式: [Addr][Code][payload...][Checksum]
    CAN 数据: 每包 [Code] + 最多 7 字节后续载荷
    """
    if len(logical_frame) < 2:
        raise ValueError("逻辑帧过短")
    addr = logical_frame[0]
    payload = logical_frame[1:]  # Code + body + checksum
    code = payload[0]
    body = payload[1:]

    frames: List[Tuple[int, bytes]] = []
    if not body:
        frames.append((make_can_id(addr, 0), bytes([code])))
        return frames

    packet = 0
    for i in range(0, len(body), _MAX_CHUNK):
        chunk = body[i : i + _MAX_CHUNK]
        frames.append((make_can_id(addr, packet), bytes([code]) + chunk))
        packet += 1
    return frames


def reassemble_logical_frame(addr: int, frames: Sequence[Tuple[int, bytes]]) -> bytes:
    """将同一地址的 CAN 帧拼回逻辑帧 [Addr][Code][...].

    frames: [(can_id, data), ...]，packet 可乱序，按 ID 低 8 位排序。
    """
    if not frames:
        raise ValueError("无帧可拼装")

    ordered = sorted(frames, key=lambda item: parse_can_id(item[0])[1])
    code: Optional[int] = None
    body = bytearray()

    for can_id, data in ordered:
        frame_addr, _packet = parse_can_id(can_id)
        if frame_addr != addr:
            raise ValueError(f"地址不匹配: expect {addr}, got {frame_addr}")
        if not data:
            raise ValueError("空 CAN 数据")
        if code is None:
            code = data[0]
        elif data[0] != code:
            raise ValueError(f"功能码不一致: {code:#x} vs {data[0]:#x}")
        body.extend(data[1:])

    assert code is not None
    return bytes([addr, code]) + bytes(body)


def filter_addr_frames(
    addr: int, messages: Iterable[Tuple[int, bytes]]
) -> List[Tuple[int, bytes]]:
    """只保留指定地址的帧."""
    return [m for m in messages if parse_can_id(m[0])[0] == addr]
