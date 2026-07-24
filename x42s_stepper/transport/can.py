"""SocketCAN Transport（经典 CAN，扩展帧）."""

from __future__ import annotations

import logging
import threading
import time
from typing import Dict, Optional

from .base import Transport, TransportError
from .can_codec import make_can_id, pack_logical_frame, parse_can_id, reassemble_logical_frame

logger = logging.getLogger(__name__)


class CanBusError(TransportError):
    """CAN 总线错误（兼容原 x42s_can.CanBusError）."""


class CanTransport(Transport):
    """共享 SocketCAN 总线（经典 CAN，扩展帧）."""

    def __init__(
        self,
        channel: str = "can0",
        bitrate: int = 500000,
        receive_own_messages: bool = False,
        bustype: str = "socketcan",
        timeout: float = 0.4,
    ):
        try:
            import can
        except ImportError as exc:
            raise ImportError(
                "CanTransport 需要 python-can，请安装: pip install 'x42s_stepper[can]'"
            ) from exc

        self.channel = channel
        self.bitrate = bitrate
        self.default_timeout = timeout
        self._lock = threading.RLock()
        self._bus = can.Bus(
            channel=channel,
            interface=bustype,
            receive_own_messages=receive_own_messages,
            fd=False,
        )
        logger.info("打开 CAN: %s (bitrate 期望 %s)", channel, bitrate)

    def close(self) -> None:
        with self._lock:
            self._bus.shutdown()

    def __enter__(self) -> "CanTransport":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def flush(self) -> None:
        """清空接收缓冲中的残留帧."""
        with self._lock:
            while self._bus.recv(timeout=0.0) is not None:
                pass

    def send(self, frame: bytes, inter_frame_gap: float = 0.0) -> None:
        """发送一条逻辑命令（自动分包），不等待应答."""
        frames = pack_logical_frame(frame)
        with self._lock:
            self._send_frames(frames, inter_frame_gap)

    # 兼容原 CanBus API
    send_logical = send

    def request(
        self,
        frame: bytes,
        expected_len: int,
        reply_addr: Optional[int] = None,
        timeout: Optional[float] = None,
        *,
        expect_code: Optional[int] = None,
        inter_frame_gap: float = 0.0,
    ) -> bytes:
        """发送逻辑命令并收集同地址同功能码应答，拼成逻辑帧."""
        if len(frame) < 2:
            raise ValueError("逻辑帧过短")
        send_addr = frame[0]
        addr = send_addr if reply_addr is None else reply_addr
        if addr == 0:
            addr = 1
        code = frame[1] if expect_code is None else expect_code
        frames = pack_logical_frame(frame)
        collected: Dict[int, bytes] = {}
        wait = self.default_timeout if timeout is None else timeout
        deadline = time.monotonic() + wait

        with self._lock:
            self.flush()
            self._send_frames(frames, inter_frame_gap)

            while time.monotonic() < deadline:
                remaining = max(0.0, deadline - time.monotonic())
                msg = self._bus.recv(timeout=min(0.05, remaining))
                if msg is None:
                    continue
                if not msg.is_extended_id:
                    continue
                frame_addr, packet = parse_can_id(msg.arbitration_id)
                if frame_addr != addr:
                    continue
                data = bytes(msg.data)
                if not data or data[0] != code:
                    logger.debug(
                        "忽略 RX id=0x%X data=%s",
                        msg.arbitration_id,
                        data.hex(" ").upper(),
                    )
                    continue

                # 忽略自发自收（loopback / 本地回显）
                sent = next(
                    (d for cid, d in frames if parse_can_id(cid)[1] == packet), None
                )
                if sent is not None and data == sent and send_addr == addr:
                    logger.debug(
                        "忽略回显 RX id=0x%X data=%s",
                        msg.arbitration_id,
                        data.hex(" ").upper(),
                    )
                    continue

                logger.debug(
                    "RX id=0x%X data=%s",
                    msg.arbitration_id,
                    data.hex(" ").upper(),
                )
                collected[packet] = data

                payload_bytes = sum(len(d) - 1 for d in collected.values())
                logical_len = 2 + payload_bytes
                if logical_len < expected_len:
                    continue

                logical = reassemble_logical_frame(
                    addr,
                    [(make_can_id(addr, p), d) for p, d in sorted(collected.items())],
                )
                if len(logical) >= expected_len and logical[1] == code:
                    return logical[:expected_len]

        raise CanBusError(
            f"超时: send_addr={send_addr} reply_addr={addr} code=0x{code:02X} "
            f"expected_len={expected_len} 已收包={sorted(collected.keys())}"
        )

    def request_dynamic(
        self,
        frame: bytes,
        length_index: int = 2,
        timeout: Optional[float] = None,
    ) -> bytes:
        """动态长度应答：逻辑帧第 length_index 字节为整帧总长（含地址）."""
        if len(frame) < 2:
            raise ValueError("逻辑帧过短")
        send_addr = frame[0]
        code = frame[1]
        frames = pack_logical_frame(frame)
        collected: Dict[int, bytes] = {}
        wait = self.default_timeout if timeout is None else timeout
        deadline = time.monotonic() + wait
        total: Optional[int] = None

        with self._lock:
            self.flush()
            self._send_frames(frames, 0.0)

            while time.monotonic() < deadline:
                remaining = max(0.0, deadline - time.monotonic())
                msg = self._bus.recv(timeout=min(0.05, remaining))
                if msg is None or not msg.is_extended_id:
                    continue
                frame_addr, packet = parse_can_id(msg.arbitration_id)
                if frame_addr != send_addr:
                    continue
                data = bytes(msg.data)
                if not data or data[0] != code:
                    continue
                sent = next(
                    (d for cid, d in frames if parse_can_id(cid)[1] == packet), None
                )
                if sent is not None and data == sent:
                    continue
                collected[packet] = data
                logical = reassemble_logical_frame(
                    send_addr,
                    [
                        (make_can_id(send_addr, p), d)
                        for p, d in sorted(collected.items())
                    ],
                )
                if len(logical) > length_index:
                    total = logical[length_index]
                if total is not None and len(logical) >= total:
                    return logical[:total]

        raise CanBusError(
            f"动态长度超时: addr={send_addr} code=0x{code:02X} "
            f"total={total} 已收包={sorted(collected.keys())}"
        )

    def _send_frames(self, frames, inter_frame_gap: float) -> None:
        import can

        for can_id, data in frames:
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=True,
                is_fd=False,
            )
            logger.debug("TX id=0x%X data=%s", can_id, data.hex(" ").upper())
            self._bus.send(msg)
            if inter_frame_gap > 0:
                time.sleep(inter_frame_gap)


# 兼容旧名
CanBus = CanTransport
