"""串口 Transport：包装 pyserial，保持跳过杂字节与校验行为."""

from __future__ import annotations

import logging
from typing import Optional

from serial import Serial

from ..configs import Address, ChecksumMode, calculate_checksum
from .base import Transport, TransportError

logger = logging.getLogger(__name__)


class SerialTransport(Transport):
    """基于 pyserial.Serial 的逻辑帧收发."""

    def __init__(
        self,
        serial_connection: Serial,
        checksum_mode: ChecksumMode = ChecksumMode.FIXED,
        timeout: Optional[float] = None,
    ):
        self._serial = serial_connection
        self.checksum_mode = checksum_mode
        if timeout is not None:
            self._serial.timeout = timeout

    @property
    def serial(self) -> Serial:
        """底层 Serial 对象."""
        return self._serial

    def flush(self) -> None:
        in_waiting = self._serial.in_waiting
        if in_waiting > 0:
            stale = self._serial.read(in_waiting)
            logger.debug("清空残留数据 (%s 字节): %s", in_waiting, stale.hex())
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def send(self, frame: bytes) -> None:
        logger.debug("发送命令: %s", frame.hex())
        self._serial.write(frame)
        self._serial.flush()

    def request(
        self,
        frame: bytes,
        expected_len: int,
        reply_addr: Optional[int] = None,
        timeout: Optional[float] = None,
    ) -> bytes:
        send_addr = frame[0] if frame else Address.DEFAULT
        expected_addr = (
            send_addr
            if reply_addr is None
            else reply_addr
        )
        if expected_addr == Address.BROADCAST:
            expected_addr = 1

        old_timeout = self._serial.timeout
        if timeout is not None:
            self._serial.timeout = timeout
        try:
            self.send(frame)
            return self._read_fixed(expected_addr, expected_len, frame)
        finally:
            if timeout is not None:
                self._serial.timeout = old_timeout

    def request_dynamic(
        self,
        frame: bytes,
        length_index: int = 2,
        timeout: Optional[float] = None,
    ) -> bytes:
        send_addr = frame[0] if frame else Address.DEFAULT
        expected_addr = 1 if send_addr == Address.BROADCAST else send_addr

        old_timeout = self._serial.timeout
        if timeout is not None:
            self._serial.timeout = timeout
        try:
            self.send(frame)
            return self._read_dynamic(expected_addr, length_index, frame)
        finally:
            if timeout is not None:
                self._serial.timeout = old_timeout

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()

    def _read_addr(self, expected_addr: int, frame: bytes, tag: str = "") -> bytes:
        skipped = b""
        for _ in range(8):
            byte = self._serial.read(1)
            if not byte:
                if skipped:
                    logger.debug(
                        "%s跳过了非预期字节后超时: 跳过=%s", tag, skipped.hex()
                    )
                raise TransportError("未收到响应")
            if byte[0] == expected_addr:
                if skipped:
                    logger.debug(
                        "%s跳过了 %s 个非预期字节: %s, 命令=%s",
                        tag,
                        len(skipped),
                        skipped.hex(),
                        frame.hex(),
                    )
                return byte
            skipped += byte

        logger.debug(
            "%s地址不匹配详情: 发送命令=%s, 期望地址=0x%02X, 跳过的字节=%s",
            tag,
            frame.hex(),
            expected_addr,
            skipped.hex(),
        )
        raise TransportError(
            f"地址不匹配: 期望 {expected_addr}, 跳过了 {skipped.hex()}"
        )

    def _verify_checksum(self, body: bytes, checksum: int) -> None:
        expected = calculate_checksum(body, self.checksum_mode)
        if checksum != expected:
            raise TransportError(
                f"校验码不匹配: 期望 0x{expected:02X}, 收到 0x{checksum:02X}"
            )

    def _read_fixed(self, expected_addr: int, expected_len: int, frame: bytes) -> bytes:
        addr = self._read_addr(expected_addr, frame)
        code = self._serial.read(1)
        if not code:
            raise TransportError("未收到功能码")
        logger.debug("收到功能码: 0x%02X", code[0])

        data_length = expected_len - 3
        data = self._serial.read(data_length) if data_length > 0 else b""
        if data_length > 0 and len(data) < data_length:
            raise TransportError(
                f"数据不完整: 期望 {data_length} 字节, 收到 {len(data)} 字节"
            )

        checksum = self._serial.read(1)
        if not checksum:
            raise TransportError("未收到校验码")

        body = addr + code + data
        self._verify_checksum(body, checksum[0])
        return body + checksum

    def _read_dynamic(
        self, expected_addr: int, length_index: int, frame: bytes
    ) -> bytes:
        tag = "[动态长度] "
        addr = self._read_addr(expected_addr, frame, tag=tag)
        code = self._serial.read(1)
        if not code:
            raise TransportError("未收到功能码")
        logger.debug("收到功能码: 0x%02X", code[0])

        # 先读到 length_index（通常为字节数），再按总长读完
        # 已有 addr(0) + code(1)，下一步读 index=2 的长度字节
        prefix = addr + code
        while len(prefix) <= length_index:
            b = self._serial.read(1)
            if not b:
                raise TransportError("未收到长度字节")
            prefix += b

        total = prefix[length_index]
        logger.debug("响应总字节数: %s", total)
        remaining = total - len(prefix)
        if remaining < 1:
            raise TransportError(f"非法响应长度: {total}")

        rest = self._serial.read(remaining)
        if len(rest) < remaining:
            raise TransportError(
                f"数据不完整: 期望 {remaining} 字节, 收到 {len(rest)} 字节"
            )

        response = prefix + rest
        body, checksum = response[:-1], response[-1]
        self._verify_checksum(body, checksum)
        return response
