"""X42S 多电机命令."""

import logging
from typing import List

from ..configs import Address, ChecksumMode, Code, StatusCode, add_checksum
from ..parameters import DeviceParams
from .base import Command

logger = logging.getLogger(__name__)


class MultiMotor(Command[bool]):
    """多电机命令.

    对应命令: 5.3.1 多电机命令（X42S/X42S）
    发送: 00 AA + 字节长度 + 子命令... + 6B

    子命令须为已含校验码的完整帧。运动类命令仅地址1会回复确认。
    禁止重试：位置/速度子命令重复发送会导致多转。
    """

    _code = Code.MULTI_MOTOR
    _response_length = 4

    def __init__(
        self,
        device: DeviceParams,
        frames: List[bytes],
        expect_ack: bool = True,
    ):
        if not frames:
            raise ValueError("多电机命令至少需要一条子命令")
        self.frames = frames
        self.expect_ack = expect_ack
        device.address = Address(Address.BROADCAST)
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        payload = b"".join(self.frames)
        # 总字节数 = 整帧长度(地址+功能码+长度字段+子命令+外层校验)
        total_len = 5 + len(payload)
        return bytes([
            Address.BROADCAST,
            self._code,
            (total_len >> 8) & 0xFF,
            total_len & 0xFF,
        ]) + payload

    def _parse_response(self, data: bytes) -> bool:
        if not data:
            return True
        return data[0] == StatusCode.SUCCESS

    def _execute(self) -> None:
        # 多电机帧禁止重试
        try:
            in_waiting = self.serial.in_waiting
            if in_waiting > 0:
                self.serial.read(in_waiting)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            logger.debug(f"发送多电机命令: {self._command.hex()}")
            self.serial.write(self._command)
            self.serial.flush()
            if not self.expect_ack:
                self._status = StatusCode.SUCCESS
                self._data = True
                self._response = self._command
                return
            response = self._read_response()
            if response:
                self._response = response
                self._status = StatusCode.SUCCESS
            else:
                logger.warning("多电机命令未收到确认，但不会重发")
                self._status = StatusCode.SUCCESS
                self._data = True
        except Exception as e:
            logger.warning(f"多电机命令执行异常(不重发): {e}")
            self._status = StatusCode.SUCCESS
            self._data = True


def build_command_frame(
    body: bytes,
    checksum_mode: ChecksumMode = ChecksumMode.FIXED,
) -> bytes:
    """构建含校验码的完整命令帧(用于多电机命令子帧)."""
    return add_checksum(body, checksum_mode)
