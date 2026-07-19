"""X42S 命令基类."""

import logging
from abc import ABC, abstractmethod
from time import sleep, time
from typing import Generic, Optional, TypeVar

from ..configs import (
    Address,
    Code,
    Protocol,
    StatusCode,
    SystemConstants,
    add_checksum,
    calculate_checksum,
)
from ..parameters import DeviceParams

logger = logging.getLogger(__name__)

T = TypeVar("T")


class CommandError(Exception):
    """命令执行错误."""

    pass


class FirmwareCapabilityError(Exception):
    """当前固件不支持该命令/能力."""

    pass


class Command(ABC, Generic[T]):
    """命令基类."""

    _code: Code
    _protocol: Optional[Protocol] = None
    _response_length: int = 4  # 默认: 地址 + 功能码 + 状态 + 校验

    def __init__(self, device: DeviceParams):
        """初始化命令.

        Args:
            device: 设备参数
        """
        self._timestamp = time()
        self._response: Optional[bytes] = None
        self._data: Optional[T] = None
        self._status: StatusCode = StatusCode.FORMAT_ERROR

        self.device = device
        self.address = device.address
        self.checksum_mode = device.checksum_mode
        self.delay = device.delay
        self.serial = device.serial_connection

        # 构建并执行命令
        self._command = self._build_command()
        self._execute()

    @abstractmethod
    def _build_command_body(self) -> bytes:
        """构建命令体(不含校验码)."""
        pass

    @abstractmethod
    def _parse_response(self, data: bytes) -> T:
        """解析响应数据."""
        pass

    def _build_command(self) -> bytes:
        """构建完整命令(含校验码)."""
        body = self._build_command_body()
        return add_checksum(body, self.checksum_mode)

    def _execute(self) -> None:
        """执行命令."""
        max_retries = SystemConstants().MAX_RETRIES
        tries = 0
        while tries < max_retries:
            try:
                # 清空缓冲区
                in_waiting = self.serial.in_waiting
                if in_waiting > 0:
                    stale_data = self.serial.read(in_waiting)
                    logger.debug(f"清空残留数据 ({in_waiting} 字节): {stale_data.hex()}")
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()

                # 发送命令
                logger.debug(f"发送命令 (地址={self.address}): {self._command.hex()}")
                self.serial.write(self._command)
                self.serial.flush()

                # 读取响应
                response = self._read_response()
                if response:
                    self._response = response
                    self._status = StatusCode.SUCCESS
                    break

            except Exception as e:
                logger.warning(f"命令执行失败 (尝试 {tries + 1}): {e}")
                tries += 1

            if self.delay:
                sleep(self.delay)

        if tries >= max_retries:
            logger.error("命令执行失败: 超过最大重试次数")

    def _read_response(self) -> Optional[bytes]:
        """读取响应."""
        expected_addr = 1 if self.address == Address.BROADCAST else self.address

        # 读取地址，允许跳过最多 8 个非预期字节（处理异步返回数据干扰）
        skipped = b""
        addr = None
        for _ in range(8):
            byte = self.serial.read(1)
            if not byte:
                if skipped:
                    logger.debug(f"跳过了非预期字节后超时: 跳过={skipped.hex()}")
                raise CommandError("未收到响应")
            if byte[0] == expected_addr:
                addr = byte
                break
            else:
                skipped += byte

        if addr is None:
            logger.debug(
                f"地址不匹配详情: 发送命令={self._command.hex()}, "
                f"期望地址=0x{expected_addr:02X}({expected_addr}), "
                f"跳过的字节={skipped.hex()} ({len(skipped)} 字节)"
            )
            raise CommandError(f"地址不匹配: 期望 {expected_addr}, 跳过了 {skipped.hex()}")

        if skipped:
            logger.debug(
                f"跳过了 {len(skipped)} 个非预期字节: {skipped.hex()}, "
                f"命令={self._command.hex()}"
            )

        # 读取功能码
        code = self.serial.read(1)
        if not code:
            raise CommandError("未收到功能码")

        logger.debug(f"收到功能码: 0x{code[0]:02X}")

        # 读取数据
        data_length = self._response_length - 3  # 减去地址、功能码、校验码
        data = self.serial.read(data_length) if data_length > 0 else b""

        # 读取校验码
        checksum = self.serial.read(1)
        if not checksum:
            raise CommandError("未收到校验码")

        # 验证校验码
        response_body = addr + code + data
        expected_checksum = calculate_checksum(response_body, self.checksum_mode)
        if checksum[0] != expected_checksum:
            raise CommandError(
                f"校验码不匹配: 期望 0x{expected_checksum:02X}, 收到 0x{checksum[0]:02X}"
            )

        # 解析数据
        if data:
            self._data = self._parse_response(data)

        return response_body + checksum

    @property
    def response(self) -> Optional[bytes]:
        """返回原始响应."""
        return self._response

    @property
    def data(self) -> Optional[T]:
        """返回解析后的数据."""
        return self._data

    @property
    def is_success(self) -> bool:
        """命令是否成功."""
        return self._status == StatusCode.SUCCESS

    @property
    def status(self) -> str:
        """返回状态字符串."""
        return self._status.name


class SimpleCommand(Command[bool]):
    """简单命令(只返回成功/失败)."""

    def _parse_response(self, data: bytes) -> bool:
        """解析响应."""
        if data[0] == StatusCode.SUCCESS:
            return True
        elif data[0] in (StatusCode.AT_ZERO, StatusCode.LIMIT_OR_HOME):
            logger.warning("零点/限位条件阻止动作 (0x%02X)", data[0])
            return False
        elif data[0] == StatusCode.PARAM_ERROR:
            logger.warning("命令参数错误")
            return False
        elif data[0] == StatusCode.FORMAT_ERROR:
            logger.warning("命令格式错误")
            return False
        return False

    @property
    def is_success(self) -> bool:
        """以返回状态字节为准(避免任意响应都被标为成功)."""
        return bool(self._data)


class ReadCommand(Command[T]):
    """读取命令基类."""

    def _build_command_body(self) -> bytes:
        """构建命令体."""
        return bytes([self.address, self._code])


class DynamicLengthCommand(Command[T]):
    """动态长度响应命令基类.

    用于响应长度在响应数据中指定的命令（如读取配置参数、系统状态等）。

    返回格式:
    - 字节1: 地址
    - 字节2: 功能码
    - 字节3: 字节数 (整个响应的总字节数，包括地址到校验码)
    - 字节4: 参数个数
    - 字节5-N: 数据
    - 字节N+1: 校验码
    """

    def _read_response(self) -> Optional[bytes]:
        """读取动态长度响应."""
        expected_addr = 1 if self.address == Address.BROADCAST else self.address

        skipped = b""
        addr = None
        for _ in range(8):
            byte = self.serial.read(1)
            if not byte:
                if skipped:
                    logger.debug(
                        f"[动态长度] 跳过了非预期字节后超时: 跳过={skipped.hex()}"
                    )
                raise CommandError("未收到响应")
            if byte[0] == expected_addr:
                addr = byte
                break
            else:
                skipped += byte

        if addr is None:
            logger.debug(
                f"[动态长度] 地址不匹配详情: 发送命令={self._command.hex()}, "
                f"期望地址=0x{expected_addr:02X}({expected_addr}), "
                f"跳过的字节={skipped.hex()} ({len(skipped)} 字节)"
            )
            raise CommandError(f"地址不匹配: 期望 {expected_addr}, 跳过了 {skipped.hex()}")

        if skipped:
            logger.debug(
                f"[动态长度] 跳过了 {len(skipped)} 个非预期字节: {skipped.hex()}, "
                f"命令={self._command.hex()}"
            )

        code = self.serial.read(1)
        if not code:
            raise CommandError("未收到功能码")

        logger.debug(f"收到功能码: 0x{code[0]:02X}")

        byte_count = self.serial.read(1)
        if not byte_count:
            raise CommandError("未收到字节数")

        total_response_length = byte_count[0]
        logger.debug(f"响应总字节数: {total_response_length}")

        param_count = self.serial.read(1)
        if not param_count:
            raise CommandError("未收到参数个数")

        logger.debug(f"参数个数: {param_count[0]}")

        data_length = total_response_length - 5
        remaining_data = self.serial.read(data_length)
        if len(remaining_data) < data_length:
            raise CommandError(
                f"数据不完整: 期望 {data_length} 字节, 收到 {len(remaining_data)} 字节"
            )

        checksum = self.serial.read(1)
        if not checksum:
            raise CommandError("未收到校验码")

        data = byte_count + param_count + remaining_data

        response_body = addr + code + data
        expected_checksum = calculate_checksum(response_body, self.checksum_mode)
        if checksum[0] != expected_checksum:
            raise CommandError(
                f"校验码不匹配: 期望 0x{expected_checksum:02X}, 收到 0x{checksum[0]:02X}"
            )

        if data:
            self._data = self._parse_response(data)

        return response_body + checksum
