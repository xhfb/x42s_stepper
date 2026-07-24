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
)
from ..parameters import DeviceParams
from ..transport.base import TransportError

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
        self.transport = device.transport
        # 兼容仍读取 self.serial 的旧代码
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

    def _reply_addr(self) -> int:
        return 1 if self.address == Address.BROADCAST else int(self.address)

    def _apply_response(self, response: bytes) -> None:
        """保存响应并解析数据段（地址与校验之间）."""
        self._response = response
        self._status = StatusCode.SUCCESS
        if len(response) >= 3:
            data = response[2:-1]
            if data:
                self._data = self._parse_response(data)

    def _execute(self) -> None:
        """执行命令."""
        max_retries = SystemConstants().MAX_RETRIES
        tries = 0
        while tries < max_retries:
            try:
                self.transport.flush()
                logger.debug(
                    "发送命令 (地址=%s): %s", self.address, self._command.hex()
                )
                response = self.transport.request(
                    self._command,
                    expected_len=self._response_length,
                    reply_addr=self._reply_addr(),
                )
                if response:
                    self._apply_response(response)
                    break

            except Exception as e:
                logger.warning("命令执行失败 (尝试 %s): %s", tries + 1, e)
                tries += 1

            if self.delay:
                sleep(self.delay)

        if tries >= max_retries:
            logger.error("命令执行失败: 超过最大重试次数")

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

    def _execute(self) -> None:
        """执行动态长度命令."""
        max_retries = SystemConstants().MAX_RETRIES
        tries = 0
        while tries < max_retries:
            try:
                self.transport.flush()
                logger.debug(
                    "发送命令 (地址=%s): %s", self.address, self._command.hex()
                )
                response = self.transport.request_dynamic(
                    self._command,
                    length_index=2,
                )
                if response:
                    self._apply_response(response)
                    break

            except (TransportError, Exception) as e:
                logger.warning("命令执行失败 (尝试 %s): %s", tries + 1, e)
                tries += 1

            if self.delay:
                sleep(self.delay)

        if tries >= max_retries:
            logger.error("命令执行失败: 超过最大重试次数")
