"""传输层抽象：串口 / CAN 共用收发契约."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional


class TransportError(Exception):
    """传输层错误（超时、校验失败、总线错误等）."""


class Transport(ABC):
    """逻辑帧收发抽象（与底层介质无关）."""

    @abstractmethod
    def flush(self) -> None:
        """清空接收缓冲中的残留数据."""

    @abstractmethod
    def send(self, frame: bytes) -> None:
        """发送逻辑帧，不等待应答."""

    @abstractmethod
    def request(
        self,
        frame: bytes,
        expected_len: int,
        reply_addr: Optional[int] = None,
        timeout: Optional[float] = None,
    ) -> bytes:
        """发送逻辑帧并读取定长应答.

        Args:
            frame: 完整逻辑帧 [Addr][Code][...][Chk]
            expected_len: 期望应答总长度（含地址与校验）
            reply_addr: 应答地址；None 表示与发送 Addr 相同
            timeout: 超时（秒）；None 使用实现默认值
        """

    @abstractmethod
    def request_dynamic(
        self,
        frame: bytes,
        length_index: int = 2,
        timeout: Optional[float] = None,
    ) -> bytes:
        """发送逻辑帧并读取动态长度应答.

        应答第 length_index 字节为整帧总长（含地址到校验）。
        """

    def close(self) -> None:
        """关闭底层资源（可选实现）."""

    def __enter__(self) -> "Transport":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
