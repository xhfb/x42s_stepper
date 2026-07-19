"""X42S 触发动作命令."""

from ..configs import Code, Protocol
from .base import SimpleCommand


class CalibrateEncoder(SimpleCommand):
    """触发编码器校准.

    对应命令: 5.2.1 触发编码器校准
    发送: 01 06 45 6B
    返回: 01 06 02 6B
    """

    _code = Code.CAL_ENCODER
    _protocol = Protocol.CAL_ENCODER

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])


class Restart(SimpleCommand):
    """重启电机.

    对应命令: 5.2.2 重启电机（X42S/X42S）
    发送: 01 08 97 6B
    返回: 01 08 02 6B
    """

    _code = Code.RESTART
    _protocol = Protocol.RESTART

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])


class ZeroPosition(SimpleCommand):
    """将当前位置角度清零.

    对应命令: 5.2.3 将当前位置角度清零
    发送: 01 0A 6D 6B
    返回: 01 0A 02 6B
    """

    _code = Code.ZERO_POSITION
    _protocol = Protocol.ZERO_POSITION

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])


class ClearProtection(SimpleCommand):
    """解除堵转/过热/过流保护.

    对应命令: 5.2.4 解除堵转/过热/过流保护
    发送: 01 0E 52 6B
    返回: 01 0E 02 6B
    """

    _code = Code.CLEAR_PROTECTION
    _protocol = Protocol.CLEAR_PROTECTION

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])


class FactoryReset(SimpleCommand):
    """恢复出厂设置.

    对应命令: 5.2.5 恢复出厂设置
    发送: 01 0F 5F 6B
    返回: 01 0F 02 6B
    """

    _code = Code.FACTORY_RESET
    _protocol = Protocol.FACTORY_RESET

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])
