"""X42S 运动控制命令."""

from ..configs import (
    Address,
    Code,
    EnableFlag,
    Protocol,
    SyncFlag,
)
from ..parameters import (
    DeviceParams,
    EmmFastPositionParams,
    EmmJogParams,
    EmmPositionParams,
    XFastPositionParams,
    XJogCurrentLimitParams,
    XJogParams,
    XPositionDirectLimitedParams,
    XPositionDirectParams,
    XPositionTrapLimitedParams,
    XPositionTrapParams,
    XTorqueLimitedParams,
    XTorqueParams,
)
from .base import SimpleCommand


class Enable(SimpleCommand):
    """电机使能控制.

    对应命令: 5.3.2 电机使能控制
    发送: 01 F3 AB 01 00 6B (使能)
    返回: 01 F3 02 6B
    """

    _code = Code.ENABLE
    _protocol = Protocol.ENABLE

    def __init__(
        self,
        device: DeviceParams,
        enable: bool = True,
        sync_flag: SyncFlag = SyncFlag.IMMEDIATE,
    ):
        self.enable = enable
        self.sync_flag = sync_flag
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            EnableFlag.ENABLE if self.enable else EnableFlag.DISABLE,
            self.sync_flag,
        ])


class Disable(Enable):
    """电机失能(松轴)."""

    def __init__(self, device: DeviceParams, sync_flag: SyncFlag = SyncFlag.IMMEDIATE):
        super().__init__(device, enable=False, sync_flag=sync_flag)


class EStop(SimpleCommand):
    """立即停止.

    对应命令: 立即停止
    发送: 01 FE 98 00 6B
    返回: 01 FE 02 6B
    """

    _code = Code.ESTOP
    _protocol = Protocol.ESTOP

    def __init__(self, device: DeviceParams, sync_flag: SyncFlag = SyncFlag.IMMEDIATE):
        self.sync_flag = sync_flag
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol, self.sync_flag])


class SyncMove(SimpleCommand):
    """触发多机同步运动.

    对应命令: 5.3.14 触发多机同步运动
    发送: 00 FF 66 6B
    返回: 01 FF 02 6B (仅地址1回复)
    """

    _code = Code.SYNC_MOVE
    _protocol = Protocol.SYNC_MOVE

    def __init__(self, device: DeviceParams):
        # 广播发送、期望地址1回复；不得永久改写调用方的 address
        saved = device.address
        device.address = Address(Address.BROADCAST)
        try:
            super().__init__(device)
        finally:
            device.address = saved

    def _build_command_body(self) -> bytes:
        return bytes([Address.BROADCAST, self._code, self._protocol])


class EmmJog(SimpleCommand):
    """速度模式控制 (Emm固件).

    对应命令: 5.3.7 速度模式控制（Emm）
    发送: 01 F6 ... 6B
    返回: 01 F6 02 6B
    """

    _code = Code.JOG

    def __init__(self, device: DeviceParams, params: EmmJogParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class EmmPosition(SimpleCommand):
    """位置模式控制 (Emm固件).

    对应命令: 5.3.12 位置模式控制（Emm）
    发送: 01 FD ... 6B
    返回: 01 FD 02 6B
    """

    _code = Code.POSITION

    def __init__(self, device: DeviceParams, params: EmmPositionParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class XTorque(SimpleCommand):
    """力矩模式控制 (X固件).

    对应命令: 5.3.3 力矩模式控制（X）, 功能码 0xF5
    """

    _code = Code.TORQUE

    def __init__(self, device: DeviceParams, params: XTorqueParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class XTorqueLimited(SimpleCommand):
    """力矩模式限速控制 (X固件).

    对应命令: 5.3.4 力矩模式限速控制（X）, 功能码 0xC5
    """

    _code = Code.TORQUE_LIMITED

    def __init__(self, device: DeviceParams, params: XTorqueLimitedParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class XJog(SimpleCommand):
    """速度模式控制 (X固件).

    对应命令: 5.3.5 速度模式控制（X）, 功能码 0xF6
    """

    _code = Code.JOG

    def __init__(self, device: DeviceParams, params: XJogParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class XJogCurrentLimit(SimpleCommand):
    """速度模式限电流控制 (X固件).

    对应命令: 5.3.6 速度模式限电流控制（X）, 功能码 0xC6
    """

    _code = Code.JOG_CURRENT_LIMIT

    def __init__(self, device: DeviceParams, params: XJogCurrentLimitParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class XPositionDirect(SimpleCommand):
    """直通限速位置模式控制 (X固件).

    对应命令: 5.3.8 直通限速位置模式控制（X）, 功能码 0xFB
    """

    _code = Code.POSITION_DIRECT

    def __init__(self, device: DeviceParams, params: XPositionDirectParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class XPositionDirectLimited(SimpleCommand):
    """直通限速位置模式限电流控制 (X固件).

    对应命令: 5.3.9 直通限速位置模式限电流控制（X）, 功能码 0xCB
    """

    _code = Code.POSITION_DIRECT_LIMITED

    def __init__(self, device: DeviceParams, params: XPositionDirectLimitedParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class XPositionTrap(SimpleCommand):
    """梯形曲线加减速位置模式控制 (X固件).

    对应命令: 5.3.10 梯形曲线加减速位置模式控制（X）, 功能码 0xFD
    """

    _code = Code.POSITION

    def __init__(self, device: DeviceParams, params: XPositionTrapParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class XPositionTrapLimited(SimpleCommand):
    """梯形曲线位置模式限电流控制 (X固件).

    对应命令: 5.3.11 梯形曲线加减速位置模式限电流控制（X）, 功能码 0xCD
    """

    _code = Code.POSITION_TRAP_LIMITED

    def __init__(self, device: DeviceParams, params: XPositionTrapLimitedParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class EmmConfigureFastPosition(SimpleCommand):
    """快速位置模式设参 (Emm, V2.0.0+, F1)."""

    _code = Code.FAST_POSITION_CFG

    def __init__(self, device: DeviceParams, params: EmmFastPositionParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class EmmFastPositionPulse(SimpleCommand):
    """快速位置模式发脉冲 (Emm, V2.0.0+, FC). 有符号 int32 脉冲."""

    _code = Code.FAST_POSITION_PULSE

    def __init__(self, device: DeviceParams, pulses: int):
        if not -0x80000000 <= pulses <= 0x7FFFFFFF:
            raise ValueError("脉冲数必须在有符号 int32 范围内")
        self.pulses = pulses
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.pulses.to_bytes(
            4, "big", signed=True
        )


class XConfigureFastPosition(SimpleCommand):
    """快速位置模式设参 (X, V2.0.0+, F1)."""

    _code = Code.FAST_POSITION_CFG

    def __init__(self, device: DeviceParams, params: XFastPositionParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.params.bytes


class XFastPositionAngle(SimpleCommand):
    """快速位置模式发角度 (X, V2.0.0+, FC). 有符号 int32, 单位 0.1°."""

    _code = Code.FAST_POSITION_PULSE

    def __init__(self, device: DeviceParams, angle_raw: int):
        if not -0x80000000 <= angle_raw <= 0x7FFFFFFF:
            raise ValueError("角度原始值必须在有符号 int32 范围内")
        self.angle_raw = angle_raw
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code]) + self.angle_raw.to_bytes(
            4, "big", signed=True
        )
