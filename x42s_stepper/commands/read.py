"""X42S 读取系统参数命令."""

from typing import Optional, Union

from ..configs import Address, Code, FirmwareType, Protocol
from ..parameters import (
    DMX512Params,
    DeviceParams,
    EmmConfigParams,
    EmmPIDParams,
    EmmSystemStatusParams,
    HomeMotorStatus,
    IOStatus,
    MotorRHParams,
    MotorStatus,
    OptionStatus,
    ProtectionThreshold,
    VersionParams,
    XConfigParams,
    XPIDParams,
    XSystemStatusParams,
    to_int,
    to_signed_int,
)
from ..units import signed_error_deg, signed_position_deg, signed_speed_rpm
from .base import Command, CommandError, DynamicLengthCommand


class GetVersion(Command[VersionParams]):
    """读取固件版本和硬件版本.

    对应命令: 5.5.2 读取固件版本和硬件版本
    发送: 01 1F 6B
    返回: 01 1F + 4字节数据 + 6B
    """

    _code = Code.GET_VERSION
    _response_length = 7

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> VersionParams:
        return VersionParams.from_bytes(data)


class GetMotorRH(Command[MotorRHParams]):
    """读取相电阻和相电感.

    对应命令: 5.5.3 读取相电阻和相电感
    发送: 01 20 6B
    返回: 01 20 + 4字节数据 + 6B
    """

    _code = Code.GET_MOTOR_RH
    _response_length = 7

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> MotorRHParams:
        return MotorRHParams.from_bytes(data)


class GetBusVoltage(Command[int]):
    """读取总线电压.

    对应命令: 5.5.4 读取总线电压
    发送: 01 24 6B
    返回: 01 24 + 2字节数据 + 6B
    """

    _code = Code.GET_BUS_VOLTAGE
    _response_length = 5

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> int:
        """返回总线电压(mV)."""
        return to_int(data)


class GetBusCurrent(Command[int]):
    """读取总线电流.

    对应命令: 5.5.5 读取总线电流（X42S/X42S）
    发送: 01 26 6B
    返回: 01 26 + 2字节数据 + 6B
    """

    _code = Code.GET_BUS_CURRENT
    _response_length = 5

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> int:
        """返回总线电流(mA)."""
        return to_int(data)


class GetPhaseCurrent(Command[int]):
    """读取相电流.

    对应命令: 5.5.6 读取相电流
    发送: 01 27 6B
    返回: 01 27 + 2字节数据 + 6B
    """

    _code = Code.GET_PHASE_CURRENT
    _response_length = 5

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> int:
        """返回相电流(mA)."""
        return to_int(data)


class GetEncoder(Command[int]):
    """读取线性化编码器值.

    对应命令: 5.5.7 读取经过线性化校准后的编码器值
    发送: 01 31 6B
    返回: 01 31 + 2字节数据 + 6B
    """

    _code = Code.GET_ENCODER
    _response_length = 5

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> int:
        """返回编码器值(0-65535表示0-360度)."""
        return to_int(data)


class GetPulseCount(Command[int]):
    """读取输入脉冲数.

    对应命令: 5.5.8 读取输入脉冲数
    发送: 01 32 6B
    返回: 01 32 + 5字节数据 + 6B
    """

    _code = Code.GET_PULSE_COUNT
    _response_length = 8

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> int:
        """返回输入脉冲数(带符号)."""
        return to_signed_int(data)


class GetTargetPositionRaw(Command[int]):
    """读取电机目标位置原始值(带符号整数).

    对应命令: 5.5.9 读取电机目标位置
    """

    _code = Code.GET_TARGET_POSITION
    _response_length = 8

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> int:
        return to_signed_int(data)


class GetTargetPosition(Command[float]):
    """读取电机目标位置(度, 固件感知换算).

    对应命令: 5.5.9 读取电机目标位置
    发送: 01 33 6B
    返回: 01 33 + 5字节数据 + 6B
    """

    _code = Code.GET_TARGET_POSITION
    _response_length = 8

    def __init__(
        self,
        device: DeviceParams,
        firmware_type: FirmwareType = FirmwareType.EMM_FIRMWARE,
    ):
        self.firmware_type = firmware_type
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> float:
        return signed_position_deg(data[0], to_int(data[1:]), self.firmware_type)


class GetRealtimeTarget(Command[float]):
    """读取电机实时设定的目标位置(度, 固件感知换算).

    对应命令: 5.5.10 读取电机实时设定的目标位置
    发送: 01 34 6B
    返回: 01 34 + 5字节数据 + 6B
    """

    _code = Code.GET_REALTIME_TARGET
    _response_length = 8

    def __init__(
        self,
        device: DeviceParams,
        firmware_type: FirmwareType = FirmwareType.EMM_FIRMWARE,
    ):
        self.firmware_type = firmware_type
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> float:
        return signed_position_deg(data[0], to_int(data[1:]), self.firmware_type)


class GetRealtimeSpeed(Command[float]):
    """读取电机实时转速(RPM, 固件感知换算).

    对应命令: 5.5.11 读取电机实时转速
    发送: 01 35 6B
    返回: 01 35 + 3字节数据 + 6B

    Emm: 整数 RPM; X: 0.1RPM 原始值.
    """

    _code = Code.GET_REALTIME_SPEED
    _response_length = 6

    def __init__(
        self,
        device: DeviceParams,
        firmware_type: FirmwareType = FirmwareType.EMM_FIRMWARE,
    ):
        self.firmware_type = firmware_type
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> float:
        return signed_speed_rpm(data[0], to_int(data[1:3]), self.firmware_type)


class GetRealtimePosition(Command[float]):
    """读取电机实时位置(度, 固件感知换算).

    对应命令: 5.5.13 读取电机实时位置
    发送: 01 36 6B
    返回: 01 36 + 5字节数据 + 6B
    """

    _code = Code.GET_REALTIME_POSITION
    _response_length = 8

    def __init__(
        self,
        device: DeviceParams,
        firmware_type: FirmwareType = FirmwareType.EMM_FIRMWARE,
    ):
        self.firmware_type = firmware_type
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> float:
        return signed_position_deg(data[0], to_int(data[1:]), self.firmware_type)


class GetPositionError(Command[float]):
    """读取电机位置误差(度, 固件感知换算).

    对应命令: 5.5.14 读取电机位置误差
    发送: 01 37 6B
    返回: 01 37 + 5字节数据 + 6B

    Emm: (raw*360)/65536; X: 0.01° 单位.
    """

    _code = Code.GET_POSITION_ERROR
    _response_length = 8

    def __init__(
        self,
        device: DeviceParams,
        firmware_type: FirmwareType = FirmwareType.EMM_FIRMWARE,
    ):
        self.firmware_type = firmware_type
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> float:
        return signed_error_deg(data[0], to_int(data[1:]), self.firmware_type)


class GetBatteryVoltage(Command[int]):
    """读取电池电压 (X42S).

    对应命令: 读取电池电压
    发送: 01 38 6B
    返回: 01 38 + 2字节数据 + 6B
    """

    _code = Code.GET_BATTERY_VOLTAGE
    _response_length = 5

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> int:
        """返回电池电压(mV)."""
        return to_int(data)


class GetTemperature(Command[int]):
    """读取驱动温度.

    对应命令: 5.5.12 读取驱动温度（X42S/X42S）
    发送: 01 39 6B
    返回: 01 39 + 2字节数据 + 6B
    """

    _code = Code.GET_TEMPERATURE
    _response_length = 5

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> int:
        """返回温度(°C, 带符号).

        手册写 00=负/01=正；X42S V2.0.5 对约 27°C 返回 ``00 1B``，
        故按实机将 00 视为正、01 视为负。
        """
        sign = 1 if data[0] == 0 else -1
        return sign * data[1]


class GetMotorStatus(Command[MotorStatus]):
    """读取电机状态标志.

    对应命令: 5.5.15 读取电机状态标志
    发送: 01 3A 6B
    返回: 01 3A + 1字节数据 + 6B
    """

    _code = Code.GET_MOTOR_STATUS
    _response_length = 4

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> MotorStatus:
        return MotorStatus.from_byte(data[0])


class GetHomeMotorStatus(Command[HomeMotorStatus]):
    """读取回零状态标志 + 电机状态标志.

    对应命令: 5.5.16 读取回零状态标志+电机状态标志（X42S/X42S）
    发送: 01 3C 6B
    返回: 01 3C + 2字节数据 + 6B
    """

    _code = Code.GET_HOME_MOTOR_STATUS
    _response_length = 5

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> HomeMotorStatus:
        return HomeMotorStatus.from_bytes(data)


class GetIOStatus(Command[IOStatus]):
    """读取引脚 IO 电平状态.

    对应命令: 5.5.17 读取引脚IO电平状态（X42S/X42S）
    发送: 01 3D 6B
    返回: 01 3D + 1字节数据 + 6B
    """

    _code = Code.GET_IO_STATUS
    _response_length = 4

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> IOStatus:
        return IOStatus.from_byte(data[0])


class TimedReturn(Command[Optional[bytes]]):
    """定时返回信息命令.

    对应命令: 5.5.1 定时返回信息命令（X42S/X42S）
    发送: 01 11 18 + 信息功能码 + 定时时间(ms) + 6B
    定时时间为0时停止返回，确认帧为 Addr 11 6B。
    """

    _code = Code.TIMED_RETURN
    _protocol = Protocol.TIMED_RETURN

    _INFO_RESPONSE_LENGTH = {
        Code.GET_VERSION: 7,
        Code.GET_MOTOR_RH: 7,
        Code.GET_BUS_VOLTAGE: 5,
        Code.GET_BUS_CURRENT: 5,
        Code.GET_PHASE_CURRENT: 5,
        Code.GET_ENCODER: 5,
        Code.GET_PULSE_COUNT: 8,
        Code.GET_TARGET_POSITION: 8,
        Code.GET_REALTIME_TARGET: 8,
        Code.GET_REALTIME_SPEED: 6,
        Code.GET_REALTIME_POSITION: 8,
        Code.GET_POSITION_ERROR: 8,
        Code.GET_BATTERY_VOLTAGE: 5,
        Code.GET_TEMPERATURE: 5,
        Code.GET_MOTOR_STATUS: 4,
        Code.GET_HOME_STATUS: 4,
        Code.GET_HOME_MOTOR_STATUS: 5,
        Code.GET_IO_STATUS: 4,
    }

    def __init__(self, device: DeviceParams, info_code: int, interval_ms: int = 0):
        self.info_code = info_code & 0xFF
        self.interval_ms = max(0, min(interval_ms, 0xFFFF))
        # 实机(V2.0.5): 开启/停止均先回 Addr 11 02 6B；开启后另有异步数据流。
        # 手册示例“停止回 Addr 11 6B”与实机不符，这里以实机 ACK 为准。
        self._response_length = 4
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            self.info_code,
            (self.interval_ms >> 8) & 0xFF,
            self.interval_ms & 0xFF,
        ])

    def _parse_response(self, data: bytes) -> Optional[bytes]:
        return data if data else None


class GetEmmPID(Command[EmmPIDParams]):
    """读取PID参数 (Emm固件).

    对应命令: 5.6.16 读取PID参数（Emm）
    发送: 01 21 6B
    返回: 01 21 + 12字节数据 + 6B
    """

    _code = Code.GET_PID
    _response_length = 15

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> EmmPIDParams:
        return EmmPIDParams.from_bytes(data)


class GetXPID(Command[XPIDParams]):
    """读取PID参数 (X固件).

    对应命令: 5.6.14 读取PID参数（X）
    发送: 01 21 6B
    返回: 01 21 + 16字节数据 + 6B
    """

    _code = Code.GET_PID
    _response_length = 19

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> XPIDParams:
        return XPIDParams.from_bytes(data)


class GetPID(Command[Union[EmmPIDParams, XPIDParams]]):
    """读取PID参数 (按固件类型选择解析)."""

    _code = Code.GET_PID

    def __init__(
        self,
        device: DeviceParams,
        firmware_type: FirmwareType = FirmwareType.EMM_FIRMWARE,
    ):
        self.firmware_type = firmware_type
        if firmware_type == FirmwareType.X_FIRMWARE:
            self._response_length = 19
        else:
            self._response_length = 15
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> Union[EmmPIDParams, XPIDParams]:
        if self.firmware_type == FirmwareType.X_FIRMWARE:
            return XPIDParams.from_bytes(data)
        return EmmPIDParams.from_bytes(data)


class ProbeConfigFirmware(DynamicLengthCommand[FirmwareType]):
    """通过驱动配置块总字节数探测固件类型.

    发送与读配置相同: Addr 42 6C Chk。
    返回字节数 ``0x25`` → X，``0x21`` → Emm。

    X42S V2.0.x 上选项参数 0x1A 的 FwType 位不可靠（Emm 时仍可能为 0），
    必须以本命令为准。
    """

    _code = Code.GET_CONFIG
    _protocol = Protocol.GET_CONFIG

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])

    def _parse_response(self, data: bytes) -> FirmwareType:
        byte_count = data[0]
        if byte_count == 0x25:
            return FirmwareType.X_FIRMWARE
        if byte_count == 0x21:
            return FirmwareType.EMM_FIRMWARE
        raise CommandError(
            f"无法识别固件配置布局: byte_count=0x{byte_count:02X}"
        )


class GetEmmConfig(DynamicLengthCommand[EmmConfigParams]):
    """读取驱动配置参数 (Emm固件).

    对应命令: 5.8.5 读取驱动配置参数（Emm）
    发送: 01 42 6C 6B
    """

    _code = Code.GET_CONFIG
    _protocol = Protocol.GET_CONFIG

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])

    def _parse_response(self, data: bytes) -> EmmConfigParams:
        # data[0] = 字节数, data[1] = 参数个数, data[2:] = 实际数据
        return EmmConfigParams.from_bytes(data[2:])


class GetXConfig(DynamicLengthCommand[XConfigParams]):
    """读取驱动配置参数 (X固件).

    对应命令: 5.8.3 读取驱动配置参数（X）
    发送: 01 42 6C 6B
    """

    _code = Code.GET_CONFIG
    _protocol = Protocol.GET_CONFIG

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])

    def _parse_response(self, data: bytes) -> XConfigParams:
        return XConfigParams.from_bytes(data[2:])


class GetConfig(DynamicLengthCommand[Union[EmmConfigParams, XConfigParams]]):
    """读取驱动配置参数 (按返回字节数自动选择 X/Emm 布局)."""

    _code = Code.GET_CONFIG
    _protocol = Protocol.GET_CONFIG

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])

    def _parse_response(self, data: bytes) -> Union[EmmConfigParams, XConfigParams]:
        if data[0] == 0x25:
            return XConfigParams.from_bytes(data[2:])
        return EmmConfigParams.from_bytes(data[2:])


class GetEmmSystemStatus(DynamicLengthCommand[EmmSystemStatusParams]):
    """读取系统状态参数 (Emm固件).

    对应命令: 5.8.2 读取系统状态参数（Emm）
    发送: 01 43 7A 6B
    """

    _code = Code.GET_SYS_STATUS
    _protocol = Protocol.GET_SYS_STATUS

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])

    def _parse_response(self, data: bytes) -> EmmSystemStatusParams:
        return EmmSystemStatusParams.from_bytes(data[2:])


class GetXSystemStatus(DynamicLengthCommand[XSystemStatusParams]):
    """读取系统状态参数 (X固件).

    对应命令: 5.8.1 读取系统状态参数（X）
    发送: 01 43 7A 6B
    """

    _code = Code.GET_SYS_STATUS
    _protocol = Protocol.GET_SYS_STATUS

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])

    def _parse_response(self, data: bytes) -> XSystemStatusParams:
        return XSystemStatusParams.from_bytes(data[2:])


class GetSystemStatus(
    DynamicLengthCommand[Union[EmmSystemStatusParams, XSystemStatusParams]]
):
    """读取系统状态参数 (按返回字节数自动选择 X/Emm 布局)."""

    _code = Code.GET_SYS_STATUS
    _protocol = Protocol.GET_SYS_STATUS

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])

    def _parse_response(
        self, data: bytes
    ) -> Union[EmmSystemStatusParams, XSystemStatusParams]:
        if data[0] == 0x25:
            return XSystemStatusParams.from_bytes(data[2:])
        return EmmSystemStatusParams.from_bytes(data[2:])


class GetOptionStatus(Command[OptionStatus]):
    """读取选项参数状态.

    对应命令: 5.6.4 读取选项参数状态（X42S/X42S）
    发送: 01 1A 6B
    返回: 01 1A + 2字节数据 + 6B (实机为双字节，含锁定等级)
    """

    _code = Code.GET_OPTION_STATUS
    _response_length = 5

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> OptionStatus:
        return OptionStatus.from_bytes(data)


class GetProtectionThreshold(Command[ProtectionThreshold]):
    """读取过热过流保护检测阈值.

    对应命令: 5.6.22 读取过热过流保护检测阈值（X42S/X42S）
    发送: 01 13 6B
    """

    _code = Code.GET_PROTECTION_THRESHOLD
    _response_length = 9

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> ProtectionThreshold:
        return ProtectionThreshold.from_bytes(data)


class GetHeartbeatTime(Command[int]):
    """读取心跳保护功能时间.

    对应命令: 5.6.24 读取心跳保护功能时间（X42S/X42S）
    发送: 01 16 6B
    """

    _code = Code.GET_HEARTBEAT_TIME
    _response_length = 7

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> int:
        """返回心跳保护时间(ms)."""
        return to_int(data[0:4])


class GetPositionWindow(Command[float]):
    """读取位置到达窗口.

    对应命令: 5.6.20 读取位置到达窗口（X42S/X42S）
    发送: 01 41 6B
    返回: 01 41 + 2字节数据 + 6B
    """

    _code = Code.GET_POSITION_WINDOW
    _response_length = 5

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> float:
        """返回位置到达窗口(度)."""
        return to_int(data[0:2]) * 0.1


class GetIntegralStiffness(Command[int]):
    """读取积分限幅/刚性系数.

    对应命令: 5.6.26 读取积分限幅/刚性系数（X42S/X42S）
    """

    _code = Code.GET_INTEGRAL_STIFFNESS
    _response_length = 7

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> int:
        return to_int(data[0:4])


class GetCollisionReturnAngle(Command[float]):
    """读取碰撞回零返回角度.

    对应命令: 5.6.28 读取碰撞回零返回角度（X42S/X42S）
    """

    _code = Code.GET_COLLISION_RETURN_ANGLE
    _response_length = 5

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code])

    def _parse_response(self, data: bytes) -> float:
        """返回角度(度); 0 表示按电流检测返回."""
        return to_int(data[0:2]) * 0.1


class GetDMX512Params(Command[DMX512Params]):
    """读取 DMX512 协议参数.

    对应命令: 5.6.18 读取DMX512协议参数（X42S/X42S）
    发送: 01 49 78 6B
    """

    _code = Code.GET_DMX512_PARAM
    _protocol = Protocol.GET_DMX512_PARAM
    _response_length = 17

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, self._protocol])

    def _parse_response(self, data: bytes) -> DMX512Params:
        return DMX512Params.from_bytes(data)


class BroadcastGetID(Command[int]):
    """广播读取ID地址.

    对应命令: 5.6.30 广播读取ID地址（X42S/X42S）
    发送: 00 15 6B
    返回: 01 15 01 6B
    """

    _code = Code.BROADCAST_GET_ID
    _response_length = 4

    def __init__(self, device: DeviceParams):
        device.address = Address(Address.BROADCAST)
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([Address.BROADCAST, self._code])

    def _parse_response(self, data: bytes) -> int:
        """返回电机ID."""
        return data[0]
