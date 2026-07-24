"""X42S 双固件步进电机参数类.

基于 ZDT_X42S_manual_v1.1（X42S 第二代闭环步进电机使用说明）。
"""

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

from .configs import (
    Address,
    BaudRate,
    CanRate,
    ChecksumMode,
    ControlMode,
    Direction,
    DirLevel,
    EnableLevel,
    FirmwareType,
    HomingDirection,
    HomingMode,
    MotionMode,
    MotorType,
    PulsePortMode,
    ResponseMode,
    SerialPortMode,
    StallProtect,
    SyncFlag,
)

# 库侧速度上限 (X42S 手册: Emm 可达 6000RPM+, X 约 3000RPM)
MAX_SPEED_RPM = 6000  # Emm / Turbo
MAX_SPEED_RPM_X = 3000
MAX_SPEED_RAW_X = 30000  # X 固件 0.1RPM → 0-3000.0 RPM
from .units import (
    emm_raw_to_deg,
    signed_from_sign_byte,
    x_error_raw_to_deg,
    x_raw_to_deg,
    x_speed_raw_to_rpm,
)

if TYPE_CHECKING:
    from serial import Serial

    from .transport.base import Transport



def to_int(data: bytes) -> int:
    """将字节转换为整数."""
    return int.from_bytes(data, "big")


def to_signed_int(data: bytes) -> int:
    """将带符号字节转换为有符号整数.

    第一个字节为符号位: 00=正, 01=负
    """
    if len(data) < 2:
        return to_int(data)
    sign = -1 if data[0] == 1 else 1
    return sign * to_int(data[1:])


def _u16(value: int) -> bytes:
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def _u32(value: int) -> bytes:
    return bytes([
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ])


@dataclass
class DeviceParams:
    """设备参数类.

    Args:
        transport: 传输层（SerialTransport / CanTransport）
        address: 电机地址 (1-255, 0为广播地址)
        checksum_mode: 校验模式
        delay: 通讯延迟(秒)
    """

    transport: "Transport"
    address: int = Address.DEFAULT
    checksum_mode: ChecksumMode = ChecksumMode.FIXED
    delay: Optional[float] = None

    def __post_init__(self):
        if isinstance(self.address, int) and not isinstance(self.address, Address):
            self.address = Address(self.address)

    @property
    def serial_connection(self) -> Optional["Serial"]:
        """兼容旧代码：仅 SerialTransport 返回底层 Serial，否则 None."""
        from .transport.serial import SerialTransport

        if isinstance(self.transport, SerialTransport):
            return self.transport.serial
        return None


@dataclass
class EmmJogParams:
    """速度模式参数 (Emm固件).

    对应命令: 5.3.7 速度模式控制（Emm）
    库侧允许 0-6000 RPM（X42S Emm 手册可达 6000RPM+）。
    """

    direction: Direction = Direction.CW
    speed: int = 100  # 0-MAX_SPEED_RPM
    acceleration: int = 10  # 0-255档位
    sync_flag: SyncFlag = SyncFlag.IMMEDIATE

    def __post_init__(self):
        if not 0 <= self.speed <= MAX_SPEED_RPM:
            raise ValueError(f"速度必须在 0-{MAX_SPEED_RPM} RPM 之间")
        if not 0 <= self.acceleration <= 255:
            raise ValueError("加速度必须在 0-255 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.direction,
            (self.speed >> 8) & 0xFF,
            self.speed & 0xFF,
            self.acceleration,
            self.sync_flag,
        ])


@dataclass
class EmmPositionParams:
    """位置模式参数 (Emm固件).

    对应命令: 5.3.12 位置模式控制（Emm）
    """

    direction: Direction = Direction.CW
    speed: int = 100  # 0-MAX_SPEED_RPM
    acceleration: int = 10  # 0-255档位
    pulse_count: int = 3200  # 脉冲数
    motion_mode: MotionMode = MotionMode.RELATIVE_LAST
    sync_flag: SyncFlag = SyncFlag.IMMEDIATE

    def __post_init__(self):
        if not 0 <= self.speed <= MAX_SPEED_RPM:
            raise ValueError(f"速度必须在 0-{MAX_SPEED_RPM} RPM 之间")
        if not 0 <= self.acceleration <= 255:
            raise ValueError("加速度必须在 0-255 之间")
        if not 0 <= self.pulse_count <= 0xFFFFFFFF:
            raise ValueError("脉冲数必须在 0-4294967295 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.direction,
            (self.speed >> 8) & 0xFF,
            self.speed & 0xFF,
            self.acceleration,
            (self.pulse_count >> 24) & 0xFF,
            (self.pulse_count >> 16) & 0xFF,
            (self.pulse_count >> 8) & 0xFF,
            self.pulse_count & 0xFF,
            self.motion_mode,
            self.sync_flag,
        ])


@dataclass
class EmmFastPositionParams:
    """快速位置模式设参 (Emm, V2.0.0+, F1).

    对应命令: 5.3.13 快速位置模式控制（Emm）— F1。
    方向由后续 FC 有符号脉冲隐含。
    """

    speed: int = 100  # RPM
    acceleration: int = 10  # 档位
    motion_mode: MotionMode = MotionMode.RELATIVE_LAST
    sync_flag: SyncFlag = SyncFlag.IMMEDIATE

    def __post_init__(self):
        if not 0 <= self.speed <= MAX_SPEED_RPM:
            raise ValueError(f"速度必须在 0-{MAX_SPEED_RPM} RPM 之间")
        if not 0 <= self.acceleration <= 255:
            raise ValueError("加速度必须在 0-255 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            (self.speed >> 8) & 0xFF,
            self.speed & 0xFF,
            self.acceleration,
            self.motion_mode,
            self.sync_flag,
        ])


@dataclass
class XFastPositionParams:
    """快速位置模式设参 (X, V2.0.0+, F1).

    对应命令: 5.3.14 快速位置模式控制（X）— F1。
    后续 FC 发有符号 0.1° 角度。
    """

    accel_rpm_s: int = 1000
    decel_rpm_s: int = 1000
    max_speed_raw: int = 10000  # 0.1RPM
    motion_mode: MotionMode = MotionMode.RELATIVE_LAST
    sync: SyncFlag = SyncFlag.IMMEDIATE
    max_current_ma: int = 3000

    def __post_init__(self):
        if not 0 <= self.accel_rpm_s <= 0xFFFF:
            raise ValueError("加速加速度必须在 0-65535 RPM/S 之间")
        if not 0 <= self.decel_rpm_s <= 0xFFFF:
            raise ValueError("减速加速度必须在 0-65535 RPM/S 之间")
        if not 0 <= self.max_speed_raw <= MAX_SPEED_RAW_X:
            raise ValueError(
                f"最大速度原始值必须在 0-{MAX_SPEED_RAW_X} (0-{MAX_SPEED_RPM_X}.0 RPM) 之间"
            )
        if not 0 <= self.max_current_ma <= 5000:
            raise ValueError("最大电流必须在 0-5000 mA 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            *_u16(self.accel_rpm_s),
            *_u16(self.decel_rpm_s),
            *_u16(self.max_speed_raw),
            self.motion_mode,
            self.sync,
            *_u16(self.max_current_ma),
        ])


@dataclass
class XJogParams:
    """速度模式参数 (X固件).

    对应命令: 5.3.5 速度模式控制（X）
    acceleration_rpm_s: RPM/S; speed_raw: 0.1RPM 单位 (0-30000 → 0-3000.0RPM)
    """

    direction: Direction = Direction.CW
    acceleration_rpm_s: int = 1000  # u16, RPM/S
    speed_raw: int = 1000  # u16, 0.1RPM, 0-MAX_SPEED_RAW_X
    sync: SyncFlag = SyncFlag.IMMEDIATE

    def __post_init__(self):
        if not 0 <= self.acceleration_rpm_s <= 0xFFFF:
            raise ValueError("加速度必须在 0-65535 RPM/S 之间")
        if not 0 <= self.speed_raw <= MAX_SPEED_RAW_X:
            raise ValueError(
                f"速度原始值必须在 0-{MAX_SPEED_RAW_X} (0-{MAX_SPEED_RPM_X}.0 RPM) 之间"
            )

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.direction,
            *_u16(self.acceleration_rpm_s),
            *_u16(self.speed_raw),
            self.sync,
        ])


@dataclass
class XJogCurrentLimitParams:
    """速度模式限电流参数 (X固件).

    对应命令: 5.3.6 速度模式限电流控制（X）
    """

    direction: Direction = Direction.CW
    acceleration_rpm_s: int = 1000
    speed_raw: int = 1000  # 0.1RPM
    sync: SyncFlag = SyncFlag.IMMEDIATE
    max_current_ma: int = 2000

    def __post_init__(self):
        if not 0 <= self.acceleration_rpm_s <= 0xFFFF:
            raise ValueError("加速度必须在 0-65535 RPM/S 之间")
        if not 0 <= self.speed_raw <= MAX_SPEED_RAW_X:
            raise ValueError(
                f"速度原始值必须在 0-{MAX_SPEED_RAW_X} (0-{MAX_SPEED_RPM_X}.0 RPM) 之间"
            )
        if not 0 <= self.max_current_ma <= 5000:
            raise ValueError("最大电流必须在 0-5000 mA 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.direction,
            *_u16(self.acceleration_rpm_s),
            *_u16(self.speed_raw),
            self.sync,
            *_u16(self.max_current_ma),
        ])


@dataclass
class XTorqueParams:
    """力矩模式参数 (X固件).

    对应命令: 5.3.3 力矩模式控制（X）
    """

    direction: Direction = Direction.CW
    slope_ma_s: int = 200  # mA/S
    current_ma: int = 600
    sync: SyncFlag = SyncFlag.IMMEDIATE

    def __post_init__(self):
        if not 0 <= self.slope_ma_s <= 0xFFFF:
            raise ValueError("斜率必须在 0-65535 mA/S 之间")
        if not 0 <= self.current_ma <= 5000:
            raise ValueError("电流必须在 0-5000 mA 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.direction,
            *_u16(self.slope_ma_s),
            *_u16(self.current_ma),
            self.sync,
        ])


@dataclass
class XTorqueLimitedParams:
    """力矩模式限速参数 (X固件).

    对应命令: 5.3.4 力矩模式限速控制（X）
    """

    direction: Direction = Direction.CW
    slope_ma_s: int = 200
    current_ma: int = 600
    sync: SyncFlag = SyncFlag.IMMEDIATE
    max_speed_raw: int = 4000  # 0.1RPM

    def __post_init__(self):
        if not 0 <= self.slope_ma_s <= 0xFFFF:
            raise ValueError("斜率必须在 0-65535 mA/S 之间")
        if not 0 <= self.current_ma <= 5000:
            raise ValueError("电流必须在 0-5000 mA 之间")
        if not 0 <= self.max_speed_raw <= MAX_SPEED_RAW_X:
            raise ValueError(
                f"最大速度原始值必须在 0-{MAX_SPEED_RAW_X} (0-{MAX_SPEED_RPM_X}.0 RPM) 之间"
            )

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.direction,
            *_u16(self.slope_ma_s),
            *_u16(self.current_ma),
            self.sync,
            *_u16(self.max_speed_raw),
        ])


@dataclass
class XPositionDirectParams:
    """直通限速位置模式参数 (X固件).

    对应命令: 5.3.8 直通限速位置模式控制（X）
    angle_raw: 0.1° 单位 u32
    """

    direction: Direction = Direction.CW
    speed_raw: int = 1000  # 0.1RPM
    angle_raw: int = 3600  # 0.1°, 默认 360.0°
    motion_mode: MotionMode = MotionMode.RELATIVE_LAST
    sync: SyncFlag = SyncFlag.IMMEDIATE

    def __post_init__(self):
        if not 0 <= self.speed_raw <= MAX_SPEED_RAW_X:
            raise ValueError(
                f"速度原始值必须在 0-{MAX_SPEED_RAW_X} (0-{MAX_SPEED_RPM_X}.0 RPM) 之间"
            )
        if not 0 <= self.angle_raw <= 0xFFFFFFFF:
            raise ValueError("角度原始值必须在 0-4294967295 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.direction,
            *_u16(self.speed_raw),
            *_u32(self.angle_raw),
            self.motion_mode,
            self.sync,
        ])


@dataclass
class XPositionDirectLimitedParams:
    """直通限速位置模式限电流参数 (X固件).

    对应命令: 5.3.9 直通限速位置模式限电流控制（X）
    """

    direction: Direction = Direction.CW
    speed_raw: int = 1000
    angle_raw: int = 3600
    motion_mode: MotionMode = MotionMode.RELATIVE_LAST
    sync: SyncFlag = SyncFlag.IMMEDIATE
    max_current_ma: int = 2000

    def __post_init__(self):
        if not 0 <= self.speed_raw <= MAX_SPEED_RAW_X:
            raise ValueError(
                f"速度原始值必须在 0-{MAX_SPEED_RAW_X} (0-{MAX_SPEED_RPM_X}.0 RPM) 之间"
            )
        if not 0 <= self.angle_raw <= 0xFFFFFFFF:
            raise ValueError("角度原始值必须在 0-4294967295 之间")
        if not 0 <= self.max_current_ma <= 5000:
            raise ValueError("最大电流必须在 0-5000 mA 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.direction,
            *_u16(self.speed_raw),
            *_u32(self.angle_raw),
            self.motion_mode,
            self.sync,
            *_u16(self.max_current_ma),
        ])


@dataclass
class XPositionTrapParams:
    """梯形曲线加减速位置模式参数 (X固件).

    对应命令: 5.3.10 梯形曲线加减速位置模式控制（X）, 功能码 0xFD
    """

    direction: Direction = Direction.CW
    accel_rpm_s: int = 500
    decel_rpm_s: int = 500
    max_speed_raw: int = 10000  # 0.1RPM
    angle_raw: int = 3600  # 0.1°
    motion_mode: MotionMode = MotionMode.RELATIVE_LAST
    sync: SyncFlag = SyncFlag.IMMEDIATE

    def __post_init__(self):
        if not 0 <= self.accel_rpm_s <= 0xFFFF:
            raise ValueError("加速加速度必须在 0-65535 RPM/S 之间")
        if not 0 <= self.decel_rpm_s <= 0xFFFF:
            raise ValueError("减速加速度必须在 0-65535 RPM/S 之间")
        if not 0 <= self.max_speed_raw <= MAX_SPEED_RAW_X:
            raise ValueError(
                f"最大速度原始值必须在 0-{MAX_SPEED_RAW_X} (0-{MAX_SPEED_RPM_X}.0 RPM) 之间"
            )
        if not 0 <= self.angle_raw <= 0xFFFFFFFF:
            raise ValueError("角度原始值必须在 0-4294967295 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.direction,
            *_u16(self.accel_rpm_s),
            *_u16(self.decel_rpm_s),
            *_u16(self.max_speed_raw),
            *_u32(self.angle_raw),
            self.motion_mode,
            self.sync,
        ])


@dataclass
class XPositionTrapLimitedParams:
    """梯形曲线位置模式限电流参数 (X固件).

    对应命令: 5.3.11 梯形曲线加减速位置模式限电流控制（X）
    """

    direction: Direction = Direction.CW
    accel_rpm_s: int = 500
    decel_rpm_s: int = 500
    max_speed_raw: int = 10000
    angle_raw: int = 3600
    motion_mode: MotionMode = MotionMode.RELATIVE_LAST
    sync: SyncFlag = SyncFlag.IMMEDIATE
    max_current_ma: int = 2000

    def __post_init__(self):
        if not 0 <= self.accel_rpm_s <= 0xFFFF:
            raise ValueError("加速加速度必须在 0-65535 RPM/S 之间")
        if not 0 <= self.decel_rpm_s <= 0xFFFF:
            raise ValueError("减速加速度必须在 0-65535 RPM/S 之间")
        if not 0 <= self.max_speed_raw <= MAX_SPEED_RAW_X:
            raise ValueError(
                f"最大速度原始值必须在 0-{MAX_SPEED_RAW_X} (0-{MAX_SPEED_RPM_X}.0 RPM) 之间"
            )
        if not 0 <= self.angle_raw <= 0xFFFFFFFF:
            raise ValueError("角度原始值必须在 0-4294967295 之间")
        if not 0 <= self.max_current_ma <= 5000:
            raise ValueError("最大电流必须在 0-5000 mA 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.direction,
            *_u16(self.accel_rpm_s),
            *_u16(self.decel_rpm_s),
            *_u16(self.max_speed_raw),
            *_u32(self.angle_raw),
            self.motion_mode,
            self.sync,
            *_u16(self.max_current_ma),
        ])


@dataclass
class HomingParams:
    """回零参数.

    对应命令: 5.4.6 修改回零参数
    """

    homing_mode: HomingMode = HomingMode.NEAREST
    homing_direction: HomingDirection = HomingDirection.CW
    homing_speed: int = 30  # RPM
    homing_timeout: int = 10000  # ms
    collision_speed: int = 300  # RPM
    collision_current: int = 800  # mA
    collision_time: int = 60  # ms
    auto_home: bool = False

    @property
    def bytes(self) -> bytes:
        return bytes([
            self.homing_mode,
            self.homing_direction,
            *_u16(self.homing_speed),
            *_u32(self.homing_timeout),
            *_u16(self.collision_speed),
            *_u16(self.collision_current),
            *_u16(self.collision_time),
            1 if self.auto_home else 0,
        ])

    @classmethod
    def from_bytes(cls, data: bytes) -> "HomingParams":
        return cls(
            homing_mode=HomingMode(data[0]),
            homing_direction=HomingDirection(data[1]),
            homing_speed=to_int(data[2:4]),
            homing_timeout=to_int(data[4:8]),
            collision_speed=to_int(data[8:10]),
            collision_current=to_int(data[10:12]),
            collision_time=to_int(data[12:14]),
            auto_home=bool(data[14]),
        )


@dataclass
class VersionParams:
    """版本参数.

    对应命令: 5.5.2 读取固件版本和硬件版本
    """

    firmware_version: int = 0
    hw_series: int = 0  # 0=X系列, 1=Y系列
    hw_type: int = 0  # 0/1/2/3/4/5 = 20/28/35/42/57/86
    hw_version: int = 0

    @classmethod
    def from_bytes(cls, data: bytes) -> "VersionParams":
        fw_ver = to_int(data[0:2])
        hw_info = to_int(data[2:4])
        return cls(
            firmware_version=fw_ver,
            hw_series=(hw_info >> 12) & 0x0F,
            hw_type=(hw_info >> 8) & 0x0F,
            hw_version=hw_info & 0xFF,
        )

    @property
    def firmware_version_str(self) -> str:
        major = self.firmware_version // 100
        minor = (self.firmware_version % 100) // 10
        patch = self.firmware_version % 10
        return f"V{major}.{minor}.{patch}"

    @property
    def hw_series_str(self) -> str:
        return "X系列" if self.hw_series == 0 else "Y系列"

    @property
    def hw_type_str(self) -> str:
        types = {0: "20", 1: "28", 2: "35", 3: "42", 4: "57", 5: "86"}
        series = "X" if self.hw_series == 0 else "Y"
        return f"{series}{types.get(self.hw_type, 'Unknown')}"


@dataclass
class MotorRHParams:
    """电机相电阻和相电感参数.

    对应命令: 5.5.3 读取相电阻和相电感
    """

    phase_resistance: int = 0  # mΩ
    phase_inductance: int = 0  # uH

    @classmethod
    def from_bytes(cls, data: bytes) -> "MotorRHParams":
        return cls(
            phase_resistance=to_int(data[0:2]),
            phase_inductance=to_int(data[2:4]),
        )


@dataclass
class EmmPIDParams:
    """PID参数 (Emm固件).

    对应命令: 5.6.16/5.6.17 读取/修改 PID 参数（Emm）
    """

    kp: int = 18000
    ki: int = 10
    kd: int = 18000

    @property
    def bytes(self) -> bytes:
        return _u32(self.kp) + _u32(self.ki) + _u32(self.kd)

    @classmethod
    def from_bytes(cls, data: bytes) -> "EmmPIDParams":
        return cls(
            kp=to_int(data[0:4]),
            ki=to_int(data[4:8]),
            kd=to_int(data[8:12]),
        )


@dataclass
class XPIDParams:
    """PID参数 (X固件).

    对应命令: 5.6.14/5.6.15 读取/修改 PID 参数（X）
    布局为 4×u32:
      pos_kp1 = 梯形曲线位置环 Kp (pTkp)
      pos_kp2 = 直通限速位置环 Kp (pBkp)
      vel_kp  = 速度环 Kp (vkp)
      vel_ki  = 速度环 Ki (vki)
    X42S 默认: pos_kp1/pos_kp2=126640, vel_kp=15600, vel_ki=26
    """

    pos_kp1: int = 126640  # 梯形曲线位置环 Kp
    pos_kp2: int = 126640  # 直通限速位置环 Kp
    vel_kp: int = 15600
    vel_ki: int = 26

    @property
    def bytes(self) -> bytes:
        return (
            _u32(self.pos_kp1)
            + _u32(self.pos_kp2)
            + _u32(self.vel_kp)
            + _u32(self.vel_ki)
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "XPIDParams":
        return cls(
            pos_kp1=to_int(data[0:4]),
            pos_kp2=to_int(data[4:8]),
            vel_kp=to_int(data[8:12]),
            vel_ki=to_int(data[12:16]),
        )


@dataclass
class HomingStatus:
    """回零状态标志.

    对应命令: 5.4.4 读取回零状态标志
    """

    encoder_ready: bool = False
    calibrated: bool = False
    is_homing: bool = False
    homing_failed: bool = False
    over_temp: bool = False
    over_current: bool = False

    @classmethod
    def from_byte(cls, data: int) -> "HomingStatus":
        return cls(
            encoder_ready=bool(data & 0x01),
            calibrated=bool(data & 0x02),
            is_homing=bool(data & 0x04),
            homing_failed=bool(data & 0x08),
            over_temp=bool(data & 0x10),
            over_current=bool(data & 0x20),
        )

    @property
    def homing_state(self) -> str:
        if self.is_homing:
            return "正在回零"
        elif self.homing_failed:
            return "回零失败"
        else:
            return "回零成功/未回零"


@dataclass
class MotorStatus:
    """电机状态标志.

    对应命令: 5.5.15 读取电机状态标志
    """

    enabled: bool = False
    position_reached: bool = False
    stall_detected: bool = False
    stall_protected: bool = False
    left_limit: bool = False
    right_limit: bool = False
    power_off_flag: bool = True

    @classmethod
    def from_byte(cls, data: int) -> "MotorStatus":
        return cls(
            enabled=bool(data & 0x01),
            position_reached=bool(data & 0x02),
            stall_detected=bool(data & 0x04),
            stall_protected=bool(data & 0x08),
            left_limit=bool(data & 0x10),
            right_limit=bool(data & 0x20),
            power_off_flag=bool(data & 0x80),
        )


@dataclass
class IOStatus:
    """引脚 IO 电平状态.

    对应命令: 5.5.17 读取引脚IO电平状态
    """

    en_pin: bool = False
    step_pin: bool = False
    dir_pin: bool = False
    dir_output_mode: bool = False

    @classmethod
    def from_byte(cls, data: int) -> "IOStatus":
        return cls(
            en_pin=bool(data & 0x01),
            step_pin=bool(data & 0x04),
            dir_pin=bool(data & 0x10),
            dir_output_mode=bool(data & 0x20),
        )


@dataclass
class HomeMotorStatus:
    """回零状态 + 电机状态.

    对应命令: 5.5.16 读取回零状态标志+电机状态标志
    """

    homing: HomingStatus = field(default_factory=HomingStatus)
    motor: MotorStatus = field(default_factory=MotorStatus)

    @classmethod
    def from_bytes(cls, data: bytes) -> "HomeMotorStatus":
        return cls(
            homing=HomingStatus.from_byte(data[0]),
            motor=MotorStatus.from_byte(data[1]),
        )


@dataclass
class OptionStatus:
    """选项参数状态.

    对应命令: 5.6.4 读取选项参数状态

    警告: X42S V2.0.x 上 ``firmware_is_emm`` / ``firmware_type`` (FwType 位)
    不可靠，Emm 固件时该位仍可能为 0。判断当前算法请用
    ``X42SDevice.detect_firmware()``（读配置块长度）。
    """

    motor_is_09_degree: bool = False
    firmware_is_emm: bool = True
    closed_loop: bool = True
    direction_ccw: bool = False
    button_locked: bool = False
    scale_input: bool = False
    lock_param_level: int = 0
    raw: int = 0

    @classmethod
    def from_byte(cls, data: int) -> "OptionStatus":
        return cls.from_bytes(bytes([data & 0xFF]))

    @classmethod
    def from_bytes(cls, data: bytes) -> "OptionStatus":
        b0 = data[0] if data else 0
        b1 = data[1] if len(data) > 1 else 0
        return cls(
            motor_is_09_degree=bool(b0 & 0x01),
            firmware_is_emm=bool(b0 & 0x02),
            closed_loop=bool(b0 & 0x04),
            direction_ccw=bool(b0 & 0x10),
            button_locked=bool(b0 & 0x20),
            scale_input=bool(b0 & 0x80),
            lock_param_level=b1 & 0x03,
            raw=b0 | (b1 << 8),
        )

    @property
    def firmware_type(self) -> FirmwareType:
        """来自 0x1A FwType 位；X42S 上可能不可信，见类文档."""
        return FirmwareType.EMM_FIRMWARE if self.firmware_is_emm else FirmwareType.X_FIRMWARE


@dataclass
class ProtectionThreshold:
    """过热过流保护检测阈值.

    对应命令: 5.6.22 读取过热过流保护检测阈值
    """

    over_temp_threshold: int = 100  # °C
    over_current_threshold: int = 6600  # mA
    detection_time: int = 1000  # ms

    @property
    def bytes(self) -> bytes:
        return _u16(self.over_temp_threshold) + _u16(self.over_current_threshold) + _u16(
            self.detection_time
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> "ProtectionThreshold":
        return cls(
            over_temp_threshold=to_int(data[0:2]),
            over_current_threshold=to_int(data[2:4]),
            detection_time=to_int(data[4:6]),
        )


@dataclass
class DMX512Params:
    """DMX512 协议参数.

    对应命令: 5.6.18/5.6.19 读取/修改DMX512协议参数
    """

    total_channels: int = 192
    channels_per_motor: int = 1
    absolute_mode: bool = True
    single_channel_speed: int = 1000  # RPM
    acceleration: int = 1000
    dual_speed_step: int = 10  # RPM
    dual_motion_step: int = 100  # ×0.1°

    @property
    def dual_motion_step_deg(self) -> float:
        return self.dual_motion_step * 0.1

    @property
    def bytes(self) -> bytes:
        return bytes([
            *_u16(self.total_channels),
            self.channels_per_motor,
            1 if self.absolute_mode else 0,
            *_u16(self.single_channel_speed),
            *_u16(self.acceleration),
            *_u16(self.dual_speed_step),
            *_u32(self.dual_motion_step),
        ])

    @classmethod
    def from_bytes(cls, data: bytes) -> "DMX512Params":
        return cls(
            total_channels=to_int(data[0:2]),
            channels_per_motor=data[2],
            absolute_mode=bool(data[3]),
            single_channel_speed=to_int(data[4:6]),
            acceleration=to_int(data[6:8]),
            dual_speed_step=to_int(data[8:10]),
            dual_motion_step=to_int(data[10:14]),
        )


@dataclass
class EmmSystemStatusParams:
    """系统状态参数 (Emm固件).

    对应命令: 5.8.2 读取系统状态参数（Emm）
    from_bytes 接收去掉地址/功能码/字节数/参数个数后的数据区。
    """

    bus_voltage: int = 0  # mV
    phase_current: int = 0  # mA
    encoder_value: int = 0
    target_position: int = 0
    target_position_sign: int = 0
    realtime_speed: int = 0  # RPM
    realtime_speed_sign: int = 0
    realtime_position: int = 0
    realtime_position_sign: int = 0
    position_error: int = 0
    position_error_sign: int = 0
    homing_status: HomingStatus = field(default_factory=HomingStatus)
    motor_status: MotorStatus = field(default_factory=MotorStatus)

    @classmethod
    def from_bytes(cls, data: bytes) -> "EmmSystemStatusParams":
        return cls(
            bus_voltage=to_int(data[0:2]),
            phase_current=to_int(data[2:4]),
            encoder_value=to_int(data[4:6]),
            target_position_sign=data[6],
            target_position=to_int(data[7:11]),
            realtime_speed_sign=data[11],
            realtime_speed=to_int(data[12:14]),
            realtime_position_sign=data[14],
            realtime_position=to_int(data[15:19]),
            position_error_sign=data[19],
            position_error=to_int(data[20:24]),
            homing_status=HomingStatus.from_byte(data[24]),
            motor_status=MotorStatus.from_byte(data[25]),
        )

    @property
    def target_position_deg(self) -> float:
        return signed_from_sign_byte(self.target_position_sign, 1) * emm_raw_to_deg(
            self.target_position
        )

    @property
    def realtime_position_deg(self) -> float:
        return signed_from_sign_byte(self.realtime_position_sign, 1) * emm_raw_to_deg(
            self.realtime_position
        )

    @property
    def position_error_deg(self) -> float:
        return signed_from_sign_byte(self.position_error_sign, 1) * emm_raw_to_deg(
            self.position_error
        )

    @property
    def realtime_speed_rpm(self) -> int:
        return signed_from_sign_byte(self.realtime_speed_sign, self.realtime_speed)


@dataclass
class XSystemStatusParams:
    """系统状态参数 (X固件).

    对应命令: 5.8.1 读取系统状态参数（X）
    整帧 37 字节 / 12 参数；from_bytes 接收去掉地址/功能码/字节数/参数个数后的数据区。

    布局:
      bus_voltage u16, bus_current u16, phase_current u16,
      encoder_raw u16, encoder_value u16,
      sign + target_position u32 (0.1°),
      sign + realtime_speed u16 (0.1RPM),
      sign + realtime_position u32 (0.1°),
      sign + position_error u32 (0.01°),
      sign + temperature u8 (°C),
      homing_status u8, motor_status u8
    """

    bus_voltage: int = 0  # mV
    bus_current: int = 0  # mA
    phase_current: int = 0  # mA
    encoder_raw: int = 0
    encoder_value: int = 0
    target_position: int = 0  # 0.1°
    target_position_sign: int = 0
    realtime_speed: int = 0  # 0.1RPM
    realtime_speed_sign: int = 0
    realtime_position: int = 0  # 0.1°
    realtime_position_sign: int = 0
    position_error: int = 0  # 0.01°
    position_error_sign: int = 0
    temperature: int = 0  # °C
    temperature_sign: int = 0
    homing_status: HomingStatus = field(default_factory=HomingStatus)
    motor_status: MotorStatus = field(default_factory=MotorStatus)

    @classmethod
    def from_bytes(cls, data: bytes) -> "XSystemStatusParams":
        return cls(
            bus_voltage=to_int(data[0:2]),
            bus_current=to_int(data[2:4]),
            phase_current=to_int(data[4:6]),
            encoder_raw=to_int(data[6:8]),
            encoder_value=to_int(data[8:10]),
            target_position_sign=data[10],
            target_position=to_int(data[11:15]),
            realtime_speed_sign=data[15],
            realtime_speed=to_int(data[16:18]),
            realtime_position_sign=data[18],
            realtime_position=to_int(data[19:23]),
            position_error_sign=data[23],
            position_error=to_int(data[24:28]),
            temperature_sign=data[28],
            temperature=data[29],
            homing_status=HomingStatus.from_byte(data[30]),
            motor_status=MotorStatus.from_byte(data[31]),
        )

    @property
    def target_position_deg(self) -> float:
        return signed_from_sign_byte(self.target_position_sign, 1) * x_raw_to_deg(
            self.target_position
        )

    @property
    def realtime_position_deg(self) -> float:
        return signed_from_sign_byte(self.realtime_position_sign, 1) * x_raw_to_deg(
            self.realtime_position
        )

    @property
    def position_error_deg(self) -> float:
        return signed_from_sign_byte(self.position_error_sign, 1) * x_error_raw_to_deg(
            self.position_error
        )

    @property
    def realtime_speed_rpm(self) -> float:
        return signed_from_sign_byte(self.realtime_speed_sign, 1) * x_speed_raw_to_rpm(
            self.realtime_speed
        )

    @property
    def temperature_c(self) -> int:
        """驱动温度 (°C).

        X42S 实机温度符号与手册相反: 00=正, 01=负。
        """
        return self.temperature if self.temperature_sign == 0 else -self.temperature


@dataclass
class EmmConfigParams:
    """驱动配置参数 (Emm固件).

    对应命令: 5.8.5/5.8.6 读取/修改驱动配置参数（Emm）
    """

    motor_type: MotorType = MotorType.DEGREE_18
    pulse_port_mode: PulsePortMode = PulsePortMode.FOC
    serial_port_mode: SerialPortMode = SerialPortMode.UART
    enable_level: EnableLevel = EnableLevel.HOLD
    dir_level: DirLevel = DirLevel.CW
    microstep: int = 16
    microstep_interp: bool = True
    open_loop_current: int = 1200  # mA
    closed_loop_current: int = 3000  # mA
    max_voltage: int = 4000  # 实际 mV = value * 3
    baud_rate: BaudRate = BaudRate.BAUD_115200
    can_rate: CanRate = CanRate.CAN_500K
    motor_id: int = 1  # 仅读取有效; SET 时该位保留须写 0
    checksum_mode: ChecksumMode = ChecksumMode.FIXED
    response_mode: ResponseMode = ResponseMode.RECEIVE
    stall_protect: StallProtect = StallProtect.ENABLE
    stall_speed: int = 8  # RPM
    stall_current: int = 2200  # mA
    stall_time: int = 2000  # ms
    position_window: int = 8  # *0.1度

    @property
    def bytes(self) -> bytes:
        """返回修改配置数据区 (不含存储标志).

        写配置时原 ID 位置为保留字节，必须为 0。
        """
        return bytes([
            self.motor_type,
            self.pulse_port_mode,
            self.serial_port_mode,
            self.enable_level,
            self.dir_level,
            self.microstep,
            1 if self.microstep_interp else 0,
            0,  # 保留
            *_u16(self.open_loop_current),
            *_u16(self.closed_loop_current),
            *_u16(self.max_voltage),
            self.baud_rate,
            self.can_rate,
            0,  # 保留 (读时为 ID)
            self.checksum_mode,
            self.response_mode,
            self.stall_protect,
            *_u16(self.stall_speed),
            *_u16(self.stall_current),
            *_u16(self.stall_time),
            *_u16(self.position_window),
        ])

    @classmethod
    def from_bytes(cls, data: bytes) -> "EmmConfigParams":
        return cls(
            motor_type=MotorType(data[0]),
            pulse_port_mode=PulsePortMode(data[1]),
            serial_port_mode=SerialPortMode(data[2]),
            enable_level=EnableLevel(data[3]),
            dir_level=DirLevel(data[4]),
            microstep=data[5],
            microstep_interp=bool(data[6]),
            open_loop_current=to_int(data[8:10]),
            closed_loop_current=to_int(data[10:12]),
            max_voltage=to_int(data[12:14]),
            baud_rate=BaudRate(data[14]),
            can_rate=CanRate(data[15]),
            motor_id=data[16],
            checksum_mode=ChecksumMode(data[17]),
            response_mode=ResponseMode(data[18]),
            stall_protect=StallProtect(data[19]),
            stall_speed=to_int(data[20:22]),
            stall_current=to_int(data[22:24]),
            stall_time=to_int(data[24:26]),
            position_window=to_int(data[26:28]),
        )

    @property
    def position_window_deg(self) -> float:
        return self.position_window * 0.1

    @property
    def max_voltage_mv(self) -> int:
        return self.max_voltage * 3


@dataclass
class XConfigParams:
    """驱动配置参数 (X固件).

    对应命令: 5.8.3/5.8.4 读取/修改驱动配置参数（X）
    from_bytes / .bytes 均不含地址、功能码、辅助码与存储标志。
    """

    lock_button: bool = False
    control_mode: ControlMode = ControlMode.CLOSED_LOOP
    pulse_port: PulsePortMode = PulsePortMode.OPEN  # 示例默认 PUL_ENA
    serial_port: SerialPortMode = SerialPortMode.UART
    enable_level: EnableLevel = EnableLevel.HOLD
    dir_level: DirLevel = DirLevel.CW
    microstep: int = 16
    microstep_interp: bool = True
    reserved0: int = 0
    reserved1: int = 0
    open_loop_current: int = 1200  # mA
    closed_loop_current: int = 3000  # mA
    max_speed_rpm: int = 3000  # RPM (整数, 非 0.1)
    current_loop_bandwidth: int = 1000  # Hz
    baud: BaudRate = BaudRate.BAUD_115200
    can: CanRate = CanRate.CAN_500K
    checksum: ChecksumMode = ChecksumMode.FIXED
    response: ResponseMode = ResponseMode.RECEIVE
    scale_x10: bool = False  # 角度缩小10倍输入 → 0.01°
    stall_protect: StallProtect = StallProtect.ENABLE
    stall_speed: int = 8  # RPM
    stall_current: int = 2200  # mA
    stall_time: int = 2000  # ms
    position_window: int = 8  # *0.1°

    @property
    def bytes(self) -> bytes:
        """返回修改配置数据区 (不含存储标志)."""
        return bytes([
            1 if self.lock_button else 0,
            self.control_mode,
            self.pulse_port,
            self.serial_port,
            self.enable_level,
            self.dir_level,
            self.microstep,
            1 if self.microstep_interp else 0,
            self.reserved0,
            self.reserved1,
            *_u16(self.open_loop_current),
            *_u16(self.closed_loop_current),
            *_u16(self.max_speed_rpm),
            *_u16(self.current_loop_bandwidth),
            self.baud,
            self.can,
            self.checksum,
            self.response,
            1 if self.scale_x10 else 0,
            self.stall_protect,
            *_u16(self.stall_speed),
            *_u16(self.stall_current),
            *_u16(self.stall_time),
            *_u16(self.position_window),
        ])

    @classmethod
    def from_bytes(cls, data: bytes) -> "XConfigParams":
        return cls(
            lock_button=bool(data[0]),
            control_mode=ControlMode(data[1]),
            pulse_port=PulsePortMode(data[2]),
            serial_port=SerialPortMode(data[3]),
            enable_level=EnableLevel(data[4]),
            dir_level=DirLevel(data[5]),
            microstep=data[6],
            microstep_interp=bool(data[7]),
            reserved0=data[8],
            reserved1=data[9],
            open_loop_current=to_int(data[10:12]),
            closed_loop_current=to_int(data[12:14]),
            max_speed_rpm=to_int(data[14:16]),
            current_loop_bandwidth=to_int(data[16:18]),
            baud=BaudRate(data[18]),
            can=CanRate(data[19]),
            checksum=ChecksumMode(data[20]),
            response=ResponseMode(data[21]),
            scale_x10=bool(data[22]),
            stall_protect=StallProtect(data[23]),
            stall_speed=to_int(data[24:26]),
            stall_current=to_int(data[26:28]),
            stall_time=to_int(data[28:30]),
            position_window=to_int(data[30:32]),
        )

    @property
    def position_window_deg(self) -> float:
        return self.position_window * 0.1


@dataclass
class EmmAutoRunParams:
    """上电自动运行参数 (Emm固件).

    对应命令: 5.7.2 存储一组速度参数，上电自动运行（Emm）
    .bytes 不含辅助码 0x1C。
    """

    store: bool = True
    direction: Direction = Direction.CW
    speed: int = 600  # RPM
    acceleration: int = 100  # 档位
    enable_en_control: bool = False

    def __post_init__(self):
        if not 0 <= self.speed <= MAX_SPEED_RPM:
            raise ValueError(f"速度必须在 0-{MAX_SPEED_RPM} RPM 之间")
        if not 0 <= self.acceleration <= 255:
            raise ValueError("加速度必须在 0-255 之间")

    @property
    def bytes(self) -> bytes:
        return bytes([
            1 if self.store else 0,
            self.direction,
            *_u16(self.speed),
            self.acceleration,
            1 if self.enable_en_control else 0,
        ])


@dataclass
class XAutoRunParams:
    """上电自动运行参数 (X固件).

    对应命令: 5.7.1 存储一组速度参数，上电自动运行（X）
    speed_raw: 0.1RPM 格式 (与 X 速度模式一致)。
    .bytes 不含辅助码 0x1C。
    """

    store: bool = True
    direction: Direction = Direction.CW
    acceleration_rpm_s: int = 511
    speed_raw: int = 6000  # 0.1RPM → 600.0RPM
    enable_en_control: bool = False

    def __post_init__(self):
        if not 0 <= self.acceleration_rpm_s <= 0xFFFF:
            raise ValueError("加速度必须在 0-65535 RPM/S 之间")
        if not 0 <= self.speed_raw <= MAX_SPEED_RAW_X:
            raise ValueError(
                f"速度原始值必须在 0-{MAX_SPEED_RAW_X} (0-{MAX_SPEED_RPM_X}.0 RPM) 之间"
            )

    @property
    def bytes(self) -> bytes:
        return bytes([
            1 if self.store else 0,
            self.direction,
            *_u16(self.acceleration_rpm_s),
            *_u16(self.speed_raw),
            1 if self.enable_en_control else 0,
        ])
