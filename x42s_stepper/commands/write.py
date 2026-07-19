"""X42S 设置/写入命令."""

from ..configs import (
    Code,
    ControlMode,
    Direction,
    FirmwareType,
    LockParamLevel,
    MotorType,
    Protocol,
    StoreFlag,
)
from ..parameters import (
    DMX512Params,
    DeviceParams,
    EmmAutoRunParams,
    EmmConfigParams,
    EmmPIDParams,
    ProtectionThreshold,
    XAutoRunParams,
    XConfigParams,
    XPIDParams,
)
from .base import SimpleCommand


class SetID(SimpleCommand):
    """修改电机ID/地址.

    对应命令: 5.6.1 修改电机ID/地址
    发送: 01 AE 4B 01 02 6B
    返回: 01 AE 02 6B
    """

    _code = Code.SET_ID
    _protocol = Protocol.SET_ID

    def __init__(self, device: DeviceParams, new_id: int, store: bool = True):
        if not 1 <= new_id <= 255:
            raise ValueError("ID必须在 1-255 之间")
        self.new_id = new_id
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            self.new_id,
        ])


class SetMicrostep(SimpleCommand):
    """修改细分值.

    对应命令: 5.6.2 修改细分值
    发送: 01 84 8A 01 10 6B
    返回: 01 84 02 6B
    """

    _code = Code.SET_MICROSTEP
    _protocol = Protocol.SET_MICROSTEP

    def __init__(self, device: DeviceParams, microstep: int, store: bool = True):
        if not 1 <= microstep <= 256:
            raise ValueError("细分值必须在 1-256 之间")
        self.microstep = microstep if microstep < 256 else 0  # 256用0表示
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            self.microstep,
        ])


class SetLoopMode(SimpleCommand):
    """修改开环/闭环控制模式.

    对应命令: 5.6.7 修改开环/闭环控制模式
    发送: 01 46 69 01 01 6B
    返回: 01 46 02 6B
    """

    _code = Code.SET_LOOP_MODE
    _protocol = Protocol.SET_LOOP_MODE

    def __init__(
        self,
        device: DeviceParams,
        closed_loop: bool = True,
        store: bool = True,
    ):
        self.closed_loop = closed_loop
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            ControlMode.CLOSED_LOOP if self.closed_loop else ControlMode.OPEN_LOOP,
        ])


class SetOpenLoopCurrent(SimpleCommand):
    """修改开环模式工作电流.

    对应命令: 5.6.12 修改开环模式工作电流
    发送: 01 44 33 01 04B0 6B
    返回: 01 44 02 6B
    """

    _code = Code.SET_OPEN_LOOP_CURRENT
    _protocol = Protocol.SET_OPEN_LOOP_CURRENT

    def __init__(self, device: DeviceParams, current_ma: int, store: bool = True):
        if not 0 <= current_ma <= 5000:
            raise ValueError("电流必须在 0-5000 mA 之间")
        self.current = current_ma
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            (self.current >> 8) & 0xFF,
            self.current & 0xFF,
        ])


class SetClosedLoopCurrent(SimpleCommand):
    """修改闭环模式最大电流.

    对应命令: 5.6.13 修改闭环模式最大电流
    发送: 01 45 66 01 0BB8 6B
    返回: 01 45 02 6B
    """

    _code = Code.SET_CLOSED_LOOP_CURRENT
    _protocol = Protocol.SET_CLOSED_LOOP_CURRENT

    def __init__(self, device: DeviceParams, current_ma: int, store: bool = True):
        if not 0 <= current_ma <= 5000:
            raise ValueError("电流必须在 0-5000 mA 之间")
        self.current = current_ma
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            (self.current >> 8) & 0xFF,
            self.current & 0xFF,
        ])


class SetEmmPID(SimpleCommand):
    """修改PID参数 (Emm固件).

    对应命令: 5.6.17 修改PID参数（Emm）
    """

    _code = Code.SET_PID
    _protocol = Protocol.SET_PID

    def __init__(self, device: DeviceParams, params: EmmPIDParams, store: bool = True):
        self.params = params
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
        ]) + self.params.bytes


class SetXPID(SimpleCommand):
    """修改PID参数 (X固件).

    对应命令: 5.6.15 修改PID参数（X）
    """

    _code = Code.SET_PID
    _protocol = Protocol.SET_PID

    def __init__(self, device: DeviceParams, params: XPIDParams, store: bool = True):
        self.params = params
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
        ]) + self.params.bytes


class SetMotorDirection(SimpleCommand):
    """修改电机运动正方向.

    对应命令: 5.6.8 修改电机运动正方向
    发送: 01 D4 60 01 00 6B
    返回: 01 D4 02 6B
    """

    _code = Code.SET_MOTOR_DIRECTION
    _protocol = Protocol.SET_MOTOR_DIRECTION

    def __init__(
        self,
        device: DeviceParams,
        direction: Direction = Direction.CW,
        store: bool = True,
    ):
        self.direction = direction
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            self.direction,
        ])


class SetFirmwareType(SimpleCommand):
    """修改固件类型.

    对应命令: 5.6.6 修改固件类型
    发送: 01 D5 69 01 01 6B
    返回: 01 D5 02 6B

    建议在电机停止时修改。
    """

    _code = Code.SET_FIRMWARE_TYPE
    _protocol = Protocol.SET_FIRMWARE_TYPE

    def __init__(
        self,
        device: DeviceParams,
        firmware_type: FirmwareType = FirmwareType.EMM_FIRMWARE,
        store: bool = True,
    ):
        self.firmware_type = firmware_type
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            int(self.firmware_type),
        ])


class SetScaleInput(SimpleCommand):
    """修改命令速度/角度值是否缩小10倍输入.

    对应命令: 5.6.11 修改命令速度值是否缩小10倍输入
    发送: 01 4F 71 01 01 6B
    返回: 01 4F 02 6B
    """

    _code = Code.SET_SCALE_INPUT
    _protocol = Protocol.SET_SCALE_INPUT

    def __init__(self, device: DeviceParams, enable: bool = False, store: bool = True):
        self.enable = enable
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            1 if self.enable else 0,
        ])


class SetLockButton(SimpleCommand):
    """修改锁定按键功能.

    对应命令: 5.6.9 修改锁定按键功能
    发送: 01 D0 B3 01 01 6B
    返回: 01 D0 02 6B
    """

    _code = Code.SET_LOCK_BUTTON
    _protocol = Protocol.SET_LOCK_BUTTON

    def __init__(self, device: DeviceParams, lock: bool = False, store: bool = True):
        self.lock = lock
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            1 if self.lock else 0,
        ])


class SetPowerOffFlag(SimpleCommand):
    """修改掉电标志.

    对应命令: 5.6.3 修改掉电标志
    发送: 01 50 00 6B
    返回: 01 50 02 6B
    """

    _code = Code.SET_POWER_OFF_FLAG

    def __init__(self, device: DeviceParams, flag: bool = False):
        self.flag = flag
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([self.address, self._code, 1 if self.flag else 0])


class SetMotorType(SimpleCommand):
    """修改电机类型.

    对应命令: 5.6.5 修改电机类型
    发送: 01 D7 35 01 19 6B (1.8°)
    返回: 01 D7 02 6B

    实机确认: 0x19=1.8°, 0x32=0.9°。修改后需重新空载校准。
    """

    _code = Code.SET_MOTOR_TYPE
    _protocol = Protocol.SET_MOTOR_TYPE

    def __init__(
        self,
        device: DeviceParams,
        motor_type: MotorType = MotorType.DEGREE_18,
        store: bool = True,
    ):
        self.motor_type = motor_type
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            int(self.motor_type),
        ])


class SetProtectionThreshold(SimpleCommand):
    """修改过热过流保护检测阈值.

    对应命令: 5.6.23 修改过热过流保护检测阈值（X42S/X42S）
    """

    _code = Code.SET_PROTECTION_THRESHOLD
    _protocol = Protocol.SET_PROTECTION_THRESHOLD

    def __init__(
        self,
        device: DeviceParams,
        params: ProtectionThreshold,
        store: bool = True,
    ):
        self.params = params
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
        ]) + self.params.bytes


class SetHeartbeatTime(SimpleCommand):
    """修改心跳保护功能时间.

    对应命令: 5.6.25 修改心跳保护功能时间（X42S/X42S）
    发送: 01 68 38 01 00001388 6B
    返回: 01 68 02 6B
    """

    _code = Code.SET_HEARTBEAT_TIME
    _protocol = Protocol.SET_HEARTBEAT_TIME

    def __init__(self, device: DeviceParams, time_ms: int = 0, store: bool = True):
        self.time_ms = time_ms
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            (self.time_ms >> 24) & 0xFF,
            (self.time_ms >> 16) & 0xFF,
            (self.time_ms >> 8) & 0xFF,
            self.time_ms & 0xFF,
        ])


class SetIntegralStiffness(SimpleCommand):
    """修改积分限幅/刚性系数.

    对应命令: 5.6.27 修改积分限幅/刚性系数（X42S/X42S）
    """

    _code = Code.SET_INTEGRAL_STIFFNESS
    _protocol = Protocol.SET_INTEGRAL_STIFFNESS

    def __init__(self, device: DeviceParams, value: int = 65535, store: bool = True):
        self.value = value & 0xFFFFFFFF
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            (self.value >> 24) & 0xFF,
            (self.value >> 16) & 0xFF,
            (self.value >> 8) & 0xFF,
            self.value & 0xFF,
        ])


class SetCollisionReturnAngle(SimpleCommand):
    """修改碰撞回零返回角度.

    对应命令: 5.6.29 修改碰撞回零返回角度（X42S/X42S）
    值为0表示基于电流检测返回；其余为固定角度(0.1°单位)。
    """

    _code = Code.SET_COLLISION_RETURN_ANGLE
    _protocol = Protocol.SET_COLLISION_RETURN_ANGLE

    def __init__(
        self,
        device: DeviceParams,
        angle_deg: float = 0.0,
        store: bool = True,
    ):
        self.angle_raw = int(round(angle_deg * 10)) & 0xFFFF
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            (self.angle_raw >> 8) & 0xFF,
            self.angle_raw & 0xFF,
        ])


class SetLockParam(SimpleCommand):
    """修改锁定修改参数功能.

    对应命令: 5.6.31 修改锁定修改参数功能（X42S/X42S）
    """

    _code = Code.SET_LOCK_PARAM
    _protocol = Protocol.SET_LOCK_PARAM

    def __init__(
        self,
        device: DeviceParams,
        level: LockParamLevel = LockParamLevel.UNLOCKED,
        store: bool = True,
    ):
        self.level = level
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            int(self.level),
        ])


class SetDMX512Params(SimpleCommand):
    """修改 DMX512 协议参数.

    对应命令: 5.6.19 修改DMX512协议参数（X42S/X42S）
    """

    _code = Code.SET_DMX512_PARAM
    _protocol = Protocol.SET_DMX512_PARAM

    def __init__(self, device: DeviceParams, params: DMX512Params, store: bool = True):
        self.params = params
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
        ]) + self.params.bytes


class SetPositionWindow(SimpleCommand):
    """修改位置到达窗口 (独立命令 D1 07).

    对应命令: 5.6.21 修改位置到达窗口（X42S/X42S）
    发送: 01 D1 07 01 0008 6B
    返回: 01 D1 02 6B
    """

    _code = Code.SET_POSITION_WINDOW
    _protocol = Protocol.SET_POSITION_WINDOW

    def __init__(
        self,
        device: DeviceParams,
        window_deg: float = 0.8,
        store: bool = True,
    ):
        self.window = int(window_deg * 10)  # 内部缩小10倍处理
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
            (self.window >> 8) & 0xFF,
            self.window & 0xFF,
        ])


class SetEmmConfig(SimpleCommand):
    """修改驱动配置参数 (Emm固件).

    对应命令: 5.8.6 修改驱动配置参数（Emm）
    帧: addr + 0x48 + 0xD1 + store + params.bytes
    """

    _code = Code.SET_CONFIG
    _protocol = Protocol.SET_CONFIG

    def __init__(
        self,
        device: DeviceParams,
        params: EmmConfigParams,
        store: bool = True,
    ):
        self.params = params
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
        ]) + self.params.bytes


class SetXConfig(SimpleCommand):
    """修改驱动配置参数 (X固件).

    对应命令: 5.8.4 修改驱动配置参数（X）
    帧: addr + 0x48 + 0xD1 + store + params.bytes
    """

    _code = Code.SET_CONFIG
    _protocol = Protocol.SET_CONFIG

    def __init__(
        self,
        device: DeviceParams,
        params: XConfigParams,
        store: bool = True,
    ):
        self.params = params
        self.store = store
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
            StoreFlag.STORE if self.store else StoreFlag.NO_STORE,
        ]) + self.params.bytes


class SetEmmAutoRun(SimpleCommand):
    """存储一组速度参数，上电自动运行 (Emm固件).

    对应命令: 5.7.2 存储一组速度参数，上电自动运行（Emm）
    """

    _code = Code.SET_AUTO_RUN
    _protocol = Protocol.SET_AUTO_RUN

    def __init__(self, device: DeviceParams, params: EmmAutoRunParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
        ]) + self.params.bytes


class SetXAutoRun(SimpleCommand):
    """存储一组速度参数，上电自动运行 (X固件).

    对应命令: 5.7.1 存储一组速度参数，上电自动运行（X）
    """

    _code = Code.SET_AUTO_RUN
    _protocol = Protocol.SET_AUTO_RUN

    def __init__(self, device: DeviceParams, params: XAutoRunParams):
        self.params = params
        super().__init__(device)

    def _build_command_body(self) -> bytes:
        return bytes([
            self.address,
            self._code,
            self._protocol,
        ]) + self.params.bytes
