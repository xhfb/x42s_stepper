"""X42S 双固件步进电机设备类.

基于 ZDT_X42S 用户手册 V1.0.5，统一 X / Emm / EmmTurbo 固件的用户侧 API。
"""

from __future__ import annotations

import logging
import time
from typing import List, Optional, Tuple, Union

from serial import Serial

from .transport.base import Transport, TransportError
from .transport.serial import SerialTransport
from .commands import (
    BroadcastGetID,
    CalibrateEncoder,
    ClearProtection,
    Disable,
    EmmConfigureFastPosition,
    EmmFastPositionPulse,
    EmmJog,
    EmmPosition,
    Enable,
    EStop,
    FactoryReset,
    FirmwareCapabilityError,
    GetBatteryVoltage,
    GetBusCurrent,
    GetBusVoltage,
    GetCollisionReturnAngle,
    GetDMX512Params,
    GetConfig,
    GetSystemStatus,
    ProbeConfigFirmware,
    GetEncoder,
    GetHeartbeatTime,
    GetHomeMotorStatus,
    GetHomingParams,
    GetHomingStatus,
    GetIntegralStiffness,
    GetIOStatus,
    GetMotorRH,
    GetMotorStatus,
    GetOptionStatus,
    GetPhaseCurrent,
    GetPID,
    GetPositionError,
    GetPositionWindow,
    GetProtectionThreshold,
    GetPulseCount,
    GetRealtimePosition,
    GetRealtimeSpeed,
    GetRealtimeTarget,
    GetTargetPosition,
    GetTemperature,
    GetVersion,
    Home,
    MultiMotor,
    Restart,
    SetClosedLoopCurrent,
    SetCollisionReturnAngle,
    SetDMX512Params,
    SetEmmAutoRun,
    SetEmmConfig,
    SetEmmPID,
    SetFirmwareType,
    SetHeartbeatTime,
    SetHomeZero,
    SetHomingParams,
    SetID,
    SetIntegralStiffness,
    SetLockButton,
    SetLockParam,
    SetLoopMode,
    SetMicrostep,
    SetMotorDirection,
    SetMotorType,
    SetOpenLoopCurrent,
    SetPositionWindow,
    SetPowerOffFlag,
    SetProtectionThreshold,
    SetScaleInput,
    SetXAutoRun,
    SetXConfig,
    SetXPID,
    StopHome,
    SyncMove,
    TimedReturn,
    XConfigureFastPosition,
    XFastPositionAngle,
    XJog,
    XJogCurrentLimit,
    XPositionDirect,
    XPositionDirectLimited,
    XPositionTrap,
    XPositionTrapLimited,
    XTorque,
    XTorqueLimited,
    ZeroPosition,
    build_command_frame,
)
from .configs import (
    Address,
    ChecksumMode,
    Direction,
    FirmwareType,
    HomingMode,
    LockParamLevel,
    MotionMode,
    MotorType,
    SyncFlag,
)
from .firmware import FirmwareProfile, get_profile
from .parameters import (
    DMX512Params,
    DeviceParams,
    EmmAutoRunParams,
    EmmConfigParams,
    EmmFastPositionParams,
    EmmJogParams,
    EmmPIDParams,
    EmmPositionParams,
    EmmSystemStatusParams,
    HomeMotorStatus,
    HomingParams,
    HomingStatus,
    IOStatus,
    MotorRHParams,
    MotorStatus,
    OptionStatus,
    ProtectionThreshold,
    VersionParams,
    XAutoRunParams,
    XConfigParams,
    XFastPositionParams,
    XJogCurrentLimitParams,
    XJogParams,
    XPIDParams,
    XPositionDirectLimitedParams,
    XPositionDirectParams,
    XPositionTrapLimitedParams,
    XPositionTrapParams,
    XSystemStatusParams,
    XTorqueLimitedParams,
    XTorqueParams,
)
from .units import (
    degrees_from_pulses,
    pulses_from_degrees,
    x_deg_to_raw,
    x_rpm_to_speed_raw,
)

logger = logging.getLogger(__name__)


class X42SDevice:
    """X42S 双固件步进电机设备门面.

    串口由外部管理，便于同一串口挂接多地址电机。

    使用示例::

        from serial import Serial
        from x42s_stepper import X42SDevice, Direction

        ser = Serial("COM3", 115200, timeout=1)
        motor = X42SDevice(ser, address=1)  # 自动检测固件
        motor.enable()
        motor.jog(speed_rpm=100)
        motor.move_position(angle_deg=90, speed_rpm=200)
        motor.stop()
        motor.disable()
        ser.close()
    """

    def __init__(
        self,
        connection: Optional[Union[Serial, Transport]] = None,
        address: int = 1,
        checksum_mode: ChecksumMode = ChecksumMode.FIXED,
        delay: Optional[float] = None,
        firmware_type: Optional[FirmwareType] = None,
        auto_test: bool = True,
        microstep: int = 16,
        motor_type: MotorType = MotorType.DEGREE_18,
        *,
        serial_connection: Optional[Serial] = None,
    ):
        """初始化设备.

        Args:
            connection: 串口 ``Serial`` 或 ``Transport``（如 ``CanTransport``）
            address: 电机地址 (1-255)
            checksum_mode: 校验模式
            delay: 通讯延迟(秒)
            firmware_type: 固件类型；None 时通过配置块长度自动检测
            auto_test: 是否在初始化时读取版本以验证连接
            microstep: 细分（用于脉冲↔角度换算）
            motor_type: 电机步距角类型
            serial_connection: 兼容旧参数名，等价于传入 ``Serial``
        """
        if connection is None:
            connection = serial_connection
        if connection is None:
            raise TypeError("必须提供 connection（Serial 或 Transport）")
        if not isinstance(connection, (Serial, Transport)):
            raise TypeError(
                "connection 须为 serial.Serial 或 Transport，"
                f"收到 {type(connection)!r}"
            )
        if not 1 <= int(address) <= 255:
            raise ValueError("地址必须在 1-255")
        if isinstance(connection, Serial):
            transport: Transport = SerialTransport(
                connection, checksum_mode=checksum_mode
            )
            self._serial: Optional[Serial] = connection
        else:
            transport = connection
            self._serial = (
                connection.serial if isinstance(connection, SerialTransport) else None
            )

        self._device_params = DeviceParams(
            transport=transport,
            address=address,
            checksum_mode=checksum_mode,
            delay=delay,
        )
        self._microstep = microstep
        self._motor_type = motor_type
        self._firmware_version: Optional[VersionParams] = None
        self._firmware_type: Optional[FirmwareType] = None
        self._profile: Optional[FirmwareProfile] = None
        self._fast_position_ready: bool = False

        if firmware_type is not None:
            self._firmware_type = FirmwareType(firmware_type)
            self._profile = get_profile(self._firmware_type)
        else:
            self.detect_firmware()

        if auto_test:
            self._test_connection()

        logger.info(
            "已连接到 X42S 电机 (地址: %s, 固件: %s)",
            address,
            self._firmware_type,
        )

    def _test_connection(self) -> None:
        """测试与电机的连接."""
        try:
            version = self.get_version()
            logger.info(
                "固件版本: %s, 硬件: %s",
                version.firmware_version_str,
                version.hw_type_str,
            )
        except Exception as e:
            raise ConnectionError(f"无法连接到电机: {e}") from e

    # ==================== 属性 ====================

    @property
    def device_params(self) -> DeviceParams:
        """设备通讯参数."""
        return self._device_params

    @property
    def transport(self) -> Transport:
        """底层传输（SerialTransport / CanTransport）."""
        return self._device_params.transport

    # 兼容原 x42s_can 示例中的 motor.bus
    @property
    def bus(self) -> Transport:
        return self.transport

    @property
    def address(self) -> int:
        """电机地址."""
        return self._device_params.address

    @property
    def firmware_type(self) -> FirmwareType:
        """当前固件类型."""
        if self._firmware_type is None:
            return self.detect_firmware()
        return self._firmware_type

    @property
    def firmware_version(self) -> Optional[VersionParams]:
        """缓存的版本信息；未读取时为 None."""
        return self._firmware_version

    @property
    def profile(self) -> FirmwareProfile:
        """当前固件能力配置."""
        if self._profile is None:
            self._profile = get_profile(self.firmware_type)
        return self._profile

    @property
    def supports_torque(self) -> bool:
        """是否支持力矩模式 (X 固件)."""
        return self.profile.supports_torque

    @property
    def supports_position_direct(self) -> bool:
        """是否支持直通位置模式 (X 固件)."""
        return self.profile.supports_position_direct

    @property
    def supports_fast_position(self) -> bool:
        """是否支持快速位置 F1/FC (需固件版本 >= V2.0.0)."""
        if not self.profile.supports_fast_position:
            return False
        if self._firmware_version is None:
            try:
                self.get_version()
            except Exception:
                return False
        ver = self._firmware_version
        return ver is not None and ver.firmware_version >= 200

    @property
    def microstep(self) -> int:
        """当前细分缓存值."""
        return self._microstep

    @property
    def motor_type(self) -> MotorType:
        """当前电机类型缓存值."""
        return self._motor_type

    # ==================== 辅助 ====================

    def _require_torque(self) -> None:
        """力矩相关命令仅 X 固件可用."""
        if not self.supports_torque:
            raise FirmwareCapabilityError(
                f"力矩模式仅支持 X 固件 (当前: {self.firmware_type})"
            )

    def _require_position_direct(self) -> None:
        """直通位置模式仅 X 固件可用."""
        if not self.supports_position_direct:
            raise FirmwareCapabilityError(
                f"直通位置模式仅支持 X 固件 (当前: {self.firmware_type})"
            )

    def _require_fast_position(self) -> None:
        if not self.supports_fast_position:
            raise FirmwareCapabilityError(
                "快速位置模式需要固件 V2.0.0+ (F1/FC)"
            )

    def _sync_flag(self, sync: bool) -> SyncFlag:
        return SyncFlag.SYNC if sync else SyncFlag.IMMEDIATE

    def _is_x(self) -> bool:
        return self.firmware_type == FirmwareType.X_FIRMWARE

    # ==================== 固件 ====================

    def detect_firmware(self) -> FirmwareType:
        """通过驱动配置块长度检测固件类型并更新 profile.

        以 ``0x42`` 返回的总字节数为准: ``0x25``=X，``0x21``=Emm/Turbo。

        ``EMM_TURBO`` 与 Emm 布局相同，探测结果为 ``EMM_FIRMWARE``；
        若此前通过 ``switch_firmware(EMM_TURBO)`` 切入，请以
        ``firmware_type`` 缓存为准，或再次显式切换。
        """
        prev = self._firmware_type
        cmd = ProbeConfigFirmware(self._device_params)
        if cmd.data is None:
            raise ConnectionError("无法通过配置块探测固件类型")
        fw = cmd.data
        # Turbo 与 Emm 配置块相同；保留已记录的 Turbo 状态
        if (
            fw == FirmwareType.EMM_FIRMWARE
            and prev == FirmwareType.EMM_TURBO
        ):
            fw = FirmwareType.EMM_TURBO
        self._firmware_type = fw
        self._profile = get_profile(fw)
        self._fast_position_ready = False
        logger.info("检测到固件类型: %s (via config layout)", fw)
        return fw

    def switch_firmware(self, fw: FirmwareType, store: bool = True) -> bool:
        """切换固件类型并校验.

        Args:
            fw: 目标 ``X_FIRMWARE`` / ``EMM_FIRMWARE`` / ``EMM_TURBO``
            store: 是否掉电保存。建议 ``True``。

        Returns:
            切换后检测结果是否与目标一致（Turbo 校验为 Emm 布局成功）
        """
        if not store:
            logger.warning(
                "switch_firmware(store=False) 可能不生效，建议 store=True"
            )
        target = FirmwareType(fw)
        cmd = SetFirmwareType(
            self._device_params, firmware_type=target, store=store
        )
        if not cmd.is_success:
            return False
        time.sleep(0.2)
        # 清空 prev 再探测，避免 Turbo 粘滞误判
        self._firmware_type = None
        detected = self.detect_firmware()
        if target == FirmwareType.EMM_TURBO:
            ok = detected == FirmwareType.EMM_FIRMWARE
            if ok:
                self._firmware_type = FirmwareType.EMM_TURBO
                self._profile = get_profile(FirmwareType.EMM_TURBO)
        elif target == FirmwareType.EMM_FIRMWARE:
            ok = detected == FirmwareType.EMM_FIRMWARE
        else:
            ok = detected == FirmwareType.X_FIRMWARE
        if not ok:
            logger.warning(
                "固件切换校验失败: 期望 %s, 实际 %s", target, detected
            )
        return ok

    # ==================== 触发动作 ====================

    def calibrate_encoder(self) -> bool:
        """触发编码器校准."""
        return CalibrateEncoder(self._device_params).is_success

    def restart(self) -> bool:
        """重启电机."""
        return Restart(self._device_params).is_success

    def zero_position(self) -> bool:
        """将当前位置角度清零."""
        return ZeroPosition(self._device_params).is_success

    def clear_protection(self) -> bool:
        """解除堵转/过热/过流保护."""
        return ClearProtection(self._device_params).is_success

    def factory_reset(self) -> bool:
        """恢复出厂设置 (恢复后需重新上电并空载校准)."""
        return FactoryReset(self._device_params).is_success

    # ==================== 运动控制 ====================

    def enable(self, sync: bool = False) -> bool:
        """使能电机(锁轴)."""
        return Enable(
            self._device_params,
            enable=True,
            sync_flag=self._sync_flag(sync),
        ).is_success

    def disable(self, sync: bool = False) -> bool:
        """失能电机(松轴)."""
        return Disable(
            self._device_params,
            sync_flag=self._sync_flag(sync),
        ).is_success

    def stop(self, sync: bool = False) -> bool:
        """立即停止 (急停)."""
        return EStop(
            self._device_params,
            sync_flag=self._sync_flag(sync),
        ).is_success

    def jog(
        self,
        speed_rpm: float = 100,
        direction: Direction = Direction.CW,
        acceleration: Optional[int] = None,
        sync: bool = False,
        max_current_ma: Optional[int] = None,
    ) -> bool:
        """速度模式运动 (物理单位 RPM).

        Emm: acceleration 默认 10 (档位 0-255)。
        X: acceleration 默认 1000 (RPM/s)；若提供 max_current_ma 则走限电流命令。
        """
        sync_flag = self._sync_flag(sync)
        if self._is_x():
            accel = 1000 if acceleration is None else int(acceleration)
            speed_raw = x_rpm_to_speed_raw(speed_rpm)
            if max_current_ma is not None:
                params = XJogCurrentLimitParams(
                    direction=direction,
                    acceleration_rpm_s=accel,
                    speed_raw=speed_raw,
                    sync=sync_flag,
                    max_current_ma=int(max_current_ma),
                )
                return XJogCurrentLimit(self._device_params, params=params).is_success
            params = XJogParams(
                direction=direction,
                acceleration_rpm_s=accel,
                speed_raw=speed_raw,
                sync=sync_flag,
            )
            return XJog(self._device_params, params=params).is_success

        if max_current_ma is not None:
            raise FirmwareCapabilityError(
                "速度限电流模式仅支持 X 固件"
            )
        accel = 10 if acceleration is None else int(acceleration)
        params = EmmJogParams(
            direction=direction,
            speed=int(round(speed_rpm)),
            acceleration=accel,
            sync_flag=sync_flag,
        )
        return EmmJog(self._device_params, params=params).is_success

    def jog_raw(
        self,
        speed: int,
        direction: Direction = Direction.CW,
        acceleration: Optional[int] = None,
        sync: bool = False,
        max_current_ma: Optional[int] = None,
    ) -> bool:
        """速度模式原始值包装.

        Emm: speed 为整数 RPM；X: speed 为 0.1RPM 原始值。
        """
        if self._is_x():
            rpm = speed / 10.0
        else:
            rpm = float(speed)
        return self.jog(
            speed_rpm=rpm,
            direction=direction,
            acceleration=acceleration,
            sync=sync,
            max_current_ma=max_current_ma,
        )

    def move_position(
        self,
        angle_deg: Optional[float] = None,
        pulses: Optional[int] = None,
        speed_rpm: float = 100,
        direction: Optional[Direction] = None,
        acceleration: Optional[int] = None,
        decel: Optional[int] = None,
        motion_mode: MotionMode = MotionMode.RELATIVE_LAST,
        sync: bool = False,
        max_current_ma: Optional[int] = None,
        mode: str = "trap",
    ) -> bool:
        """位置模式运动 (统一物理单位).

        Args:
            angle_deg: 目标角度(度)；与 pulses 二选一（Emm 两者都给时优先 pulses）
            pulses: Emm 脉冲数；X 固件会先换算为角度再发
            speed_rpm: 速度 RPM
            direction: 方向；None 时由 angle/pulses 符号决定
            acceleration: Emm 为档位(默认10)；X 为 RPM/s(默认1000, trap 默认500)
            decel: X 梯形减速 RPM/s；默认等于 acceleration
            motion_mode: 相对/绝对等运动模式
            sync: 多机同步标志
            max_current_ma: X 限电流版本电流上限
            mode: X 固件 ``"trap"``(梯形) 或 ``"direct"``(直通)

        Returns:
            是否成功
        """
        if angle_deg is None and pulses is None:
            raise ValueError("必须提供 angle_deg 或 pulses 之一")

        sync_flag = self._sync_flag(sync)

        if self._is_x():
            return self._move_position_x(
                angle_deg=angle_deg,
                pulses=pulses,
                speed_rpm=speed_rpm,
                direction=direction,
                acceleration=acceleration,
                decel=decel,
                motion_mode=motion_mode,
                sync_flag=sync_flag,
                max_current_ma=max_current_ma,
                mode=mode,
            )

        if max_current_ma is not None:
            raise FirmwareCapabilityError(
                "位置限电流模式仅支持 X 固件"
            )
        if mode == "direct":
            raise FirmwareCapabilityError(
                "直通位置模式仅支持 X 固件"
            )

        # Emm: pulses 优先
        if pulses is not None:
            pulse_count = int(pulses)
        else:
            pulse_count = pulses_from_degrees(
                float(angle_deg),
                full_steps=self._motor_type,
                microstep=self._microstep,
            )

        if direction is None:
            if pulse_count < 0:
                direction = Direction.CCW
                pulse_count = abs(pulse_count)
            else:
                direction = Direction.CW
        else:
            pulse_count = abs(pulse_count)

        accel = 10 if acceleration is None else int(acceleration)
        params = EmmPositionParams(
            direction=direction,
            speed=int(round(speed_rpm)),
            acceleration=accel,
            pulse_count=pulse_count,
            motion_mode=motion_mode,
            sync_flag=sync_flag,
        )
        return EmmPosition(self._device_params, params=params).is_success

    def _move_position_x(
        self,
        angle_deg: Optional[float],
        pulses: Optional[int],
        speed_rpm: float,
        direction: Optional[Direction],
        acceleration: Optional[int],
        decel: Optional[int],
        motion_mode: MotionMode,
        sync_flag: SyncFlag,
        max_current_ma: Optional[int],
        mode: str,
    ) -> bool:
        if angle_deg is not None:
            degrees = float(angle_deg)
        else:
            degrees = degrees_from_pulses(
                int(pulses),
                full_steps=self._motor_type,
                microstep=self._microstep,
            )

        if direction is None:
            if degrees < 0:
                direction = Direction.CCW
                degrees = abs(degrees)
            else:
                direction = Direction.CW
        else:
            degrees = abs(degrees)

        angle_raw = x_deg_to_raw(degrees)
        speed_raw = x_rpm_to_speed_raw(speed_rpm)
        mode_l = (mode or "trap").lower()

        if mode_l == "direct":
            self._require_position_direct()
            if max_current_ma is not None:
                params = XPositionDirectLimitedParams(
                    direction=direction,
                    speed_raw=speed_raw,
                    angle_raw=angle_raw,
                    motion_mode=motion_mode,
                    sync=sync_flag,
                    max_current_ma=int(max_current_ma),
                )
                return XPositionDirectLimited(
                    self._device_params, params=params
                ).is_success
            params = XPositionDirectParams(
                direction=direction,
                speed_raw=speed_raw,
                angle_raw=angle_raw,
                motion_mode=motion_mode,
                sync=sync_flag,
            )
            return XPositionDirect(self._device_params, params=params).is_success

        if mode_l != "trap":
            raise ValueError('mode 须为 "trap" 或 "direct"')

        # trap 默认加速度 500 RPM/s（与参数类默认一致）；若未指定则用 1000 作为 jog 风格默认也可，
        # 规格要求 X acceleration default 1000，此处对 trap 使用 1000。
        accel = 1000 if acceleration is None else int(acceleration)
        decel_v = accel if decel is None else int(decel)

        if max_current_ma is not None:
            params = XPositionTrapLimitedParams(
                direction=direction,
                accel_rpm_s=accel,
                decel_rpm_s=decel_v,
                max_speed_raw=speed_raw,
                angle_raw=angle_raw,
                motion_mode=motion_mode,
                sync=sync_flag,
                max_current_ma=int(max_current_ma),
            )
            return XPositionTrapLimited(
                self._device_params, params=params
            ).is_success

        params = XPositionTrapParams(
            direction=direction,
            accel_rpm_s=accel,
            decel_rpm_s=decel_v,
            max_speed_raw=speed_raw,
            angle_raw=angle_raw,
            motion_mode=motion_mode,
            sync=sync_flag,
        )
        return XPositionTrap(self._device_params, params=params).is_success

    def configure_fast_position(
        self,
        speed_rpm: float = 100,
        acceleration: Optional[int] = None,
        decel: Optional[int] = None,
        motion_mode: MotionMode = MotionMode.RELATIVE_LAST,
        sync: bool = False,
        max_current_ma: int = 3000,
    ) -> bool:
        """快速位置模式设参 (F1, V2.0.0+).

        Emm: acceleration 为档位(默认10)；后续用 ``move_fast_pulses``。
        X: acceleration/decel 为 RPM/s(默认1000)；后续用 ``move_fast_degrees``。
        """
        self._require_fast_position()
        sync_flag = self._sync_flag(sync)
        if self._is_x():
            accel = 1000 if acceleration is None else int(acceleration)
            decel_v = accel if decel is None else int(decel)
            params = XFastPositionParams(
                accel_rpm_s=accel,
                decel_rpm_s=decel_v,
                max_speed_raw=x_rpm_to_speed_raw(speed_rpm),
                motion_mode=motion_mode,
                sync=sync_flag,
                max_current_ma=int(max_current_ma),
            )
            ok = XConfigureFastPosition(
                self._device_params, params=params
            ).is_success
        else:
            accel = 10 if acceleration is None else int(acceleration)
            params = EmmFastPositionParams(
                speed=int(round(speed_rpm)),
                acceleration=accel,
                motion_mode=motion_mode,
                sync_flag=sync_flag,
            )
            ok = EmmConfigureFastPosition(
                self._device_params, params=params
            ).is_success
        self._fast_position_ready = bool(ok)
        return ok

    def move_fast_pulses(self, pulses: int) -> bool:
        """快速位置发脉冲 (Emm FC). 需先 ``configure_fast_position``."""
        self._require_fast_position()
        if self._is_x():
            raise FirmwareCapabilityError(
                "X 固件请使用 move_fast_degrees (FC 为 0.1° 角度)"
            )
        return EmmFastPositionPulse(self._device_params, pulses=pulses).is_success

    def move_fast_degrees(self, degrees: float) -> bool:
        """快速位置运动.

        Emm: 按当前细分换算脉冲后走 FC。
        X: 直接发 0.1° 有符号角度 (FC)。
        均需先 ``configure_fast_position``。
        """
        self._require_fast_position()
        if self._is_x():
            angle_raw = int(round(degrees * 10))
            return XFastPositionAngle(
                self._device_params, angle_raw=angle_raw
            ).is_success
        pulses = pulses_from_degrees(
            degrees,
            full_steps=self._motor_type,
            microstep=self._microstep,
        )
        return self.move_fast_pulses(pulses)

    def move_fast_revolutions(self, revolutions: float) -> bool:
        """快速位置按圈数运动 — 需已 configure_fast_position."""
        return self.move_fast_degrees(revolutions * 360.0)

    def torque(
        self,
        current_ma: int,
        direction: Direction = Direction.CW,
        slope_ma_s: int = 200,
        sync: bool = False,
    ) -> bool:
        """力矩模式 (仅 X 固件)."""
        self._require_torque()
        params = XTorqueParams(
            direction=direction,
            slope_ma_s=slope_ma_s,
            current_ma=current_ma,
            sync=self._sync_flag(sync),
        )
        return XTorque(self._device_params, params=params).is_success

    def torque_limited(
        self,
        current_ma: int,
        max_speed_rpm: float,
        direction: Direction = Direction.CW,
        slope_ma_s: int = 200,
        sync: bool = False,
    ) -> bool:
        """力矩模式限速 (仅 X 固件)."""
        self._require_torque()
        params = XTorqueLimitedParams(
            direction=direction,
            slope_ma_s=slope_ma_s,
            current_ma=current_ma,
            sync=self._sync_flag(sync),
            max_speed_raw=x_rpm_to_speed_raw(max_speed_rpm),
        )
        return XTorqueLimited(self._device_params, params=params).is_success

    @staticmethod
    def sync_move(
        connection: Union[DeviceParams, Transport],
        checksum_mode: ChecksumMode = ChecksumMode.FIXED,
        delay: Optional[float] = None,
    ) -> bool:
        """触发多机同步运动 (广播).

        Args:
            connection: ``DeviceParams`` 或共享 ``Transport``（串口/CAN）
        """
        if isinstance(connection, DeviceParams):
            return SyncMove(connection).is_success
        params = DeviceParams(
            transport=connection,
            address=Address.BROADCAST,
            checksum_mode=checksum_mode,
            delay=delay,
        )
        return SyncMove(params).is_success

    @staticmethod
    def multi_motor(
        frames: List[bytes],
        connection: Union[DeviceParams, Transport],
        expect_ack: bool = True,
        checksum_mode: ChecksumMode = ChecksumMode.FIXED,
        delay: Optional[float] = None,
    ) -> bool:
        """发送多电机命令.

        Args:
            frames: 已含校验码的完整子命令帧
            connection: ``DeviceParams`` 或共享 ``Transport``
            expect_ack: 是否等待地址1确认
        """
        if isinstance(connection, DeviceParams):
            device_params = connection
            temp = DeviceParams(
                transport=device_params.transport,
                address=Address.BROADCAST,
                checksum_mode=device_params.checksum_mode,
                delay=device_params.delay,
            )
        else:
            temp = DeviceParams(
                transport=connection,
                address=Address.BROADCAST,
                checksum_mode=checksum_mode,
                delay=delay,
            )
        return MultiMotor(temp, frames=frames, expect_ack=expect_ack).is_success

    @staticmethod
    def build_frame(
        body: bytes,
        checksum_mode: ChecksumMode = ChecksumMode.FIXED,
    ) -> bytes:
        """构建含校验码的完整命令帧 (用于 multi_motor 子帧)."""
        return build_command_frame(body, checksum_mode)

    # ==================== 原点回零 ====================

    def set_home_zero(self, store: bool = True) -> bool:
        """设置单圈回零的零点位置."""
        return SetHomeZero(self._device_params, store=store).is_success

    def home(
        self,
        mode: HomingMode = HomingMode.NEAREST,
        sync: bool = False,
    ) -> bool:
        """触发回零."""
        return Home(
            self._device_params,
            mode=mode,
            sync_flag=self._sync_flag(sync),
        ).is_success

    def stop_home(self) -> bool:
        """强制中断并退出回零."""
        return StopHome(self._device_params).is_success

    def get_homing_params(self) -> HomingParams:
        """读取回零参数."""
        return GetHomingParams(self._device_params).data

    def set_homing_params(self, params: HomingParams, store: bool = True) -> bool:
        """修改回零参数."""
        return SetHomingParams(
            self._device_params, params=params, store=store
        ).is_success

    def get_homing_status(self) -> HomingStatus:
        """读取回零状态标志."""
        return GetHomingStatus(self._device_params).data

    def wait_homing(
        self,
        timeout: float = 30.0,
        poll: float = 0.05,
    ) -> bool:
        """等待回零结束.

        Returns:
            True 表示回零流程结束且未失败；超时或失败返回 False
        """
        start = time.time()
        while time.time() - start < timeout:
            status = self.get_homing_status()
            if status.homing_failed:
                return False
            if not status.is_homing:
                return True
            time.sleep(poll)
        return False

    # ==================== 读取 ====================

    def get_version(self) -> VersionParams:
        """读取固件版本和硬件版本."""
        cmd = GetVersion(self._device_params)
        if cmd.data is not None:
            self._firmware_version = cmd.data
        return cmd.data

    def ping(self) -> VersionParams:
        """通讯探测（读版本）."""
        return self.get_version()

    def get_option_status(self) -> OptionStatus:
        """读取选项参数状态."""
        return GetOptionStatus(self._device_params).data

    def get_motor_rh(self) -> MotorRHParams:
        """读取相电阻和相电感."""
        return GetMotorRH(self._device_params).data

    def get_bus_voltage(self) -> int:
        """读取总线电压 (mV)."""
        return GetBusVoltage(self._device_params).data

    def get_bus_current(self) -> int:
        """读取总线电流 (mA)."""
        return GetBusCurrent(self._device_params).data

    def get_phase_current(self) -> int:
        """读取相电流 (mA)."""
        return GetPhaseCurrent(self._device_params).data

    def get_battery_voltage(self) -> int:
        """读取电池电压 (mV, X42S)."""
        return GetBatteryVoltage(self._device_params).data

    def get_temperature(self) -> int:
        """读取驱动温度 (°C)."""
        return GetTemperature(self._device_params).data

    def get_encoder(self) -> int:
        """读取线性化编码器值 (0-65535 → 0-360°)."""
        return GetEncoder(self._device_params).data

    def get_encoder_degrees(self) -> float:
        """读取编码器角度 (度)."""
        return (self.get_encoder() * 360.0) / 65536.0

    def get_pulse_count(self) -> int:
        """读取输入脉冲数 (有符号)."""
        return GetPulseCount(self._device_params).data

    def get_target_position(self) -> float:
        """读取电机目标位置 (度)."""
        return GetTargetPosition(
            self._device_params, firmware_type=self.firmware_type
        ).data

    def get_realtime_position(self) -> float:
        """读取电机实时位置 (度)."""
        return GetRealtimePosition(
            self._device_params, firmware_type=self.firmware_type
        ).data

    def get_position_error(self) -> float:
        """读取电机位置误差 (度)."""
        return GetPositionError(
            self._device_params, firmware_type=self.firmware_type
        ).data

    def get_realtime_speed(self) -> float:
        """读取电机实时转速 (RPM)."""
        return GetRealtimeSpeed(
            self._device_params, firmware_type=self.firmware_type
        ).data

    def get_realtime_target(self) -> float:
        """读取电机实时设定目标位置 (度)."""
        return GetRealtimeTarget(
            self._device_params, firmware_type=self.firmware_type
        ).data

    def get_motor_status(self) -> MotorStatus:
        """读取电机状态标志."""
        return GetMotorStatus(self._device_params).data

    def get_home_motor_status(self) -> HomeMotorStatus:
        """读取回零状态 + 电机状态."""
        return GetHomeMotorStatus(self._device_params).data

    def get_io_status(self) -> IOStatus:
        """读取引脚 IO 电平状态."""
        return GetIOStatus(self._device_params).data

    def get_pid(self) -> Union[EmmPIDParams, XPIDParams]:
        """读取 PID 参数 (固件感知)."""
        return GetPID(
            self._device_params, firmware_type=self.firmware_type
        ).data

    def get_config(self) -> Union[EmmConfigParams, XConfigParams]:
        """读取驱动配置参数 (按返回布局自动解析 X/Emm)."""
        data = GetConfig(self._device_params).data
        if isinstance(data, XConfigParams):
            self._firmware_type = FirmwareType.X_FIRMWARE
            self._profile = get_profile(self._firmware_type)
        elif isinstance(data, EmmConfigParams):
            self._firmware_type = FirmwareType.EMM_FIRMWARE
            self._profile = get_profile(self._firmware_type)
        return data

    def get_system_status(
        self,
    ) -> Union[EmmSystemStatusParams, XSystemStatusParams]:
        """读取系统状态参数 (按返回布局自动解析 X/Emm)."""
        return GetSystemStatus(self._device_params).data

    def get_protection_threshold(self) -> ProtectionThreshold:
        """读取过热过流保护检测阈值."""
        return GetProtectionThreshold(self._device_params).data

    def get_heartbeat_time(self) -> int:
        """读取心跳保护时间 (ms), 0 表示关闭."""
        return GetHeartbeatTime(self._device_params).data

    def get_position_window(self) -> float:
        """读取位置到达窗口 (度)."""
        return GetPositionWindow(self._device_params).data

    def get_dmx512_params(self) -> DMX512Params:
        """读取 DMX512 协议参数."""
        return GetDMX512Params(self._device_params).data

    def get_integral_stiffness(self) -> int:
        """读取积分限幅/刚性系数."""
        return GetIntegralStiffness(self._device_params).data

    def get_collision_return_angle(self) -> float:
        """读取碰撞回零返回角度 (度)."""
        return GetCollisionReturnAngle(self._device_params).data

    def timed_return(self, info_code: int, interval_ms: int = 0) -> bool:
        """定时返回信息; interval_ms=0 停止返回."""
        return TimedReturn(
            self._device_params,
            info_code=info_code,
            interval_ms=interval_ms,
        ).is_success

    def wait_position_reached(
        self,
        timeout: float = 30.0,
        poll: float = 0.05,
    ) -> bool:
        """轮询 motor_status.position_reached 直至到位或超时."""
        start = time.time()
        while time.time() - start < timeout:
            status = self.get_motor_status()
            if status is not None and status.position_reached:
                return True
            time.sleep(poll)
        return False

    def read_event(self, timeout: float = 0.1) -> Optional[Tuple[int, int]]:
        """从串口读取动作完成帧 (地址 + 功能码 + 9F + 校验).

        Returns:
            (addr, code) 或 None
        """
        ser = self._serial
        if ser is None:
            raise TransportError("read_event 仅支持串口传输")
        old_timeout = ser.timeout
        buf = b""
        deadline = time.time() + timeout
        try:
            ser.timeout = min(timeout, 0.05) if timeout > 0 else 0
            while time.time() < deadline:
                waiting = getattr(ser, "in_waiting", 0) or 0
                chunk = ser.read(waiting if waiting > 0 else 1)
                if chunk:
                    buf += chunk
                    for i in range(len(buf) - 3):
                        if buf[i + 2] == 0x9F:
                            return int(buf[i]), int(buf[i + 1])
                else:
                    time.sleep(0.005)
            return None
        finally:
            ser.timeout = old_timeout

    # ==================== 写入 ====================

    def set_id(self, new_id: int, store: bool = True) -> bool:
        """修改电机 ID/地址."""
        cmd = SetID(self._device_params, new_id=new_id, store=store)
        if cmd.is_success:
            self._device_params.address = Address(new_id)
        return cmd.is_success

    def set_microstep(self, microstep: int, store: bool = True) -> bool:
        """修改细分值，成功后更新本地缓存."""
        cmd = SetMicrostep(
            self._device_params, microstep=microstep, store=store
        )
        if cmd.is_success:
            self._microstep = microstep
        return cmd.is_success

    def set_loop_mode(self, closed_loop: bool = True, store: bool = True) -> bool:
        """修改开环/闭环控制模式."""
        return SetLoopMode(
            self._device_params, closed_loop=closed_loop, store=store
        ).is_success

    def set_open_loop_current(self, current_ma: int, store: bool = True) -> bool:
        """修改开环模式工作电流 (mA)."""
        return SetOpenLoopCurrent(
            self._device_params, current_ma=current_ma, store=store
        ).is_success

    def set_closed_loop_current(self, current_ma: int, store: bool = True) -> bool:
        """修改闭环模式最大电流 (mA)."""
        return SetClosedLoopCurrent(
            self._device_params, current_ma=current_ma, store=store
        ).is_success

    def set_pid(
        self,
        params: Union[EmmPIDParams, XPIDParams],
        store: bool = True,
    ) -> bool:
        """修改 PID 参数 (按参数类型路由；须与当前固件匹配)."""
        if isinstance(params, XPIDParams):
            if not self._is_x():
                raise FirmwareCapabilityError(
                    f"set_pid(XPIDParams) 仅支持 X 固件 (当前: {self.firmware_type})"
                )
            return SetXPID(
                self._device_params, params=params, store=store
            ).is_success
        if isinstance(params, EmmPIDParams):
            if self._is_x():
                raise FirmwareCapabilityError(
                    f"set_pid(EmmPIDParams) 仅支持 Emm/Turbo 固件 (当前: {self.firmware_type})"
                )
            return SetEmmPID(
                self._device_params, params=params, store=store
            ).is_success
        raise TypeError("params 须为 EmmPIDParams 或 XPIDParams")

    def set_config(
        self,
        params: Union[EmmConfigParams, XConfigParams],
        store: bool = True,
    ) -> bool:
        """修改驱动配置参数 (按参数类型路由)."""
        if isinstance(params, XConfigParams):
            return SetXConfig(
                self._device_params, params=params, store=store
            ).is_success
        if isinstance(params, EmmConfigParams):
            return SetEmmConfig(
                self._device_params, params=params, store=store
            ).is_success
        raise TypeError("params 须为 EmmConfigParams 或 XConfigParams")

    def set_auto_run(
        self,
        params: Union[EmmAutoRunParams, XAutoRunParams],
    ) -> bool:
        """存储一组速度参数，上电自动运行."""
        if isinstance(params, XAutoRunParams):
            return SetXAutoRun(self._device_params, params=params).is_success
        if isinstance(params, EmmAutoRunParams):
            return SetEmmAutoRun(self._device_params, params=params).is_success
        raise TypeError("params 须为 EmmAutoRunParams 或 XAutoRunParams")

    def set_motor_direction(
        self,
        direction: Direction = Direction.CW,
        store: bool = True,
    ) -> bool:
        """修改电机运动正方向."""
        return SetMotorDirection(
            self._device_params, direction=direction, store=store
        ).is_success

    def set_scale_x10(self, enable: bool = False, store: bool = True) -> bool:
        """修改命令速度/角度是否缩小 10 倍输入 (别名)."""
        return self.set_scale_input(enable=enable, store=store)

    def set_scale_input(self, enable: bool = False, store: bool = True) -> bool:
        """修改命令速度/角度值是否缩小 10 倍输入."""
        return SetScaleInput(
            self._device_params, enable=enable, store=store
        ).is_success

    def set_lock_button(self, lock: bool = False, store: bool = True) -> bool:
        """修改锁定按键功能."""
        return SetLockButton(
            self._device_params, lock=lock, store=store
        ).is_success

    def set_power_off_flag(self, flag: bool = False) -> bool:
        """修改掉电标志."""
        return SetPowerOffFlag(self._device_params, flag=flag).is_success

    def set_motor_type(
        self,
        motor_type: MotorType = MotorType.DEGREE_18,
        store: bool = True,
    ) -> bool:
        """修改电机类型，成功后更新本地缓存."""
        cmd = SetMotorType(
            self._device_params, motor_type=motor_type, store=store
        )
        if cmd.is_success:
            self._motor_type = motor_type
        return cmd.is_success

    def set_protection_threshold(
        self,
        params: ProtectionThreshold,
        store: bool = True,
    ) -> bool:
        """修改过热过流保护检测阈值."""
        return SetProtectionThreshold(
            self._device_params, params=params, store=store
        ).is_success

    def set_heartbeat_time(self, time_ms: int = 0, store: bool = True) -> bool:
        """修改心跳保护功能时间 (ms), 0 关闭."""
        return SetHeartbeatTime(
            self._device_params, time_ms=time_ms, store=store
        ).is_success

    def set_position_window(
        self,
        window_deg: float = 0.8,
        store: bool = True,
    ) -> bool:
        """修改位置到达窗口 (度)."""
        return SetPositionWindow(
            self._device_params, window_deg=window_deg, store=store
        ).is_success

    def set_dmx512_params(
        self,
        params: DMX512Params,
        store: bool = True,
    ) -> bool:
        """修改 DMX512 协议参数."""
        return SetDMX512Params(
            self._device_params, params=params, store=store
        ).is_success

    def set_integral_stiffness(
        self,
        value: int = 65535,
        store: bool = True,
    ) -> bool:
        """修改积分限幅/刚性系数."""
        return SetIntegralStiffness(
            self._device_params, value=value, store=store
        ).is_success

    def set_collision_return_angle(
        self,
        angle_deg: float = 0.0,
        store: bool = True,
    ) -> bool:
        """修改碰撞回零返回角度 (度); 0 表示按电流检测返回."""
        return SetCollisionReturnAngle(
            self._device_params, angle_deg=angle_deg, store=store
        ).is_success

    def set_lock_param(
        self,
        level: LockParamLevel = LockParamLevel.UNLOCKED,
        store: bool = True,
    ) -> bool:
        """修改锁定修改参数功能等级."""
        return SetLockParam(
            self._device_params, level=level, store=store
        ).is_success

    # ==================== 便捷方法 ====================

    @staticmethod
    def broadcast_get_id(connection: Union[Serial, Transport]) -> int:
        """广播读取 ID 地址 (单机接线时使用)."""
        if isinstance(connection, Serial):
            transport: Transport = SerialTransport(connection)
        else:
            transport = connection
        device_params = DeviceParams(
            transport=transport,
            address=Address.BROADCAST,
        )
        return BroadcastGetID(device_params).data

    def is_enabled(self) -> bool:
        """检查电机是否使能."""
        return bool(self.get_motor_status().enabled)

    def is_position_reached(self) -> bool:
        """检查是否到达目标位置."""
        return bool(self.get_motor_status().position_reached)

    def is_stalled(self) -> bool:
        """检查是否堵转/堵转保护."""
        status = self.get_motor_status()
        return bool(status.stall_detected or status.stall_protected)

    def __repr__(self) -> str:
        return (
            f"X42SDevice(address={self.address}, "
            f"firmware={self._firmware_type})"
        )

    def __str__(self) -> str:
        return f"X42S步进电机 (地址: {self.address}, 固件: {self._firmware_type})"
