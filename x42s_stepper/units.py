"""X42S 双固件单位换算.

X 固件:
  - 位置角度: 原始值单位 0.1°
  - 位置误差: 原始值单位 0.01°
  - 速度: 原始值单位 0.1 RPM

Emm 固件:
  - 位置/误差: (raw * 360) / 65536 度
  - 速度: 整数 RPM
"""

from typing import Union

from .configs import FirmwareType, MotorType


def signed_from_sign_byte(sign: int, magnitude: int) -> int:
    """根据符号字节还原有符号值.

    Args:
        sign: 00=正, 01=负
        magnitude: 无符号幅值
    """
    return -magnitude if sign == 1 else magnitude


def sign_and_magnitude(value: int) -> tuple:
    """将有符号整数拆为 (sign_byte, magnitude)."""
    if value < 0:
        return 1, -value
    return 0, value


def emm_raw_to_deg(raw: int) -> float:
    """Emm 位置/误差原始值 → 度: (raw * 360) / 65536."""
    return (raw * 360.0) / 65536.0


def emm_deg_to_raw(deg: float) -> int:
    """度 → Emm 位置原始值."""
    return int(round(deg * 65536.0 / 360.0))


def x_raw_to_deg(raw: int) -> float:
    """X 位置原始值 → 度 (单位 0.1°)."""
    return raw / 10.0


def x_deg_to_raw(deg: float) -> int:
    """度 → X 位置原始值 (round deg * 10)."""
    return int(round(deg * 10.0))


def x_error_raw_to_deg(raw: int) -> float:
    """X 位置误差原始值 → 度 (单位 0.01°)."""
    return raw / 100.0


def x_error_deg_to_raw(deg: float) -> int:
    """度 → X 位置误差原始值."""
    return int(round(deg * 100.0))


def x_speed_raw_to_rpm(raw: int) -> float:
    """X 速度原始值 → RPM (单位 0.1 RPM)."""
    return raw / 10.0


def x_rpm_to_speed_raw(rpm: float) -> int:
    """RPM → X 速度原始值 (round rpm * 10)."""
    return int(round(rpm * 10.0))


def emm_speed_raw_to_rpm(raw: int) -> float:
    """Emm 速度原始值 → RPM (整数 RPM)."""
    return float(raw)


def emm_rpm_to_speed_raw(rpm: float) -> int:
    """RPM → Emm 速度原始值."""
    return int(round(rpm))


def normalize_microstep(microstep: int) -> int:
    """细分值规范化: 协议 0 表示 256 细分."""
    return 256 if microstep == 0 else microstep


def pulses_from_degrees(
    degrees: float,
    full_steps: Union[int, MotorType] = 200,
    microstep: int = 16,
) -> int:
    """角度 → Emm 位置模式脉冲数.

    Args:
        degrees: 目标角度 (可为负，返回有符号脉冲)
        full_steps: 整步数/圈，或 MotorType
        microstep: 细分 (0 表示 256)
    """
    if isinstance(full_steps, MotorType):
        steps = full_steps.full_steps_per_rev
    else:
        steps = full_steps
    ms = normalize_microstep(microstep)
    return int(degrees / 360.0 * steps * ms)


def degrees_from_pulses(
    pulses: int,
    full_steps: Union[int, MotorType] = 200,
    microstep: int = 16,
) -> float:
    """Emm 脉冲数 → 角度."""
    if isinstance(full_steps, MotorType):
        steps = full_steps.full_steps_per_rev
    else:
        steps = full_steps
    ms = normalize_microstep(microstep)
    return pulses * 360.0 / (steps * ms)


def position_raw_to_deg(raw: int, firmware: FirmwareType) -> float:
    """按固件类型将位置原始值转换为度."""
    if firmware == FirmwareType.X_FIRMWARE:
        return x_raw_to_deg(raw)
    return emm_raw_to_deg(raw)


def position_deg_to_raw(deg: float, firmware: FirmwareType) -> int:
    """按固件类型将度转换为位置原始值."""
    if firmware == FirmwareType.X_FIRMWARE:
        return x_deg_to_raw(deg)
    return emm_deg_to_raw(deg)


def error_raw_to_deg(raw: int, firmware: FirmwareType) -> float:
    """按固件类型将位置误差原始值转换为度."""
    if firmware == FirmwareType.X_FIRMWARE:
        return x_error_raw_to_deg(raw)
    return emm_raw_to_deg(raw)


def speed_raw_to_rpm(raw: int, firmware: FirmwareType) -> float:
    """按固件类型将速度原始值转换为 RPM."""
    if firmware == FirmwareType.X_FIRMWARE:
        return x_speed_raw_to_rpm(raw)
    return emm_speed_raw_to_rpm(raw)


def rpm_to_speed_raw(rpm: float, firmware: FirmwareType) -> int:
    """按固件类型将 RPM 转换为速度原始值."""
    if firmware == FirmwareType.X_FIRMWARE:
        return x_rpm_to_speed_raw(rpm)
    return emm_rpm_to_speed_raw(rpm)


def signed_position_deg(sign: int, raw: int, firmware: FirmwareType) -> float:
    """带符号位置 → 度."""
    return signed_from_sign_byte(sign, 1) * position_raw_to_deg(raw, firmware)


def signed_error_deg(sign: int, raw: int, firmware: FirmwareType) -> float:
    """带符号位置误差 → 度."""
    return signed_from_sign_byte(sign, 1) * error_raw_to_deg(raw, firmware)


def signed_speed_rpm(sign: int, raw: int, firmware: FirmwareType) -> float:
    """带符号速度 → RPM."""
    return signed_from_sign_byte(sign, 1) * speed_raw_to_rpm(raw, firmware)
