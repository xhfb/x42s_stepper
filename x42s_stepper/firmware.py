"""X42S 固件能力配置."""

from dataclasses import dataclass

from .commands.base import FirmwareCapabilityError
from .configs import FirmwareType

__all__ = [
    "FirmwareCapabilityError",
    "FirmwareProfile",
    "XProfile",
    "EmmProfile",
    "EmmTurboProfile",
    "get_profile",
]


@dataclass
class FirmwareProfile:
    """固件能力描述."""

    firmware_type: FirmwareType
    supports_torque: bool
    supports_position_direct: bool  # X only
    supports_current_limit_motion: bool  # X C5/C6/CB/CD
    supports_fast_position: bool = True  # F1/FC, 另受固件版本门控
    is_turbo: bool = False


class XProfile(FirmwareProfile):
    """X 固件能力."""

    def __init__(self):
        super().__init__(
            FirmwareType.X_FIRMWARE,
            supports_torque=True,
            supports_position_direct=True,
            supports_current_limit_motion=True,
            supports_fast_position=True,
            is_turbo=False,
        )


class EmmProfile(FirmwareProfile):
    """Emm 固件能力."""

    def __init__(self):
        super().__init__(
            FirmwareType.EMM_FIRMWARE,
            supports_torque=False,
            supports_position_direct=False,
            supports_current_limit_motion=False,
            supports_fast_position=True,
            is_turbo=False,
        )


class EmmTurboProfile(FirmwareProfile):
    """Emm 狂暴模式 (配置布局同 Emm)."""

    def __init__(self):
        super().__init__(
            FirmwareType.EMM_TURBO,
            supports_torque=False,
            supports_position_direct=False,
            supports_current_limit_motion=False,
            supports_fast_position=True,
            is_turbo=True,
        )


def get_profile(fw: FirmwareType) -> FirmwareProfile:
    """按固件类型返回能力配置."""
    if fw == FirmwareType.X_FIRMWARE:
        return XProfile()
    if fw == FirmwareType.EMM_FIRMWARE:
        return EmmProfile()
    if fw == FirmwareType.EMM_TURBO:
        return EmmTurboProfile()
    raise FirmwareCapabilityError(f"未知固件类型: {fw}")
