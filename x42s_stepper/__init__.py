"""ZDT X42S dual-firmware (X/Emm/Turbo) closed-loop stepper driver."""

from .commands.base import CommandError, FirmwareCapabilityError
from .configs import (
    Address,
    ChecksumMode,
    ControlMode,
    Direction,
    FirmwareType,
    HomingMode,
    LockParamLevel,
    MotionMode,
    MotorType,
    StoreFlag,
    SyncFlag,
)
from .device import X42SDevice
from .transport.base import Transport, TransportError
from .transport.serial import SerialTransport
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
    MAX_SPEED_RAW_X,
    MAX_SPEED_RPM,
    MAX_SPEED_RPM_X,
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

__version__ = "0.2.0"

__all__ = [
    "X42SDevice",
    "__version__",
    "CommandError",
    "FirmwareCapabilityError",
    "Transport",
    "TransportError",
    "SerialTransport",
    "CanTransport",
    "CanBus",
    "CanBusError",
    "Address",
    "ChecksumMode",
    "ControlMode",
    "Direction",
    "FirmwareType",
    "HomingMode",
    "LockParamLevel",
    "MotionMode",
    "MotorType",
    "StoreFlag",
    "SyncFlag",
    "DeviceParams",
    "EmmJogParams",
    "EmmPositionParams",
    "EmmFastPositionParams",
    "XJogParams",
    "XJogCurrentLimitParams",
    "XTorqueParams",
    "XTorqueLimitedParams",
    "XPositionDirectParams",
    "XPositionDirectLimitedParams",
    "XPositionTrapParams",
    "XPositionTrapLimitedParams",
    "XFastPositionParams",
    "HomingParams",
    "HomingStatus",
    "VersionParams",
    "MotorRHParams",
    "EmmPIDParams",
    "XPIDParams",
    "MotorStatus",
    "IOStatus",
    "HomeMotorStatus",
    "OptionStatus",
    "ProtectionThreshold",
    "DMX512Params",
    "EmmSystemStatusParams",
    "XSystemStatusParams",
    "EmmConfigParams",
    "XConfigParams",
    "EmmAutoRunParams",
    "XAutoRunParams",
    "MAX_SPEED_RPM",
    "MAX_SPEED_RPM_X",
    "MAX_SPEED_RAW_X",
]


def __getattr__(name: str):
    """延迟导出 CanTransport，避免未安装 python-can 时影响串口导入."""
    if name in ("CanTransport", "CanBus", "CanBusError"):
        from .transport.can import CanBus, CanBusError, CanTransport

        return {
            "CanTransport": CanTransport,
            "CanBus": CanBus,
            "CanBusError": CanBusError,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

