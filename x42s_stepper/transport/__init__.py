"""传输层：SerialTransport / CanTransport."""

from .base import Transport, TransportError
from .serial import SerialTransport

__all__ = [
    "Transport",
    "TransportError",
    "SerialTransport",
    "CanTransport",
    "CanBus",
    "CanBusError",
]


def __getattr__(name: str):
    """延迟导入 CanTransport，避免未装 python-can 时影响串口路径."""
    if name in ("CanTransport", "CanBus", "CanBusError"):
        from .can import CanBus, CanBusError, CanTransport

        return {
            "CanTransport": CanTransport,
            "CanBus": CanBus,
            "CanBusError": CanBusError,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
