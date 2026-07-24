"""串口 Transport / 构造兼容性（无硬件）."""

from unittest.mock import MagicMock

from serial import Serial

from x42s_stepper import FirmwareType, SerialTransport, X42SDevice
from x42s_stepper.parameters import DeviceParams


def _mock_serial(response: bytes) -> MagicMock:
    ser = MagicMock(spec=Serial)
    ser.timeout = 0.1
    ser.in_waiting = 0
    data = list(response)

    def read(n=1):
        out = bytearray()
        for _ in range(n):
            if not data:
                return b""
            out.append(data.pop(0))
        return bytes(out)

    ser.read.side_effect = read
    return ser


def test_serial_transport_request_skips_junk_and_checks():
    # 前缀杂字节 + 完整应答
    resp = bytes.fromhex("AA BB 01 F3 02 6B")
    ser = _mock_serial(resp)
    tr = SerialTransport(ser)
    out = tr.request(bytes.fromhex("01 F3 AB 01 00 6B"), expected_len=4)
    assert out == bytes.fromhex("01 F3 02 6B")
    ser.write.assert_called()


def test_x42sdevice_accepts_serial_positional():
    # 版本应答 7 字节（auto_test）
    resp = bytes.fromhex("01 1F 00 6B 03 01 6B")
    ser = _mock_serial(resp)
    # 跳过 detect：指定固件
    m = X42SDevice(
        ser,
        address=1,
        firmware_type=FirmwareType.EMM_FIRMWARE,
        auto_test=True,
    )
    assert isinstance(m.transport, SerialTransport)
    assert m.device_params.serial_connection is ser
    assert m.address == 1


def test_device_params_transport_field():
    ser = MagicMock(spec=Serial)
    tr = SerialTransport(ser)
    p = DeviceParams(transport=tr, address=2)
    assert p.serial_connection is ser
    assert p.transport is tr
