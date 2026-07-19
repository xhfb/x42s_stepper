"""X42S 双固件驱动库冒烟测试.

用法:
  python full_test.py COM9
  python full_test.py COM9 --move
  python full_test.py COM9 --firmware
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback
from typing import List, Optional

from serial import Serial

from x42s_stepper import Direction, FirmwareType, MotionMode, X42SDevice


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description="X42S stepper smoke test")
    p.add_argument("port", nargs="?", default="COM9")
    p.add_argument("--addr", type=int, default=1)
    p.add_argument("--move", action="store_true")
    p.add_argument("--firmware", action="store_true", help="固件切换往返")
    args = p.parse_args(argv)

    print(f"port={args.port} addr={args.addr} move={args.move} firmware={args.firmware}")
    ser = Serial(args.port, 115200, timeout=0.15)
    fails = 0
    original: Optional[FirmwareType] = None

    try:
        motor = X42SDevice(ser, address=args.addr)
        original = motor.detect_firmware()
        ver = motor.get_version()
        print(
            f"fw={original.name} ver={ver.firmware_version_str} "
            f"hw={ver.hw_type_str} fast={motor.supports_fast_position}"
        )
        print("Vbus", motor.get_bus_voltage(), "mV")
        print("config", type(motor.get_config()).__name__)

        if args.move:
            assert motor.enable()
            assert motor.jog(speed_rpm=60, direction=Direction.CW)
            time.sleep(0.4)
            print("speed", motor.get_realtime_speed())
            assert motor.stop()

            if motor.firmware_type == FirmwareType.X_FIRMWARE:
                ok = motor.move_position(
                    angle_deg=45, speed_rpm=150, mode="direct",
                    motion_mode=MotionMode.RELATIVE_CURRENT,
                )
            else:
                ok = motor.move_position(
                    pulses=800, speed_rpm=150, acceleration=15,
                    motion_mode=MotionMode.RELATIVE_CURRENT,
                )
            assert ok and motor.wait_position_reached(timeout=8)
            print("position ok")

            if motor.supports_fast_position:
                assert motor.configure_fast_position(speed_rpm=200, acceleration=20)
                if motor.firmware_type == FirmwareType.X_FIRMWARE:
                    assert motor.move_fast_degrees(30)
                else:
                    assert motor.move_fast_pulses(400)
                motor.wait_position_reached(timeout=8)
                print("fast position ok")

            assert motor.disable()

        if args.firmware:
            other = (
                FirmwareType.X_FIRMWARE
                if original != FirmwareType.X_FIRMWARE
                else FirmwareType.EMM_FIRMWARE
            )
            motor.stop()
            assert motor.switch_firmware(other, store=True), f"to {other}"
            print("switched to", motor.firmware_type.name)
            assert motor.switch_firmware(original, store=True), f"restore {original}"
            print("restored", motor.firmware_type.name)

    except Exception:
        traceback.print_exc()
        fails += 1
    finally:
        try:
            if original is not None:
                m = X42SDevice(ser, address=args.addr, auto_test=False)
                m.detect_firmware()
                m.stop()
                m.switch_firmware(original, store=True)
                m.disable()
        except Exception as e:
            print("restore warn:", e)
        ser.close()

    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
