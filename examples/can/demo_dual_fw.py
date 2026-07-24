#!/usr/bin/env python3
"""双固件切换验收：默认只动 addr=1，结束后强制切回 Emm + store=True.

步骤:
  1. detect 当前固件
  2. stop → switch X → detect
  3. X: enable → jog → stop → trap 位置 → 短 torque → stop
  4. switch Emm → detect
  5. Emm jog 冒烟

任何异常也会尽量切回 Emm。
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
import traceback
from pathlib import Path
from typing import Iterable, List

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from x42s_stepper import (  # noqa: E402
    CanTransport,
    Direction,
    FirmwareCapabilityError,
    FirmwareType,
    MotionMode,
    X42SDevice,
)

logger = logging.getLogger(__name__)


def _restore_emm(motors: Iterable[X42SDevice]) -> None:
    for m in motors:
        try:
            m.stop()
        except Exception:
            pass
        try:
            ok = m.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
            print(f"  restore Emm addr={m.address}: {'OK' if ok else 'FAIL'} → {m.firmware_type}")
        except Exception as exc:
            print(f"  restore Emm addr={m.address} 异常: {exc}")


def run_one(m: X42SDevice) -> None:
    print(f"=== addr={m.address} ===")
    v = m.get_version()
    print(f"  version={v.firmware_version_str} hw={v.hw_type_str}")

    fw0 = m.detect_firmware()
    print(f"  [1] detect → {fw0}")
    try:
        opt = m.get_option_status()
        print(
            f"      option_status FwType(不可靠)={opt.firmware_type} "
            f"raw=0x{opt.raw:04X}"
        )
    except Exception as exc:
        print(f"      option_status 跳过: {exc}")

    try:
        m.stop()
    except Exception:
        pass
    print("  [2] switch → X_FIRMWARE")
    assert m.switch_firmware(FirmwareType.X_FIRMWARE, store=True)
    fw_x = m.detect_firmware()
    print(f"      detect → {fw_x}")
    assert fw_x == FirmwareType.X_FIRMWARE, fw_x

    print("  [3] X motion: enable / jog / trap / torque")
    assert m.enable()
    assert m.jog(80, acceleration=1000)
    time.sleep(0.8)
    assert m.stop()
    time.sleep(0.2)

    p0 = m.get_realtime_position()
    assert m.move_position(
        angle_deg=45,
        speed_rpm=200,
        mode="trap",
        motion_mode=MotionMode.RELATIVE_LAST,
    )
    reached = m.wait_position_reached(timeout=10)
    p1 = m.get_realtime_position()
    print(f"      trap 45° reached={reached} Δ={p1 - p0:.1f}°")
    assert reached, "X trap 未到位"

    assert m.torque(current_ma=300, slope_ma_s=200)
    time.sleep(0.3)
    assert m.stop()
    time.sleep(0.2)

    # Emm 下 torque 应门控失败（切回后测）
    print("  [4] switch → EMM_FIRMWARE")
    assert m.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
    fw_e = m.detect_firmware()
    print(f"      detect → {fw_e}")
    assert fw_e == FirmwareType.EMM_FIRMWARE, fw_e

    print("  [5] Emm jog smoke")
    assert m.enable()
    assert m.jog(60, acceleration=10)
    time.sleep(0.6)
    assert m.stop()

    try:
        m.torque(current_ma=200)
        raise AssertionError("Emm 下 torque 应抛 FirmwareCapabilityError")
    except FirmwareCapabilityError:
        print("      Emm torque 门控 OK")

    m.disable()
    print(f"  addr={m.address} 验收通过")


def main() -> int:
    parser = argparse.ArgumentParser(description="X42S 双固件切换实机验收")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bustype", default="socketcan")
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument(
        "--addrs",
        default="1",
        help="逗号分隔地址，默认只测 1（降低双机风险）",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(message)s",
    )

    addrs: List[int] = [int(x) for x in args.addrs.split(",") if x.strip()]
    motors: List[X42SDevice] = []
    rc = 0

    with CanTransport(channel=args.channel, bustype=args.bustype, timeout=args.timeout) as bus:
        try:
            for addr in addrs:
                m = X42SDevice(bus, address=addr, auto_test=False)
                motors.append(m)
                run_one(m)
        except Exception:
            rc = 1
            traceback.print_exc()
        finally:
            print("--- 强制恢复 Emm ---")
            _restore_emm(motors)

    return rc


if __name__ == "__main__":
    sys.exit(main())
