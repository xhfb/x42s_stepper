#!/usr/bin/env python3
"""双机同步运动测试：sync=True 缓存 + 广播 FF66 齐发."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from x42s_stepper import CanTransport, Direction, MotionMode, X42SDevice  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="X42S 双机同步 FF66 测试")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bustype", default="socketcan")
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(message)s",
    )

    with CanTransport(channel=args.channel, bustype=args.bustype, timeout=args.timeout) as bus:
        m1 = X42SDevice(bus, address=1, auto_test=False)
        m2 = X42SDevice(bus, address=2, auto_test=False)

        for m in (m1, m2):
            v = m.get_version()
            print(f"addr={m.address} {v.firmware_version_str} pos={m.get_realtime_position():.1f}°")
            assert m.enable()

        # --- 同步速度：反向齐转 ---
        print("--- sync jog: m1 CW / m2 CCW 50RPM ---")
        assert m1.jog(50, Direction.CW, acceleration=10, sync=True)
        assert m2.jog(50, Direction.CCW, acceleration=10, sync=True)
        print("  cached; speeds before trigger:", m1.get_realtime_speed(), m2.get_realtime_speed())
        t0 = time.monotonic()
        assert X42SDevice.sync_move(bus)
        print(f"  FF66 OK, trigger at t=0")
        time.sleep(0.15)
        s1, s2 = m1.get_realtime_speed(), m2.get_realtime_speed()
        p1a, p2a = m1.get_realtime_position(), m2.get_realtime_position()
        print(f"  t=0.15s spd=({s1:.0f},{s2:.0f}) pos=({p1a:.1f},{p2a:.1f})")
        time.sleep(1.0)
        s1, s2 = m1.get_realtime_speed(), m2.get_realtime_speed()
        p1b, p2b = m1.get_realtime_position(), m2.get_realtime_position()
        print(f"  t=1.15s spd=({s1:.0f},{s2:.0f}) pos=({p1b:.1f},{p2b:.1f})")
        assert m1.stop()
        assert m2.stop()
        time.sleep(0.3)

        # --- 同步位置：各相对 +60° ---
        print("--- sync position: both +60° ---")
        p1_0, p2_0 = m1.get_realtime_position(), m2.get_realtime_position()
        assert m1.move_position(
            angle_deg=60, speed_rpm=100, acceleration=15,
            motion_mode=MotionMode.RELATIVE_CURRENT, sync=True,
        )
        assert m2.move_position(
            angle_deg=60, speed_rpm=100, acceleration=15,
            motion_mode=MotionMode.RELATIVE_CURRENT, sync=True,
        )
        print(f"  cached at pos=({p1_0:.1f},{p2_0:.1f}); triggering...")
        assert X42SDevice.sync_move(bus)
        ok1 = m1.wait_position_reached(timeout=8)
        ok2 = m2.wait_position_reached(timeout=8)
        p1_1, p2_1 = m1.get_realtime_position(), m2.get_realtime_position()
        print(
            f"  reached=({ok1},{ok2}) "
            f"delta=({p1_1 - p1_0:.1f},{p2_1 - p2_0:.1f})"
        )

        assert m1.disable()
        assert m2.disable()
        print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
