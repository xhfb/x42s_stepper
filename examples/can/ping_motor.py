#!/usr/bin/env python3
"""最小通讯测试：对指定地址发送读版本 (0x1F)."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# 允许直接从仓库根运行
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from x42s_stepper import CanBusError, CanTransport, X42SDevice  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="X42S CAN ping (读版本)")
    parser.add_argument("--channel", default="can0")
    parser.add_argument(
        "--bustype",
        default="socketcan",
        help="python-can 接口类型，默认 socketcan；模拟器用 virtual",
    )
    parser.add_argument(
        "--addrs",
        default="1,2",
        help="逗号分隔的电机地址，默认 1,2",
    )
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument(
        "--loopback-check",
        action="store_true",
        help="仅检查 can0 是否 UP 且非 LOOPBACK（不访问电机）",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(message)s",
    )

    if args.loopback_check:
        import subprocess

        out = subprocess.check_output(
            ["ip", "-details", "link", "show", args.channel], text=True
        )
        if "state DOWN" in out or "state UNKNOWN" in out and "UP" not in out:
            # ip 输出含 <...,UP,...>
            pass
        if f"{args.channel}:" not in out:
            print(f"[FAIL] 无接口 {args.channel}")
            return 2
        if "UP" not in out.split("\n")[0]:
            print(f"[FAIL] {args.channel} 未 UP，请运行 examples/can/bringup_can0.sh")
            return 2
        if "LOOPBACK" in out:
            print(f"[FAIL] {args.channel} 仍为 LOOPBACK，请重新 bringup")
            return 2
        print(f"[OK] {args.channel} UP, classic CAN, no LOOPBACK")
        return 0

    addrs = [int(x.strip()) for x in args.addrs.split(",") if x.strip()]
    ok_all = True

    with CanTransport(channel=args.channel, bustype=args.bustype, timeout=args.timeout) as bus:
        for addr in addrs:
            motor = X42SDevice(bus, address=addr, auto_test=False)
            try:
                ver = motor.ping()
                print(
                    f"[OK] addr={addr} fw={ver.firmware_version_str} "
                    f"hw={ver.hw_type_str} raw_fw={ver.firmware_version}"
                )
            except CanBusError as exc:
                print(f"[FAIL] addr={addr}: {exc}")
                ok_all = False
            except OSError as exc:
                print(f"[FAIL] addr={addr}: CAN 接口错误: {exc}")
                print("提示: 先运行 examples/can/bringup_can0.sh")
                return 2

    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
