#!/usr/bin/env python3
"""Emm 电机简易模拟器（python-can virtual），用于无硬件时验收驱动."""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import can  # noqa: E402

from x42s_stepper.configs import add_checksum  # noqa: E402
from x42s_stepper.transport.can_codec import (  # noqa: E402
    make_can_id,
    pack_logical_frame,
    parse_can_id,
    reassemble_logical_frame,
)


def handle(addr: int, logical: bytes):
    """根据请求逻辑帧生成应答（含地址与校验）."""
    if len(logical) < 2 or logical[0] != addr:
        return None
    code = logical[1]

    if code == 0x1F:  # 版本: FW=107 → V1.0.7, HW=X42
        # fw=107 → 00 6B; hw: series0 type3 ver1 → 0x0301
        return add_checksum(bytes([addr, 0x1F, 0x00, 0x6B, 0x03, 0x01]))
    if code == 0xF3:  # 使能
        return add_checksum(bytes([addr, 0xF3, 0x02]))
    if code == 0xFE:  # 急停
        return add_checksum(bytes([addr, 0xFE, 0x02]))
    if code == 0xF6:  # jog
        return add_checksum(bytes([addr, 0xF6, 0x02]))
    if code == 0xFD:  # position
        return add_checksum(bytes([addr, 0xFD, 0x02]))
    if code == 0x35:  # speed 0
        return add_checksum(bytes([addr, 0x35, 0x00, 0x00, 0x00]))
    if code == 0x36:  # position 0
        return add_checksum(bytes([addr, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00]))
    if code == 0x3A:  # status: enabled + reached
        return add_checksum(bytes([addr, 0x3A, 0x03]))
    return None


def pack_reply(logical: bytes):
    return pack_logical_frame(logical)


def run_sim(channel: str, addrs, bustype: str) -> None:
    bus = can.Bus(channel=channel, interface=bustype, fd=False)
    logging.info("sim listening on %s://%s addrs=%s", bustype, channel, addrs)
    # 按地址缓存未完成分包
    pending = {a: {} for a in addrs}

    try:
        while True:
            msg = bus.recv(timeout=0.5)
            if msg is None or not msg.is_extended_id:
                continue
            addr, packet = parse_can_id(msg.arbitration_id)
            if addr not in pending:
                continue
            data = bytes(msg.data)
            if not data:
                continue
            pending[addr][packet] = data
            # 尝试拼装：直到末字节为 0x6B
            try:
                frames = [
                    (make_can_id(addr, p), d)
                    for p, d in sorted(pending[addr].items())
                ]
                logical = reassemble_logical_frame(addr, frames)
            except ValueError:
                continue
            if not logical.endswith(b"\x6b"):
                continue
            pending[addr].clear()
            reply = handle(addr, logical)
            if reply is None:
                logging.debug("no handler for %s", logical.hex())
                continue
            logging.info("REQ %s -> REP %s", logical.hex(" "), reply.hex(" "))
            for can_id, pdata in pack_reply(reply):
                bus.send(
                    can.Message(
                        arbitration_id=can_id,
                        data=pdata,
                        is_extended_id=True,
                        is_fd=False,
                    )
                )
    finally:
        bus.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="X42S Emm CAN 模拟器")
    parser.add_argument("--channel", default="x42s-sim")
    parser.add_argument("--bustype", default="virtual")
    parser.add_argument("--addrs", default="1,2")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    addrs = [int(x) for x in args.addrs.split(",") if x.strip()]
    run_sim(args.channel, addrs, args.bustype)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
