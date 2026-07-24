#!/usr/bin/env bash
# 拉起 RDK X5 板载 can0：经典 CAN @ 500 kbps（X42S 非 CAN FD）
set -euo pipefail

CHANNEL="${1:-can0}"
BITRATE="${2:-500000}"

if [[ "$(id -u)" -ne 0 ]]; then
  exec sudo "$0" "$@"
fi

ip link set "$CHANNEL" down 2>/dev/null || true

# 先配置 type（必须在 down 时），明确关闭 loopback / listen-only，勿开 fd
ip link set "$CHANNEL" type can bitrate "$BITRATE" restart-ms 100 \
  loopback off listen-only off

ip link set "$CHANNEL" up

echo "==> $CHANNEL up @ ${BITRATE} bps (classic CAN, no FD)"
ip -details -statistics link show "$CHANNEL"

if ip -details link show "$CHANNEL" | grep -qE 'LOOPBACK|LISTEN-ONLY'; then
  echo "警告: 接口仍处于 LOOPBACK 或 LISTEN-ONLY" >&2
  exit 1
fi
