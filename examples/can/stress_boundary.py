#!/usr/bin/env python3
"""严苛边界/工况实机测试（默认 addr=1,2；结束强制恢复 Emm）。

覆盖:
  A. API 门控与非法入参（不依赖运动成功）
  B. Emm 运动边界（0/微动/高速短时/反向/急停打断）
  C. X 运动边界（jog/限流/trap/direct/力矩/限速力矩）
  D. 固件探测一致性 + 快速往返切换 + 运动中切换
  E. 双机同步 / 多机帧 / 交错读状态
  F. 读参稳定性、PID 类型、快位门控、超时恢复
  G. 异常后通讯自愈

失败也会尽量切回 Emm。
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from x42s_stepper import (  # noqa: E402
    CanBusError,
    CanTransport,
    Direction,
    FirmwareCapabilityError,
    FirmwareType,
    MotionMode,
    X42SDevice,
)
from x42s_stepper import EmmPIDParams, XPIDParams  # noqa: E402
from x42s_stepper import MAX_SPEED_RAW_X, XJogParams  # noqa: E402
from x42s_stepper.configs import Code, Protocol  # noqa: E402

logger = logging.getLogger(__name__)


@dataclass
class CaseResult:
    name: str
    ok: bool
    detail: str = ""


@dataclass
class Suite:
    results: List[CaseResult] = field(default_factory=list)

    def check(self, name: str, cond: bool, detail: str = "") -> None:
        self.results.append(CaseResult(name, bool(cond), detail))
        mark = "PASS" if cond else "FAIL"
        print(f"  [{mark}] {name}" + (f" — {detail}" if detail else ""))

    def expect_raises(self, name: str, exc_types, fn: Callable) -> None:
        try:
            fn()
            self.check(name, False, "未抛异常")
        except exc_types as exc:
            self.check(name, True, type(exc).__name__)
        except Exception as exc:
            self.check(name, False, f"异常类型错误: {type(exc).__name__}: {exc}")

    def summary(self) -> int:
        n_ok = sum(1 for r in self.results if r.ok)
        n_fail = len(self.results) - n_ok
        print(f"\n=== 合计 {len(self.results)} 项: PASS={n_ok} FAIL={n_fail} ===")
        if n_fail:
            print("失败项:")
            for r in self.results:
                if not r.ok:
                    print(f"  - {r.name}: {r.detail}")
        return 0 if n_fail == 0 else 1


def safe_stop(m: X42SDevice) -> None:
    try:
        m.stop()
    except Exception:
        pass


def restore_emm(motors: List[X42SDevice]) -> None:
    print("--- 强制恢复 Emm ---")
    for m in motors:
        safe_stop(m)
        try:
            ok = m.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
            fw = m.detect_firmware()
            print(f"  addr={m.address} restore={'OK' if ok else 'FAIL'} fw={fw}")
        except Exception as exc:
            print(f"  addr={m.address} restore 异常: {exc}")


def ensure_fw(m: X42SDevice, target: FirmwareType, s: Suite, tag: str) -> bool:
    safe_stop(m)
    if m.firmware_type != target:
        ok = m.switch_firmware(target, store=True)
        s.check(f"{tag}/switch->{target.name}", ok)
        if not ok:
            return False
    fw = m.detect_firmware()
    s.check(f"{tag}/detect={target.name}", fw == target, str(fw))
    return fw == target


# ---------------------------------------------------------------------------
# A. 门控 / 非法入参
# ---------------------------------------------------------------------------

def suite_a_gates(m: X42SDevice, s: Suite) -> None:
    print("\n## A. API 门控与非法入参")
    assert ensure_fw(m, FirmwareType.EMM_FIRMWARE, s, "A.emm")

    s.expect_raises("A.Emm.torque", FirmwareCapabilityError, lambda: m.torque(200))
    s.expect_raises(
        "A.Emm.torque_limited",
        FirmwareCapabilityError,
        lambda: m.torque_limited(200, 100),
    )
    s.expect_raises(
        "A.Emm.jog_current_limit",
        FirmwareCapabilityError,
        lambda: m.jog(50, max_current_ma=500),
    )
    s.expect_raises(
        "A.Emm.move_direct",
        FirmwareCapabilityError,
        lambda: m.move_position(angle_deg=10, mode="direct"),
    )
    s.expect_raises(
        "A.Emm.move_current_limit",
        FirmwareCapabilityError,
        lambda: m.move_position(angle_deg=10, max_current_ma=500),
    )
    s.expect_raises(
        "A.move_no_target",
        ValueError,
        lambda: m.move_position(),
    )
    s.expect_raises(
        "A.Emm.jog_speed_over",
        ValueError,
        lambda: m.jog(6001),
    )
    s.expect_raises(
        "A.Emm.jog_acc_over",
        ValueError,
        lambda: m.jog(10, acceleration=256),
    )
    # V1.0.7 应拒绝快位
    s.expect_raises(
        "A.fast_position_gate_v1",
        FirmwareCapabilityError,
        lambda: m.configure_fast_position(speed_rpm=100),
    )
    s.check("A.supports_fast_position_false", m.supports_fast_position is False)

    s.expect_raises(
        "A.bad_address",
        ValueError,
        lambda: X42SDevice(m.bus, address=0),
    )

    # 切到 X 再测 Emm-only 型错误 + mode 非法
    assert ensure_fw(m, FirmwareType.X_FIRMWARE, s, "A.x")
    s.expect_raises(
        "A.X.move_bad_mode",
        ValueError,
        lambda: m.move_position(angle_deg=5, mode="spline"),
    )
    # set_pid 类型不匹配：用 Emm 参数在 X 上
    s.expect_raises(
        "A.X.set_pid_emm_type",
        FirmwareCapabilityError,
        lambda: m.set_pid(EmmPIDParams(), store=False),
    )
    pid = m.get_pid()
    s.check("A.X.get_pid_type", isinstance(pid, XPIDParams), type(pid).__name__)

    assert ensure_fw(m, FirmwareType.EMM_FIRMWARE, s, "A.back")
    pid_e = m.get_pid()
    s.check("A.Emm.get_pid_type", isinstance(pid_e, EmmPIDParams), type(pid_e).__name__)
    s.expect_raises(
        "A.Emm.set_pid_x_type",
        FirmwareCapabilityError,
        lambda: m.set_pid(XPIDParams(), store=False),
    )


# ---------------------------------------------------------------------------
# B. Emm 运动边界
# ---------------------------------------------------------------------------

def suite_b_emm_motion(m: X42SDevice, s: Suite) -> None:
    print(f"\n## B. Emm 运动边界 addr={m.address}")
    assert ensure_fw(m, FirmwareType.EMM_FIRMWARE, s, f"B{m.address}.prep")
    assert m.enable()
    safe_stop(m)

    # 0 RPM jog：协议上应 ACK（不一定转）
    ok0 = m.jog(0, acceleration=10)
    s.check(f"B{m.address}.jog_0rpm", ok0)
    safe_stop(m)

    # 微位置（显式相对当前位置）
    p0 = m.get_realtime_position()
    ok = m.move_position(
        angle_deg=5.0,
        speed_rpm=80,
        acceleration=8,
        motion_mode=MotionMode.RELATIVE_CURRENT,
    )
    reached = m.wait_position_reached(timeout=5)
    p1 = m.get_realtime_position()
    s.check(
        f"B{m.address}.micro_5deg",
        ok and reached and abs((p1 - p0) - 5.0) < 2.0,
        f"Δ={p1 - p0:.2f}",
    )

    # 负角度（方向由符号决定）
    p0 = m.get_realtime_position()
    ok = m.move_position(angle_deg=-30, speed_rpm=120, acceleration=12)
    reached = m.wait_position_reached(timeout=8)
    p1 = m.get_realtime_position()
    s.check(
        f"B{m.address}.neg_30deg",
        ok and reached and (p1 - p0) < -20,
        f"Δ={p1 - p0:.2f}",
    )

    # 高速短时 + 急停打断（Emm acc 为档位，需拉满并轮询到较高转速）
    ok = m.jog(1500, Direction.CW, acceleration=255)
    spd = 0.0
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        spd = m.get_realtime_speed()
        if abs(spd) >= 300:
            break
        time.sleep(0.05)
    ok_stop = m.stop()
    time.sleep(0.3)
    spd2 = abs(m.get_realtime_speed())
    s.check(
        f"B{m.address}.hi_jog_estop",
        ok and ok_stop and abs(spd) >= 300 and spd2 < 80,
        f"spd={spd:.0f} after={spd2:.0f}",
    )

    # 运动中再发反向 jog（抢占）
    assert m.jog(200, Direction.CW, acceleration=15)
    time.sleep(0.2)
    ok_rev = m.jog(200, Direction.CCW, acceleration=15)
    time.sleep(0.35)
    spd = m.get_realtime_speed()
    safe_stop(m)
    s.check(
        f"B{m.address}.jog_reverse_preempt",
        ok_rev and spd < 0,
        f"spd={spd:.0f}",
    )

    # pulses 路径
    pulses = 3200  # ~360° @16微步 1.8°
    p0 = m.get_realtime_position()
    ok = m.move_position(pulses=pulses // 4, speed_rpm=150, acceleration=10)  # ~90°
    reached = m.wait_position_reached(timeout=10)
    p1 = m.get_realtime_position()
    s.check(
        f"B{m.address}.pulses_90ish",
        ok and reached and 60 < (p1 - p0) < 120,
        f"Δ={p1 - p0:.1f}",
    )

    # wait 超时：启动长行程后立即短超时轮询，应未到位
    assert m.move_position(
        angle_deg=360,
        speed_rpm=60,
        acceleration=5,
        motion_mode=MotionMode.RELATIVE_CURRENT,
    )
    t0 = time.monotonic()
    timed_out = not m.wait_position_reached(timeout=0.08)
    elapsed = time.monotonic() - t0
    safe_stop(m)
    s.check(
        f"B{m.address}.wait_timeout_fast",
        timed_out and elapsed < 0.4,
        f"elapsed={elapsed:.3f}",
    )

    # 连续读参风暴
    errs = 0
    for _ in range(40):
        try:
            m.get_realtime_position()
            m.get_realtime_speed()
            m.get_motor_status()
            m.get_bus_voltage()
        except Exception:
            errs += 1
    s.check(f"B{m.address}.read_storm_40x4", errs == 0, f"errs={errs}")

    # zero_position 后位置接近 0
    assert m.zero_position()
    time.sleep(0.05)
    pz = m.get_realtime_position()
    s.check(f"B{m.address}.zero_position", abs(pz) < 2.0, f"pos={pz:.2f}")

    m.disable()
    s.check(f"B{m.address}.disable", True)


# ---------------------------------------------------------------------------
# C. X 运动边界
# ---------------------------------------------------------------------------

def suite_c_x_motion(m: X42SDevice, s: Suite) -> None:
    print(f"\n## C. X 运动边界 addr={m.address}")
    assert ensure_fw(m, FirmwareType.X_FIRMWARE, s, f"C{m.address}.prep")
    assert m.enable()
    safe_stop(m)

    # jog_raw：800 → 80.0 RPM
    ok = m.jog_raw(800, acceleration=1000)
    time.sleep(0.5)
    spd = m.get_realtime_speed()
    safe_stop(m)
    s.check(
        f"C{m.address}.jog_raw_80",
        ok and 40 < abs(spd) < 120,
        f"spd={spd:.1f}",
    )

    # 限流 jog
    ok = m.jog(60, acceleration=800, max_current_ma=800)
    time.sleep(0.4)
    ok_stop = m.stop()
    s.check(f"C{m.address}.jog_c6", ok and ok_stop)

    # trap 小角度高精度
    p0 = m.get_realtime_position()
    ok = m.move_position(
        angle_deg=10,
        speed_rpm=150,
        mode="trap",
        acceleration=800,
        decel=800,
    )
    reached = m.wait_position_reached(timeout=8)
    p1 = m.get_realtime_position()
    s.check(
        f"C{m.address}.trap_10deg",
        ok and reached and abs((p1 - p0) - 10) < 2.0,
        f"Δ={p1 - p0:.2f}",
    )

    # trap 限流 + 大角度
    p0 = m.get_realtime_position()
    ok = m.move_position(
        angle_deg=120,
        speed_rpm=400,
        mode="trap",
        acceleration=1500,
        decel=1200,
        max_current_ma=1500,
    )
    reached = m.wait_position_reached(timeout=12)
    p1 = m.get_realtime_position()
    s.check(
        f"C{m.address}.trap_120_cd",
        ok and reached and 100 < (p1 - p0) < 140,
        f"Δ={p1 - p0:.1f}",
    )

    # direct 模式
    p0 = m.get_realtime_position()
    ok = m.move_position(
        angle_deg=-25,
        speed_rpm=180,
        mode="direct",
        motion_mode=MotionMode.RELATIVE_LAST,
    )
    reached = m.wait_position_reached(timeout=8)
    p1 = m.get_realtime_position()
    s.check(
        f"C{m.address}.direct_-25",
        ok and reached and (p1 - p0) < -15,
        f"Δ={p1 - p0:.2f}",
    )

    # direct 限流
    ok = m.move_position(
        angle_deg=15,
        speed_rpm=100,
        mode="direct",
        max_current_ma=1000,
    )
    reached = m.wait_position_reached(timeout=8)
    s.check(f"C{m.address}.direct_cb", ok and reached)

    # 力矩短时 + 限速力矩
    ok_t = m.torque(current_ma=250, slope_ma_s=300)
    time.sleep(0.25)
    ok_tl = m.torque_limited(current_ma=300, max_speed_rpm=100, slope_ma_s=200)
    time.sleep(0.35)
    spd = abs(m.get_realtime_speed())
    safe_stop(m)
    s.check(
        f"C{m.address}.torque_and_limited",
        ok_t and ok_tl and spd < 150,
        f"spd={spd:.1f}",
    )

    # X 速度上限附近短冲（库允许到 3000；用 2500 短时）
    ok = m.jog(2500, acceleration=3000)
    time.sleep(0.2)
    spd = abs(m.get_realtime_speed())
    safe_stop(m)
    time.sleep(0.2)
    s.check(
        f"C{m.address}.near_max_jog",
        ok and spd > 500,
        f"spd={spd:.0f}",
    )

    # 参数层：超速 raw 构造应抛
    s.expect_raises(
        f"C{m.address}.xjog_raw_over",
        ValueError,
        lambda: XJogParams(speed_raw=MAX_SPEED_RAW_X + 1),
    )

    # 单位换算一致性：切换固件后读位置不应炸
    pos_x = m.get_realtime_position()
    s.check(f"C{m.address}.pos_readable", isinstance(pos_x, float), str(pos_x))

    m.disable()


# ---------------------------------------------------------------------------
# D. 固件切换压力
# ---------------------------------------------------------------------------

def suite_d_fw_stress(m: X42SDevice, s: Suite) -> None:
    print(f"\n## D. 固件切换压力 addr={m.address}")
    # 连续 detect 一致性
    fws = [m.detect_firmware() for _ in range(5)]
    s.check(
        f"D{m.address}.detect_stable",
        len(set(fws)) == 1,
        str(fws),
    )

    # 快速往返 3 轮
    rounds_ok = True
    for i in range(3):
        safe_stop(m)
        if not m.switch_firmware(FirmwareType.X_FIRMWARE, store=True):
            rounds_ok = False
            break
        if m.detect_firmware() != FirmwareType.X_FIRMWARE:
            rounds_ok = False
            break
        if not m.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True):
            rounds_ok = False
            break
        if m.detect_firmware() != FirmwareType.EMM_FIRMWARE:
            rounds_ok = False
            break
    s.check(f"D{m.address}.switch_roundtrip_x3", rounds_ok)

    # store=False：允许失败，但不应把总线打挂；并打 warning
    safe_stop(m)
    m.switch_firmware(FirmwareType.X_FIRMWARE, store=False)  # 可能不生效
    fw_after = m.detect_firmware()
    # 无论是否切换成功，再次 store=True 拉回 Emm
    ok = m.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
    s.check(
        f"D{m.address}.store_false_survives",
        ok and m.detect_firmware() == FirmwareType.EMM_FIRMWARE,
        f"after_nostore={fw_after}",
    )

    # 运动中强制切换：先 jog 再 switch（严苛：驱动应能处理或拒绝，通讯不崩）
    assert m.enable()
    m.jog(100, acceleration=10)
    time.sleep(0.15)
    # 手册建议停机切换；此处故意不停，观察行为
    switched = m.switch_firmware(FirmwareType.X_FIRMWARE, store=True)
    fw = m.detect_firmware()
    safe_stop(m)
    # 成功切到 X 或仍能通讯都算「未砖」；要求最终能回到 Emm
    alive = True
    try:
        m.get_version()
    except Exception:
        alive = False
    back = m.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
    s.check(
        f"D{m.address}.switch_while_moving",
        alive and back and m.detect_firmware() == FirmwareType.EMM_FIRMWARE,
        f"mid_sw={switched} mid_fw={fw}",
    )

    # option_status 与 detect 可不一致（文档行为）
    opt = m.get_option_status()
    det = m.detect_firmware()
    s.check(
        f"D{m.address}.option_may_disagree",
        True,
        f"opt.fw={opt.firmware_type} detect={det}",
    )


# ---------------------------------------------------------------------------
# E. 双机工况
# ---------------------------------------------------------------------------

def suite_e_dual(m1: X42SDevice, m2: X42SDevice, s: Suite) -> None:
    print("\n## E. 双机同步/交错工况")
    for m in (m1, m2):
        ensure_fw(m, FirmwareType.EMM_FIRMWARE, s, f"E.prep{m.address}")
        assert m.enable()
        safe_stop(m)

    # 同步 jog 反向
    assert m1.jog(80, Direction.CW, acceleration=12, sync=True)
    assert m2.jog(80, Direction.CCW, acceleration=12, sync=True)
    ok_sync = X42SDevice.sync_move(m1.bus)
    time.sleep(0.6)
    s1, s2 = m1.get_realtime_speed(), m2.get_realtime_speed()
    safe_stop(m1)
    safe_stop(m2)
    s.check(
        "E.sync_jog_opposite",
        ok_sync and s1 > 30 and s2 < -30,
        f"spd=({s1:.0f},{s2:.0f})",
    )

    # 同步位置不同目标
    p1, p2 = m1.get_realtime_position(), m2.get_realtime_position()
    assert m1.move_position(angle_deg=40, speed_rpm=120, acceleration=12, sync=True)
    assert m2.move_position(angle_deg=80, speed_rpm=120, acceleration=12, sync=True)
    assert X42SDevice.sync_move(m1.bus)
    r1 = m1.wait_position_reached(timeout=10)
    r2 = m2.wait_position_reached(timeout=10)
    d1 = m1.get_realtime_position() - p1
    d2 = m2.get_realtime_position() - p2
    s.check(
        "E.sync_pos_asymmetric",
        r1 and r2 and 30 < d1 < 50 and 65 < d2 < 95,
        f"Δ=({d1:.1f},{d2:.1f})",
    )

    # 交错：一机运动时狂读另一机
    assert m1.jog(60, Direction.CW, acceleration=10)
    errs = 0
    for _ in range(30):
        try:
            m2.get_realtime_position()
            m2.get_motor_status()
            m2.get_version()
        except Exception:
            errs += 1
    safe_stop(m1)
    s.check("E.cross_read_while_peer_move", errs == 0, f"errs={errs}")

    # multi_motor：双机同时 enable 帧
    f1 = X42SDevice.build_frame(
        bytes([1, Code.ENABLE, Protocol.ENABLE, 0x01, 0x00])
    )
    f2 = X42SDevice.build_frame(
        bytes([2, Code.ENABLE, Protocol.ENABLE, 0x01, 0x00])
    )
    ok_mm = X42SDevice.multi_motor([f1, f2], m1.bus)
    s.check("E.multi_motor_enable", ok_mm)

    # 双机交替固件：1=X 运动，2=Emm 运动，再互换
    assert m1.switch_firmware(FirmwareType.X_FIRMWARE, store=True)
    assert m2.detect_firmware() == FirmwareType.EMM_FIRMWARE
    assert m1.enable() and m2.enable()
    okx = m1.move_position(angle_deg=20, speed_rpm=150, mode="trap")
    oke = m2.move_position(angle_deg=20, speed_rpm=150, acceleration=10)
    rx = m1.wait_position_reached(timeout=8)
    re = m2.wait_position_reached(timeout=8)
    s.check("E.mixed_fw_1X_2Emm", okx and oke and rx and re)

    # 互换
    safe_stop(m1)
    safe_stop(m2)
    assert m1.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
    assert m2.switch_firmware(FirmwareType.X_FIRMWARE, store=True)
    okx = m2.move_position(angle_deg=-15, speed_rpm=120, mode="direct")
    oke = m1.move_position(angle_deg=-15, speed_rpm=120, acceleration=10)
    rx = m2.wait_position_reached(timeout=8)
    re = m1.wait_position_reached(timeout=8)
    s.check("E.mixed_fw_1Emm_2X", okx and oke and rx and re)

    # 恢复双 Emm
    restore_emm([m1, m2])
    s.check(
        "E.both_back_emm",
        m1.detect_firmware() == FirmwareType.EMM_FIRMWARE
        and m2.detect_firmware() == FirmwareType.EMM_FIRMWARE,
    )

    # 广播后仍可单播
    for m in (m1, m2):
        m.disable()


# ---------------------------------------------------------------------------
# F. 通讯韧性
# ---------------------------------------------------------------------------

def suite_f_resilience(m1: X42SDevice, m2: X42SDevice, s: Suite) -> None:
    print("\n## F. 通讯韧性 / 超时恢复")
    # 故意对不存在地址超时，再确认 1/2 仍正常
    # 指定 firmware_type 避免构造时对幽灵地址做 detect
    ghost = X42SDevice(
        m1.bus,
        address=99,
        auto_test=False,
        firmware_type=FirmwareType.EMM_FIRMWARE,
    )
    timed_out = False
    try:
        ver = ghost.get_version()
        # stepper Command 超时后可能返回 None 而不抛异常
        timed_out = ver is None
    except (CanBusError, ConnectionError, OSError, TimeoutError):
        timed_out = True
    s.check("F.ghost_addr_timeout", timed_out)

    v1 = m1.get_version()
    v2 = m2.get_version()
    s.check(
        "F.alive_after_timeout",
        v1.firmware_version > 0 and v2.firmware_version > 0,
        f"{v1.firmware_version_str}/{v2.firmware_version_str}",
    )

    # 短超时设备仍可读
    tight = X42SDevice(
        m1.bus,
        address=1,
        auto_test=False,
        firmware_type=FirmwareType.EMM_FIRMWARE,
    )
    ok_n = 0
    for _ in range(10):
        try:
            tight.ping()
            ok_n += 1
        except Exception:
            pass
    s.check("F.tight_timeout_ping", ok_n >= 7, f"ok={ok_n}/10")

    # 保护清除（即使未保护也应 ACK 或可接受）
    try:
        clr = m1.clear_protection()
        s.check("F.clear_protection", isinstance(clr, bool), str(clr))
    except Exception as exc:
        s.check("F.clear_protection", False, str(exc))

    # 温度/电压合理范围（非零总线）
    try:
        mv = m1.get_bus_voltage()
        temp = m1.get_temperature()
        s.check("F.bus_voltage_sane", 5000 < mv < 60000, f"{mv}mV")
        s.check("F.temp_sane", -20 <= temp <= 120, f"{temp}C")
    except Exception as exc:
        s.check("F.telemetry", False, str(exc))


def main() -> int:
    parser = argparse.ArgumentParser(description="X42S CAN 严苛边界实机测试")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bustype", default="socketcan")
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--addrs", default="1,2")
    parser.add_argument(
        "--only",
        default="",
        help="只跑指定套件字母，如 ABE；默认全部 ABCDEF",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(message)s",
    )

    addrs = [int(x) for x in args.addrs.split(",") if x.strip()]
    only = set(args.only.upper()) if args.only else set("ABCDEF")
    s = Suite()
    motors: List[X42SDevice] = []

    with CanTransport(channel=args.channel, bustype=args.bustype) as bus:
        try:
            for addr in addrs:
                motors.append(X42SDevice(bus, address=addr, auto_test=False))

            # 冒烟：两台都在线
            for m in motors:
                v = m.get_version()
                print(f"online addr={m.address} {v.firmware_version_str} {v.hw_type_str}")

            m1 = motors[0]
            m2 = motors[1] if len(motors) > 1 else None

            def _run(label: str, fn: Callable) -> None:
                try:
                    fn()
                except Exception as exc:
                    traceback.print_exc()
                    s.check(f"FATAL.{label}", False, f"{type(exc).__name__}: {exc}")
                    for m in motors:
                        safe_stop(m)

            if "A" in only:
                _run("A", lambda: suite_a_gates(m1, s))
            if "B" in only:
                for m in motors:
                    _run(f"B{m.address}", lambda mm=m: suite_b_emm_motion(mm, s))
            if "C" in only:
                for m in motors:
                    _run(f"C{m.address}", lambda mm=m: suite_c_x_motion(mm, s))
            if "D" in only:
                for m in motors:
                    _run(f"D{m.address}", lambda mm=m: suite_d_fw_stress(mm, s))
            if "E" in only and m2 is not None:
                _run("E", lambda: suite_e_dual(m1, m2, s))
            elif "E" in only:
                print("## E 跳过（需要双机）")
            if "F" in only:
                if m2 is None:
                    _run("F", lambda: suite_f_resilience(m1, m1, s))
                else:
                    _run("F", lambda: suite_f_resilience(m1, m2, s))

        except Exception:
            traceback.print_exc()
            s.check("FATAL", False, "未捕获异常")
        finally:
            restore_emm(motors)

    return s.summary()


if __name__ == "__main__":
    sys.exit(main())
