"""X42S 双电机全方位工况压测.

用法: python bug_hunt_dual.py [COM口]
"""

from __future__ import annotations

import sys
import time
import traceback
from dataclasses import dataclass, field
from typing import Callable, List

from serial import Serial

from x42s_stepper import (
    Direction,
    FirmwareType,
    HomingMode,
    MotionMode,
    X42SDevice,
)
from x42s_stepper.commands import FirmwareCapabilityError, build_command_frame
from x42s_stepper.configs import Code, Protocol, SyncFlag, add_checksum
from x42s_stepper.parameters import (
    MAX_SPEED_RPM,
    MAX_SPEED_RPM_X,
    EmmJogParams,
    EmmPositionParams,
    XJogParams,
    XPositionDirectParams,
)


@dataclass
class Case:
    name: str
    ok: bool
    detail: str = ""
    severity: str = "bug"


@dataclass
class Suite:
    cases: List[Case] = field(default_factory=list)

    def add(self, name: str, ok: bool, detail: str = "", severity: str = "bug") -> bool:
        mark = "PASS" if ok else ("WARN" if severity == "warn" else "FAIL")
        extra = f" | {detail}" if detail else ""
        print(f"  [{mark}] {name}{extra}")
        self.cases.append(Case(name, ok, detail, severity))
        return ok

    def run(self, name: str, fn: Callable[[], str], severity: str = "bug") -> bool:
        try:
            return self.add(name, True, fn() or "", severity)
        except Exception as e:
            return self.add(name, False, f"{type(e).__name__}: {e}", severity)


def section(title: str) -> None:
    print(f"\n{'=' * 64}\n{title}\n{'=' * 64}")


def settle(sec: float = 0.15) -> None:
    time.sleep(sec)


def safe_stop_disable(*motors: X42SDevice) -> None:
    for m in motors:
        try:
            m.stop()
        except Exception:
            pass
    settle(0.08)
    for m in motors:
        try:
            m.disable()
        except Exception:
            pass


def switch_fw(m: X42SDevice, fw: FirmwareType, tag: str, S: Suite) -> bool:
    m.stop()
    settle(0.05)
    ok = m.switch_firmware(fw, store=True)
    actual = m.firmware_type
    # Turbo 探测为 Emm 布局，允许 firmware_type 缓存为 TURBO
    if fw == FirmwareType.EMM_TURBO:
        good = ok and actual in (FirmwareType.EMM_TURBO, FirmwareType.EMM_FIRMWARE)
    else:
        good = ok and actual == fw
    return S.add(
        f"{tag}_switch_{fw.name}",
        good,
        f"ok={ok} actual={actual.name}",
    )


def switch_both(m1: X42SDevice, m2: X42SDevice, fw: FirmwareType, S: Suite) -> bool:
    return switch_fw(m1, fw, "m1", S) and switch_fw(m2, fw, "m2", S)


def test_baseline(m1: X42SDevice, m2: X42SDevice, S: Suite) -> None:
    section("A. 基线探测")
    for tag, m in (("m1", m1), ("m2", m2)):
        def _ver(mm=m) -> str:
            v = mm.get_version()
            return (
                f"{v.firmware_version_str} hw={v.hw_type_str} "
                f"series={v.hw_series} type={v.hw_type} raw_hw={v.hw_version}"
            )

        S.run(f"{tag}_version", _ver)

        def _fw(mm=m) -> str:
            fw = mm.detect_firmware()
            return f"{fw.name} fast={mm.supports_fast_position} torque={mm.supports_torque}"

        S.run(f"{tag}_detect_fw", _fw)

        def _reads(mm=m) -> str:
            vbus = mm.get_bus_voltage()
            assert vbus > 5000
            return (
                f"V={vbus/1000:.2f} T={mm.get_temperature()} "
                f"spd={mm.get_realtime_speed()} pos={mm.get_realtime_position():.2f} "
                f"cfg={type(mm.get_config()).__name__}"
            )

        S.run(f"{tag}_reads", _reads)


def test_motion_single(m: X42SDevice, tag: str, fw: FirmwareType, S: Suite) -> None:
    section(f"B. 单轴 {tag} / {fw.name}")
    is_emm = fw != FirmwareType.X_FIRMWARE
    accel = 0 if is_emm else 2000

    def _enable() -> str:
        ok = m.enable()
        settle(0.1)
        assert ok and m.get_motor_status().enabled
        return "enabled"

    S.run(f"{tag}_enable", _enable)

    for dir_, dname in ((Direction.CW, "CW"), (Direction.CCW, "CCW")):
        def _jog(d=dir_, dn=dname) -> str:
            ok = m.jog(speed_rpm=120, direction=d, acceleration=accel)
            settle(0.5)
            spd = m.get_realtime_speed()
            m.stop()
            settle(0.2)
            assert ok and abs(spd) > 30, f"ok={ok} spd={spd}"
            return f"spd={spd:.1f}"

        S.run(f"{tag}_jog_{dname}", _jog)

    def _ladder() -> str:
        peaks = []
        # X 加速度为 RPM/s，高转速需更长爬升时间
        ladder_accel = 0 if is_emm else 8000
        for rpm in (50, 200, 500, 1000, 2000):
            ok = m.jog(
                speed_rpm=rpm, direction=Direction.CW, acceleration=ladder_accel
            )
            settle(1.2 if rpm >= 1000 and not is_emm else 0.85)
            peak = max(abs(m.get_realtime_speed()) for _ in range(5))
            peaks.append((rpm, peak))
            m.stop()
            settle(0.3)
            tol = max(40, rpm * 0.18)
            assert ok and abs(peak - rpm) <= tol, f"cmd={rpm} peak={peak}"
        return str([(c, round(p)) for c, p in peaks])

    S.run(f"{tag}_speed_ladder", _ladder)

    def _high() -> str:
        target = 3000 if not is_emm else 3000
        a = 0 if is_emm else 8000
        ok = m.jog(speed_rpm=target, direction=Direction.CW, acceleration=a)
        settle(1.0)
        peak = max(abs(m.get_realtime_speed()) for _ in range(6))
        m.stop()
        settle(0.3)
        assert ok and peak > target * 0.7, f"ok={ok} peak={peak}"
        return f"peak={peak:.1f}"

    S.run(f"{tag}_jog_3000", _high)

    if is_emm:
        def _emm_high() -> str:
            ok = m.jog(speed_rpm=4000, direction=Direction.CW, acceleration=0)
            settle(1.0)
            peak = max(abs(m.get_realtime_speed()) for _ in range(6))
            m.stop()
            settle(0.3)
            assert ok and peak > 2000, f"ok={ok} peak={peak}"
            return f"peak={peak:.1f} (cmd=4000)"

        S.run(f"{tag}_jog_4000_emm", _emm_high)
    else:
        def _x_cap() -> str:
            ok = m.jog(speed_rpm=MAX_SPEED_RPM_X, direction=Direction.CW, acceleration=8000)
            settle(1.0)
            peak = max(abs(m.get_realtime_speed()) for _ in range(5))
            m.stop()
            settle(0.25)
            assert ok and peak > 2000
            # 超上限应被库拒绝
            try:
                m.jog(speed_rpm=MAX_SPEED_RPM_X + 1, acceleration=8000)
                raise AssertionError("should reject")
            except ValueError as e:
                return f"peak@{MAX_SPEED_RPM_X}={peak:.1f}; reject={e}"

        S.run(f"{tag}_x_speed_cap", _x_cap)

    def _pos() -> str:
        if is_emm:
            ok = m.move_position(
                pulses=1600, speed_rpm=300, acceleration=20,
                motion_mode=MotionMode.RELATIVE_CURRENT,
            )
        else:
            ok = m.move_position(
                angle_deg=90, speed_rpm=300, acceleration=1500,
                mode="direct", motion_mode=MotionMode.RELATIVE_CURRENT,
            )
        reached = m.wait_position_reached(timeout=10)
        assert ok and reached
        return f"pos={m.get_realtime_position():.2f}"

    S.run(f"{tag}_pos_relative", _pos)

    if not is_emm:
        def _trap() -> str:
            ok = m.move_position(
                angle_deg=60, speed_rpm=200, acceleration=800, decel=800,
                mode="trap", motion_mode=MotionMode.RELATIVE_CURRENT,
            )
            assert ok and m.wait_position_reached(timeout=10)
            return "ok"

        S.run(f"{tag}_pos_trap", _trap)

        def _torque() -> str:
            ok = m.torque(current_ma=400, slope_ma_s=2000)
            settle(0.4)
            spd = abs(m.get_realtime_speed())
            m.stop()
            settle(0.2)
            assert ok and spd > 5
            return f"spd={spd:.1f}"

        S.run(f"{tag}_torque", _torque)

        def _tq_lim() -> str:
            ok = m.torque_limited(current_ma=500, max_speed_rpm=80, slope_ma_s=2000)
            settle(0.5)
            spd = abs(m.get_realtime_speed())
            m.stop()
            settle(0.2)
            assert ok and 20 < spd < 200
            return f"spd={spd:.1f}"

        S.run(f"{tag}_torque_limited", _tq_lim)
    else:
        def _no_tq() -> str:
            try:
                m.torque(current_ma=300)
                raise AssertionError("should reject")
            except FirmwareCapabilityError as e:
                return str(e)

        S.run(f"{tag}_reject_torque", _no_tq)

    # 快速位置 F1/FC
    def _fast() -> str:
        if not m.supports_fast_position:
            raise AssertionError("no fast position")
        ok = m.configure_fast_position(
            speed_rpm=300,
            acceleration=20 if is_emm else 1000,
            motion_mode=MotionMode.RELATIVE_CURRENT,
        )
        assert ok
        if is_emm:
            assert m.move_fast_pulses(800)
        else:
            assert m.move_fast_degrees(45)
        reached = m.wait_position_reached(timeout=8)
        assert reached
        return "ok"

    S.run(f"{tag}_fast_position", _fast)

    def _estop() -> str:
        m.jog(speed_rpm=600, direction=Direction.CW, acceleration=accel)
        settle(0.35)
        ok = m.stop()
        settle(0.3)
        spd = abs(m.get_realtime_speed())
        assert ok and spd < 40
        return f"spd={spd:.1f}"

    S.run(f"{tag}_estop", _estop)

    def _home() -> str:
        m.enable()
        m.clear_protection()
        # 大多圈坐标下 ABS/NEAREST 易失败，先清零坐标系
        m.zero_position()
        settle(0.08)
        okz = m.set_home_zero(store=True)
        settle(0.1)
        if is_emm:
            m.move_position(pulses=500, speed_rpm=120, acceleration=12)
        else:
            m.move_position(angle_deg=40, speed_rpm=120, acceleration=1000, mode="direct")
        m.wait_position_reached(timeout=8)
        ok = m.home(mode=HomingMode.ABS_ZERO)
        if not ok:
            ok = m.home(mode=HomingMode.NEAREST)
        done = m.wait_homing(timeout=15)
        hs = m.get_homing_status()
        assert okz and ok and done and not hs.homing_failed, (
            f"ok={ok} done={done} hs={hs}"
        )
        return f"pos={m.get_realtime_position():.2f}"

    S.run(f"{tag}_home", _home)

    safe_stop_disable(m)


def test_dual(m1: X42SDevice, m2: X42SDevice, fw: FirmwareType, S: Suite) -> None:
    section(f"C. 双轴协同 / {fw.name}")
    tag = fw.name
    is_emm = fw != FirmwareType.X_FIRMWARE
    accel = 0 if is_emm else 2000

    m1.enable()
    m2.enable()
    settle(0.1)

    def _iso() -> str:
        p2a = m2.get_realtime_position()
        m1.jog(speed_rpm=150, direction=Direction.CW, acceleration=accel)
        settle(0.55)
        s1, s2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        p2b = m2.get_realtime_position()
        m1.stop()
        settle(0.2)
        assert s1 > 40 and s2 < 20 and abs(p2b - p2a) < 3
        return f"s1={s1:.1f} s2={s2:.1f}"

    S.run(f"{tag}_isolation", _iso)

    def _opp() -> str:
        ok1 = m1.jog(speed_rpm=100, direction=Direction.CW, acceleration=accel)
        ok2 = m2.jog(speed_rpm=100, direction=Direction.CCW, acceleration=accel)
        settle(0.45)
        s1, s2 = m1.get_realtime_speed(), m2.get_realtime_speed()
        m1.stop(); m2.stop(); settle(0.2)
        assert ok1 and ok2 and abs(s1) > 30 and abs(s2) > 30
        return f"s1={s1:.1f} s2={s2:.1f}"

    S.run(f"{tag}_opposite_jog", _opp)

    def _sync() -> str:
        a = accel or 10
        ok1 = m1.jog(speed_rpm=80, direction=Direction.CW, acceleration=a, sync=True)
        ok2 = m2.jog(speed_rpm=80, direction=Direction.CCW, acceleration=a, sync=True)
        settle(0.12)
        pre1, pre2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        ok = X42SDevice.sync_move(m1.device_params)
        settle(0.45)
        post1, post2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        m1.stop(); m2.stop(); settle(0.2)
        assert ok1 and ok2 and ok and pre1 < 20 and pre2 < 20 and post1 > 20 and post2 > 20
        return f"pre={pre1:.0f}/{pre2:.0f} post={post1:.0f}/{post2:.0f}"

    S.run(f"{tag}_sync_ff66", _sync)

    def _aa() -> str:
        cs = m1.device_params.checksum_mode
        if is_emm:
            b1 = bytes([1, Code.JOG]) + EmmJogParams(
                direction=Direction.CW, speed=60, acceleration=10
            ).bytes
            b2 = bytes([2, Code.JOG]) + EmmJogParams(
                direction=Direction.CCW, speed=60, acceleration=10
            ).bytes
        else:
            b1 = bytes([1, Code.JOG]) + XJogParams(
                direction=Direction.CW, acceleration_rpm_s=1000, speed_raw=600
            ).bytes
            b2 = bytes([2, Code.JOG]) + XJogParams(
                direction=Direction.CCW, acceleration_rpm_s=1000, speed_raw=600
            ).bytes
        ok = X42SDevice.multi_motor(
            [build_command_frame(b1, cs), build_command_frame(b2, cs)],
            m1.device_params,
            expect_ack=True,
        )
        settle(0.4)
        s1, s2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        stop1 = add_checksum(bytes([1, Code.ESTOP, Protocol.ESTOP, 0]), cs)
        stop2 = add_checksum(bytes([2, Code.ESTOP, Protocol.ESTOP, 0]), cs)
        X42SDevice.multi_motor([stop1, stop2], m1.device_params, expect_ack=True)
        settle(0.2)
        assert ok and s1 > 10 and s2 > 10
        return f"s1={s1:.1f} s2={s2:.1f}"

    S.run(f"{tag}_multi_aa", _aa)

    def _sync_pos() -> str:
        if is_emm:
            ok1 = m1.move_position(pulses=1000, speed_rpm=200, acceleration=15, sync=True)
            ok2 = m2.move_position(pulses=1000, speed_rpm=200, acceleration=15, sync=True)
        else:
            ok1 = m1.move_position(
                angle_deg=40, speed_rpm=200, acceleration=1000, mode="direct", sync=True
            )
            ok2 = m2.move_position(
                angle_deg=40, speed_rpm=200, acceleration=1000, mode="direct", sync=True
            )
        X42SDevice.sync_move(m1.device_params)
        r1 = m1.wait_position_reached(timeout=10)
        r2 = m2.wait_position_reached(timeout=10)
        assert ok1 and ok2 and r1 and r2
        return "ok"

    S.run(f"{tag}_sync_position", _sync_pos)

    def _indep() -> str:
        a = accel or 10
        m1.jog(speed_rpm=120, direction=Direction.CW, acceleration=a)
        m2.jog(speed_rpm=120, direction=Direction.CW, acceleration=a)
        settle(0.4)
        m1.stop()
        settle(0.35)
        s1, s2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        m2.stop()
        settle(0.2)
        assert s1 < 40 and s2 > 30
        return f"s1={s1:.1f} s2={s2:.1f}"

    S.run(f"{tag}_independent_stop", _indep)
    safe_stop_disable(m1, m2)


def test_turbo_and_mixed(m1: X42SDevice, m2: X42SDevice, S: Suite) -> None:
    section("D. Turbo / 混固件 / 切固件压力")

    def _flip() -> str:
        seq = []
        for fw in (
            FirmwareType.X_FIRMWARE,
            FirmwareType.EMM_FIRMWARE,
            FirmwareType.X_FIRMWARE,
            FirmwareType.EMM_FIRMWARE,
        ):
            m1.stop()
            ok = m1.switch_firmware(fw, store=True)
            actual = m1.firmware_type
            seq.append((fw.name, ok, actual.name))
            assert ok and actual == fw, str(seq)
            m1.enable()
            a = 0 if fw != FirmwareType.X_FIRMWARE else 1500
            assert m1.jog(speed_rpm=80, acceleration=a)
            settle(0.35)
            assert abs(m1.get_realtime_speed()) > 20
            m1.stop()
            settle(0.12)
        return str(seq)

    S.run("m1_fw_flip_x4", _flip)

    def _turbo() -> str:
        # Turbo 可能需校准；失败记 WARN
        m1.stop()
        ok = m1.switch_firmware(FirmwareType.EMM_TURBO, store=True)
        if not ok:
            raise AssertionError("switch turbo failed")
        m1.enable()
        okj = m1.jog(speed_rpm=100, acceleration=10)
        settle(0.4)
        spd = abs(m1.get_realtime_speed())
        m1.stop()
        # 切回普通 Emm
        assert m1.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
        assert okj and spd > 20
        return f"turbo_fw={m1.firmware_type.name} peak_during_turbo~{spd:.0f}"

    try:
        detail = _turbo()
        S.add("m1_emm_turbo_roundtrip", True, detail)
    except Exception as e:
        S.add("m1_emm_turbo_roundtrip", False, str(e), severity="warn")

    def _mixed() -> str:
        assert m1.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
        assert m2.switch_firmware(FirmwareType.X_FIRMWARE, store=True)
        m1.enable(); m2.enable()
        assert m1.jog(speed_rpm=70, acceleration=10)
        assert m2.jog(speed_rpm=70, acceleration=1500)
        settle(0.4)
        s1, s2 = abs(m1.get_realtime_speed()), abs(m2.get_realtime_speed())
        m1.stop(); m2.stop()
        assert s1 > 15 and s2 > 15
        return f"s={s1:.0f}/{s2:.0f}"

    S.run("mixed_firmware_dual", _mixed)
    safe_stop_disable(m1, m2)


def test_edges(m1: X42SDevice, m2: X42SDevice, S: Suite) -> None:
    section("E. 边界")
    if m1.firmware_type == FirmwareType.X_FIRMWARE:
        m1.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
    m1.enable()

    def _reject() -> str:
        try:
            m1.jog(speed_rpm=MAX_SPEED_RPM + 1)
            raise AssertionError("should reject")
        except ValueError as e:
            return str(e)

    S.run("reject_over_emm_max", _reject)

    def _timed() -> str:
        ok1 = m1.timed_return(0x35, 80)
        settle(0.15)
        ser = m1.device_params.serial_connection
        ser.reset_input_buffer()
        ok2 = m1.timed_return(0x35, 0)
        settle(0.05)
        ser.reset_input_buffer()
        return f"on={ok1} off={ok2}"

    S.run("timed_return", _timed, severity="warn")

    def _flood() -> str:
        t0 = time.time()
        for _ in range(40):
            m1.get_realtime_speed()
            m2.get_bus_voltage()
        return f"80 reads in {time.time()-t0:.2f}s"

    S.run("read_flood", _flood)
    safe_stop_disable(m1, m2)


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
    print(f"X42S Bug Hunt — port={port} addrs=1,2")
    S = Suite()
    ser = Serial(port, 115200, timeout=0.18)
    originals: dict[int, FirmwareType] = {}

    try:
        m1 = X42SDevice(ser, address=1)
        m2 = X42SDevice(ser, address=2)
        originals[1] = m1.detect_firmware()
        originals[2] = m2.detect_firmware()
        v1, v2 = m1.get_version(), m2.get_version()
        section(
            f"初始 m1={originals[1].name}/{v1.firmware_version_str}/{v1.hw_type_str} "
            f"m2={originals[2].name}/{v2.firmware_version_str}/{v2.hw_type_str} "
            f"Vbus={m1.get_bus_voltage()/1000:.2f}V"
        )
        print(
            "注: 版本帧 hw_series 可能非 0/1；按用户确认按 X42S 全量测试。"
        )

        test_baseline(m1, m2, S)

        for fw in (FirmwareType.EMM_FIRMWARE, FirmwareType.X_FIRMWARE):
            if not switch_both(m1, m2, fw, S):
                S.add(f"abort_{fw.name}", False, "switch failed")
                continue
            test_motion_single(m1, "m1", fw, S)
            test_motion_single(m2, "m2", fw, S)
            test_dual(m1, m2, fw, S)

        test_turbo_and_mixed(m1, m2, S)
        test_edges(m1, m2, S)

    except Exception:
        print("\n[FATAL]\n" + traceback.format_exc())
        S.add("fatal", False, "exception")
    finally:
        section("恢复 Emm 并停机")
        try:
            for addr in (1, 2):
                m = X42SDevice(ser, address=addr, auto_test=False)
                m.detect_firmware()
                m.stop()
                # 默认回到 Emm（X42S 出厂常见）
                target = originals.get(addr, FirmwareType.EMM_FIRMWARE)
                if target == FirmwareType.EMM_TURBO:
                    target = FirmwareType.EMM_FIRMWARE
                ok = m.switch_firmware(target, store=True)
                S.add(f"restore_addr{addr}", ok, target.name, severity="warn")
                m.disable()
        except Exception as e:
            print("restore warn:", e)
        try:
            ser.close()
        except Exception:
            pass

    section("汇总")
    fails = [c for c in S.cases if not c.ok and c.severity == "bug"]
    warns = [c for c in S.cases if not c.ok and c.severity == "warn"]
    passes = [c for c in S.cases if c.ok]
    print(f"total={len(S.cases)} pass={len(passes)} FAIL={len(fails)} WARN={len(warns)}")
    if fails:
        print("\n--- FAIL ---")
        for c in fails:
            print(f"  - {c.name}: {c.detail}")
    if warns:
        print("\n--- WARN ---")
        for c in warns:
            print(f"  - {c.name}: {c.detail}")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
