# x42s_stepper — ZDT X42S 双固件闭环步进电机控制库

**版本 0.2.0** · 基于 **[ZDT X42S 用户手册 V1.0.5](docs/ZDT_X42S第二代闭环步进电机用户手册V1.0.5_260527.pdf)**

面向 **X42S 板卡**，同一套 `X42SDevice` API 同时支持：

| 传输 | 用法 | 依赖 |
|------|------|------|
| **串口** TTL / RS485 | `X42SDevice(Serial(...), address=1)` | `pyserial`（默认） |
| **SocketCAN** | `X42SDevice(CanTransport("can0"), address=1)` | `x42s_stepper[can]` → `python-can` |

固件：

- **X 固件**
- **Emm 固件**
- **Emm 狂暴 (Turbo)**（`0xD5` 类型码 `02`）

实机能力因固件版本而异；快速位置 F1/FC 需要 **V2.0.0+**。

## 相关项目

| 仓库 | 定位 |
|------|------|
| [xhfb/emm_stepper](https://github.com/xhfb/emm_stepper) | X42S **仅 Emm**（历史库，本库为其双固件超集） |
| [xhfb/y42_stepper](https://github.com/xhfb/y42_stepper) | **Y42** 板卡双固件（X/Emm，无 Turbo/F1 侧重点不同） |

> 板卡系列协议相近但不全相同，**请按板卡选型装库**，不要混用 `X42SDevice` / `Y42Device`。  
> 原 RDK 侧独立包 `x42s_can` 的协议实现已合并进本库；请改用 `x42s_stepper[can]`，避免双份协议漂移。

## 特性

- ✅ 串口 TTL/RS485 自由协议（FIXED `0x6B` / XOR / CRC8）
- ✅ **SocketCAN**（经典 CAN @ **500 kbps**，扩展帧分包；**勿开 CAN FD**）
- ✅ 统一 `Transport` 抽象：`SerialTransport` / `CanTransport`，业务命令只写一份
- ✅ 多电机（同串口 / 同总线不同地址）
- ✅ X / Emm / Turbo 统一物理单位 API（度 / RPM / mA）
- ✅ 固件探测（配置块 `0x42`：`0x25`=X，`0x21`=Emm）与切换（`0xD5`）
- ✅ 速度 / 位置 / X 力矩与限流运动
- ✅ 快速位置 F1/FC（Emm 脉冲 / X 0.1° 角度，V2.0.0+）
- ✅ 回零、同步 `FF66`、多机 `0xAA`、定时返回、配置与 PID

## 架构（0.2.0）

```text
X42SDevice
    └── DeviceParams.transport : Transport
            ├── SerialTransport  →  pyserial.Serial
            └── CanTransport     →  python-can SocketCAN + 扩展帧分包
commands/*  ──►  transport.request / request_dynamic / send
```

串口用法保持兼容：传入 `Serial` 时自动包装为 `SerialTransport`。  
CAN：显式传入 `CanTransport`（或别名 `CanBus`）。

## 安装

```bash
# 仅串口
pip install x42s_stepper

# 串口 + CAN
pip install 'x42s_stepper[can]'
```

依赖：Python `>=3.8`，`pyserial>=3.5`；CAN 另需 `python-can>=4.0`。

源码 / 可编辑安装：

```bash
cd x42s_stepper
pip install -e ".[can]"          # 含 CAN
pip install -e ".[can,dev]"      # 另含 pytest 等
```

或：

```bash
pip install "x42s_stepper[can] @ git+https://github.com/xhfb/x42s_stepper.git"
```

PyPI：[https://pypi.org/project/x42s-stepper/](https://pypi.org/project/x42s-stepper/)

## 快速开始（串口）

```python
from serial import Serial
from x42s_stepper import X42SDevice, Direction, FirmwareType, MotionMode

ser = Serial("COM9", 115200, timeout=0.1)
motor = X42SDevice(ser, address=1)

print(motor.firmware_type, motor.get_version().firmware_version_str)

motor.enable()
motor.jog(speed_rpm=100, direction=Direction.CW)
import time
time.sleep(1)
motor.stop()

motor.move_position(
    angle_deg=90, speed_rpm=200, motion_mode=MotionMode.RELATIVE_CURRENT
)
motor.wait_position_reached(timeout=10)

if motor.supports_fast_position:
    motor.configure_fast_position(speed_rpm=800, acceleration=100)
    if motor.firmware_type != FirmwareType.X_FIRMWARE:
        motor.move_fast_pulses(3200)
    else:
        motor.move_fast_degrees(90)
    motor.wait_position_reached()

motor.disable()
ser.close()
```

## 快速开始（CAN / SocketCAN）

硬件要点（如 RDK X5 板载 `can0`）：

- 经典 CAN **500 kbps**，扩展帧；**不要开 CAN FD / LOOPBACK**
- 电机：`P_Serial=CAN1_MAP`，`CAN_Baud=500000`，地址按现场配置

```bash
./examples/can/bringup_can0.sh
python examples/can/ping_motor.py --addrs 1
python examples/can/demo_dual_fw.py --addrs 1   # 结束强制恢复 Emm
```

```python
from x42s_stepper import X42SDevice, CanTransport, Direction, MotionMode

with CanTransport("can0") as bus:
    m1 = X42SDevice(bus, address=1)
    m2 = X42SDevice(bus, address=2)  # 多机共享同一 bus

    print(m1.ping().firmware_version_str)
    m1.enable()
    m1.jog(speed_rpm=100, direction=Direction.CW)
    m1.stop()
    m1.move_position(
        angle_deg=90, speed_rpm=200, motion_mode=MotionMode.RELATIVE_CURRENT
    )
    m1.wait_position_reached()
    m1.disable()
```

同步运动（缓存后广播 `FF66`）：

```python
m1.jog(50, Direction.CW, acceleration=10, sync=True)
m2.jog(50, Direction.CCW, acceleration=10, sync=True)
X42SDevice.sync_move(bus)   # 或 X42SDevice.sync_move(m1.device_params)
```

> `sync_move` / `multi_motor` 在 CAN 上偶发无 ACK 时**不重试**（避免位置/速度命令多转）。

### 无硬件时用 vcan 自测

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set up vcan0

# 终端 A
python examples/can/sim_emm_motor.py --channel vcan0

# 终端 B
python examples/can/ping_motor.py --channel vcan0 --bustype virtual
```

## CAN 示例脚本

| 脚本 | 说明 |
|------|------|
| `examples/can/bringup_can0.sh` | 拉起经典 CAN @ 500 kbps |
| `examples/can/ping_motor.py` | 读版本 / 接口检查 |
| `examples/can/demo_dual_fw.py` | Emm↔X 切换与运动冒烟（结束恢复 Emm） |
| `examples/can/demo_sync_dual.py` | 双机 `FF66` 同步 |
| `examples/can/stress_boundary.py` | 双机严苛边界（ABCDEF） |
| `examples/can/sim_emm_motor.py` | virtual/vcan 模拟电机 |

```bash
# 双机边界（默认地址 1,2）
python examples/can/stress_boundary.py --addrs 1,2
```

## 双固件说明

| 项目 | X | Emm / Turbo |
|------|---|-------------|
| 速度 | 0.1RPM + RPM/s；库上限约 **3000 RPM** | 整数 RPM + 档位；库上限 **6000 RPM** |
| 位置 | 直通 `FB` / 梯形 `FD` | 脉冲 `FD` |
| 力矩 | 有 | 无（调用抛 `FirmwareCapabilityError`） |
| 快速位置 F1/FC | F1 梯形参数 + FC 角度(0.1°) | F1 速度档位 + FC 脉冲 |
| PID | `XPIDParams`（16 字节） | `EmmPIDParams`（12 字节）；`set_pid` 须与当前固件匹配 |
| Turbo | — | `FirmwareType.EMM_TURBO`，布局同 Emm |

```python
motor.stop()
motor.switch_firmware(FirmwareType.X_FIRMWARE, store=True)
motor.detect_firmware()   # 以配置块 0x42 总长度为准；勿信 option_status 的 FwType
motor.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
# 狂暴模式前请按手册完成校准
motor.switch_firmware(FirmwareType.EMM_TURBO, store=True)
```

> 探测：只看配置块长度 — `0x25`=X，`0x21`=Emm。`store=False` 的切换实机上常不生效，建议 `store=True`。

## 主要导出

```python
from x42s_stepper import (
    X42SDevice,
    SerialTransport,
    CanTransport,   # 别名 CanBus
    CanBusError,
    TransportError,
    Direction,
    MotionMode,
    FirmwareType,
    HomingMode,
    FirmwareCapabilityError,
)
```

- `X42SDevice(connection, address=1, ...)`：`connection` 为 `Serial` 或 `Transport`
- `motor.transport` / `motor.bus`：底层传输（CAN 示例里常用 `.bus`）
- `motor.device_params.serial_connection`：仅串口时返回底层 `Serial`，否则 `None`

## 测试

```bash
# 无硬件（codec + 串口 Transport 构造）
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests/ -q

# 串口实机
python full_test.py COM9
python full_test.py COM9 --move --firmware

# CAN 实机（见 examples/can/）
python examples/can/ping_motor.py --addrs 1
python examples/can/demo_dual_fw.py --addrs 1
python examples/can/stress_boundary.py --addrs 1,2
```

本机（RDK + 双机 Y42 / V1.0.7）边界验收：`stress_boundary.py --addrs 1,2` → **92/92 PASS**，结束已恢复 Emm。

## 与 emm_stepper / y42_stepper / x42s_can

- 从 **emm_stepper** 迁徙：`EmmDevice` → `X42SDevice`；`jog(speed=...)` → `jog(speed_rpm=...)`；位置用 `move_position`。
- 与 **y42_stepper** 同架构，但增加 Turbo 与双边 F1/FC，速度上限按 X42S 手册区分。
- 从 **x42s_can** 迁徙：`CanBus` → `CanTransport`（或仍用别名 `CanBus`）；`X42SCanDevice` → `X42SDevice`；安装改为 `pip install 'x42s_stepper[can]'`。

## 许可证

MIT License

## 作者

XHFB
