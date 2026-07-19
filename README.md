# x42s_stepper — ZDT X42S 双固件闭环步进电机控制库

基于 **[ZDT X42S 用户手册 V1.0.5](docs/ZDT_X42S第二代闭环步进电机用户手册V1.0.5_260527.pdf)**，面向 **X42S 板卡**，同时支持：

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

## 特性

- ✅ 串口 TTL/RS485 自由协议（FIXED `0x6B` / XOR / CRC8）
- ✅ 多电机（同串口不同地址）
- ✅ X / Emm / Turbo 统一物理单位 API（度 / RPM / mA）
- ✅ 固件探测（配置块 `0x42`：`0x25`=X，`0x21`=Emm）与切换（`0xD5`）
- ✅ 速度 / 位置 / X 力矩与限流运动
- ✅ 快速位置 F1/FC（Emm 脉冲 / X 0.1° 角度，V2.0.0+）
- ✅ 回零、同步 `FF66`、多机 `0xAA`、定时返回、配置与 PID

## 安装

本项目已发布到 PyPI，推荐直接使用 pip 安装：

```bash
pip install x42s_stepper
```

该命令会自动安装依赖（`pyserial>=3.5`）。需要 Python `>=3.8`。

也可从源码或 GitHub 安装：

```bash
cd x42s_stepper && pip install -e .
# 或
pip install git+https://github.com/xhfb/x42s_stepper.git
```

PyPI：[https://pypi.org/project/x42s-stepper/](https://pypi.org/project/x42s-stepper/)

## 快速开始

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

motor.move_position(angle_deg=90, speed_rpm=200, motion_mode=MotionMode.RELATIVE_CURRENT)
motor.wait_position_reached(timeout=10)

# 快速位置 (V2.0.0+)
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

## 双固件说明

| 项目 | X | Emm / Turbo |
|------|---|-------------|
| 速度 | 0.1RPM + RPM/s；库上限约 **3000 RPM** | 整数 RPM + 档位；库上限 **6000 RPM** |
| 位置 | 直通 `FB` / 梯形 `FD` | 脉冲 `FD` |
| 力矩 | 有 | 无 |
| 快速位置 F1/FC | F1 梯形参数 + FC 角度(0.1°) | F1 速度档位 + FC 脉冲 |
| Turbo | — | `FirmwareType.EMM_TURBO`，布局同 Emm |

```python
motor.stop()
motor.switch_firmware(FirmwareType.X_FIRMWARE, store=True)
motor.switch_firmware(FirmwareType.EMM_FIRMWARE, store=True)
# 狂暴模式前请按手册完成校准
motor.switch_firmware(FirmwareType.EMM_TURBO, store=True)
```

## 与 emm_stepper / y42_stepper

- 从 **emm_stepper** 迁徙：把 `EmmDevice` 换成 `X42SDevice`；`jog(speed=...)` 改为 `jog(speed_rpm=...)`；位置用 `move_position`；F1/FC API 名称兼容（`configure_fast_position` / `move_fast_pulses`）。
- 与 **y42_stepper** 同架构（门面 + 双布局参数 + 配置块探测），但增加 Turbo 与双边 F1/FC，速度上限按 X42S 手册区分。

## 实机测试

```bash
python full_test.py COM9
python full_test.py COM9 --move --firmware
```

## 许可证

MIT License

## 作者

XHFB
