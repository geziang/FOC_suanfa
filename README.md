# FocKit

> 关节电机中间件/上层算法开发库 —— 在 DengFOC V4 套件上按工业标准先行开发三环控制、力控与运控算法，实验室自研硬件到位后**换一个适配层、上层零改动**完成平移。

文档入口见 [docs/00_文档总览.md](docs/00_文档总览.md)——分层文档库（L1 需求选型 → L6 验证记录），含台账、验收合同与模板体系。

## 架构

```
上层 app        力控(阻抗/重力补偿) · 运控(轨迹) · 辨识
中间件 control  三环级联(位置←速度←力矩) · 滤波
契约 core       IMotor 运动接口  ★移植边界
适配层 adapter  SimpleFocMotor(现行) · LabCanMotor(未来)
板级/支撑       bsp(DengFOC V4) · persist(NVS标定固化) · hmi(串口调参)
依赖            官方 SimpleFOC 库（只调用，不修改）
```

**分层契约**：上层只给目标（力矩 N·m / 速度 rad/s / 位置 rad）、读状态、切模式；禁止直接操作电压/PWM/电流。`core/` 与 `control/` 层不得 `#include <SimpleFOC.h>`，编译即检查。

## 目录

```
src/
├── core/       IMotor.h 契约接口、ControlMode、MotorState
├── adapter/    SimpleFocMotor（官方库实现）、MotorManager（M0/M1 切换）
├── bsp/        DengFocBoard.h 板级真值、PowerMonitor 欠压保护
├── control/    PID、VelocityNode、PositionNode（中间件三环起步）
├── persist/    CalibrationStore（NVS 标定固化，对标 STM32 Flash 双页）
└── hmi/        SerialShell（串口调参命令行）
examples/       01 电压力矩 → 02 速度环 → 03 位置环（渐进验证）
docs/           分层文档库（00_文档总览 为入口：需求/架构/验收合同/硬件档案/详细设计/验证记录）
main/           main.ino = 01 号示例的入口副本
```

## 快速开始

1. **装依赖**（Arduino IDE）：
   - ESP32 开发板支持 ≥ 2.0.4（离线包在 `v4/1、V4到手资料/`，或开发板管理器在线装）
   - 库管理器搜索 **Simple FOC** 安装（≥ 2.2.1，官方例程实测版本）
2. **选板卡**：`ESP32 Dev Module`
3. **打开示例**：`examples/FocKit_01_torque_voltage/FocKit_01_torque_voltage.ino`
   - Arduino IDE 会自动识别示例所属的本库；若未识别，做一次目录联接（管理员 cmd）：
     `mklink /J "%USERPROFILE%\Documents\Arduino\libraries\FocKit" "C:\Users\21153\Desktop\simplefoc"`
4. **上电流程**：接 12V 供电（≥11.1V）→ 打开串口 115200 → 首次烧录会自动做编码器零位/方向标定并**写入 NVS**，断电重启自动注入，不再重复标定。
5. 串口命令（见 `help`）：`on`/`off`、`t 0.01`（N·m）、`v 5`（rad/s）、`p 3.14`（rad）、`stream`（10Hz 状态流，串口绘图器可用）、`pid v kp ki kd`、`save`、`sel 0|1`（切电机）。

## 硬件档案（真值见 docs/00 §6）

| 项 | 值 |
|---|---|
| 板 | DengFOC V4（ESP32-WROOM-32U 双路） |
| 电机 | 2208 云台电机，7 对极，21.2Ω(线)，5.3mH，110KV，0.03N·m |
| 编码器 | AS5600 ×2（I2C 双总线：M0=19/18，M1=23/5） |
| 引脚 | M0 相线 32/33/25，M1 相线 26/27/14，使能 12，VIN 检测 13 |

## 移植指南（P4）

实验室硬件到位后：

1. 新增 `src/adapter/LabCanMotor.h/.cpp`，实现 `IMotor` 全部纯虚接口（CAN 对接实验室底层）；
2. 上层与中间件**一行不改**；
3. `bsp/` 新增对应板卡档案即可。

## 致谢

- [DengFOC 灯哥开源FOC](https://github.com/ToanTech/Deng-s-foc-controller)（硬件与教程）
- [SimpleFOC](https://docs.simplefoc.com/)（底层库）
