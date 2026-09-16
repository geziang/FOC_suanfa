#pragma once
// FocKit —— 关节电机中间件/上层算法开发库
// 架构与契约见 docs/00_项目目标与总体方案.md
//
//   core/     IMotor 契约（移植边界）
//   adapter/  SimpleFocMotor（官方库实现）、MotorManager（双电机切换）
//   bsp/      DengFOC V4 板级真值、欠压保护
//   control/  PID、速度/位置环节点（中间件三环起步）
//   persist/  NVS 标定固化
//   hmi/      串口调参命令行

#define FOCKIT_VERSION "0.1.0"

#include "core/ControlMode.h"
#include "core/MotorState.h"
#include "core/IMotor.h"

#include "bsp/DengFocBoard.h"
#include "bsp/PowerMonitor.h"

#include "persist/CalibrationStore.h"

#include "adapter/SimpleFocMotor.h"
#include "adapter/MotorManager.h"

#include "control/PID.h"
#include "control/VelocityNode.h"
#include "control/PositionNode.h"

#include "hmi/SerialShell.h"
