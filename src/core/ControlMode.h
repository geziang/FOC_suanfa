#pragma once
// FocKit 契约层 —— 控制模式定义
// 分层契约（docs/00 §3.1）：上层只能通过目标接口下指令，
// 不存在“直接给电压/PWM”的模式。

#include <stdint.h>

namespace fockit {

enum class ControlMode : uint8_t {
  Idle,      // 空载：保持使能、零电压输出（电机可被手动旋动）
  Torque,    // 力矩目标，单位 N·m
  Velocity,  // 速度目标，单位 rad/s
  Position   // 位置目标，单位 rad
};

inline const char* controlModeName(ControlMode m) {
  switch (m) {
    case ControlMode::Idle:     return "idle";
    case ControlMode::Torque:   return "torque";
    case ControlMode::Velocity: return "velocity";
    case ControlMode::Position: return "position";
  }
  return "?";
}

} // namespace fockit
