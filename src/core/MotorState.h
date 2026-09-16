#pragma once
// FocKit 契约层 —— 电机状态快照
// 单位与语义见 docs/00 §7 术语约定（全库 SI）。

#include <stdint.h>
#include "core/ControlMode.h"

namespace fockit {

struct MotorState {
  float angle = 0.0f;    // [rad]  机械角，多圈连续，零位=标定零位
  float velocity = 0.0f; // [rad/s] 机械角速度（LPF 后）
  float iq = 0.0f;       // [A]    q轴电流（P0 为指令/估算值，接电流采样后为测量值）
  float torque = 0.0f;   // [N·m]  估计力矩 = iq × KT
  ControlMode mode = ControlMode::Idle;
  bool enabled = false;
  uint32_t stampUs = 0;  // 快照时刻 micros()
};

/// 目标与限幅（SI 单位）。IMotor 实现内强制钳位，上层无需重复防御。
struct MotorLimits {
  float maxTorque   = 0.03f;           // [N·m]
  float maxVelocity = 30.0f;           // [rad/s]
  float minPosition = -12.566371f;     // [rad] (-4π)
  float maxPosition = 12.566371f;      // [rad] (+4π)
};

} // namespace fockit
