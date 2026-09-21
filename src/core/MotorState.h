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

/// 环类型（整定配置用）
enum class LoopType : uint8_t { CurrentQ, CurrentD, Velocity, Position };

/// 环增益包（整定配置用）。
/// 计算优先（SPEC-T §1）：初值来自建模与带宽计算，上机只在计算值附近微调；
/// 未提供的项保持 -1（不变更）。
struct LoopGains {
  float kp;
  float ki;
  float kd;
  float lpfTf;        // 反馈低通时间常数 [s]
  float outputLimit;  // PID 输出限幅；级联外环 = 内环指令红线（如速度环 0.5A）。
                      // 2026-09-21 级联版速度环引入：速度 PID 输出即 Iq 指令，
                      // 必须钳在持续电流红线，电压限幅 4.125V 不再适用。
  LoopGains(float kp = -1.0f, float ki = -1.0f, float kd = -1.0f,
            float lpfTf = -1.0f, float outputLimit = -1.0f)
      : kp(kp), ki(ki), kd(kd), lpfTf(lpfTf), outputLimit(outputLimit) {}
};

} // namespace fockit
