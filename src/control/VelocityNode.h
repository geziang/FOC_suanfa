#pragma once
// FocKit 中间件 —— 速度环节点（中间件三环的中间层）
//
// 结构：目标速度 vs 反馈速度 → PID → 力矩目标 → IMotor（力矩模式）。
// 这是“中间件路径”：三环中的速度环由本库实现；
// IMotor 内置的 SimpleFOC 速度环仅作为对照组（docs/00 §3）。

#include <stdint.h>
#include "core/IMotor.h"
#include "control/PID.h"

namespace fockit {

class VelocityNode {
public:
  /// 绑定电机并切入力矩模式。默认增益由官方 2208 电压域基线换算：
  //   P=0.021 V/(rad/s)、I=0.12 V/(rad/s·s)，× KT/R = ×0.0827/8.25 ≈ ×0.01（N·m 域）
  void begin(IMotor* m, float kp = 2.1e-4f, float ki = 1.2e-3f, float kd = 0.0f) {
    m_ = m;
    pid_.setGains(kp, ki, kd);
    if (m_ != nullptr) m_->setMode(ControlMode::Torque);
  }

  void setTarget(float radPerSec) {
    if (m_ == nullptr) return;
    MotorLimits lim = m_->getLimits();
    target_ = (radPerSec > lim.maxVelocity) ? lim.maxVelocity
            : (radPerSec < -lim.maxVelocity) ? -lim.maxVelocity
                                             : radPerSec;
  }
  float target() const { return target_; }

  void setGains(float kp, float ki, float kd) { pid_.setGains(kp, ki, kd); }
  PID& pid() { return pid_; }

  /// 每个控制拍调用（与 motor->update() 同拍或更高频率）
  void update() {
    uint32_t now = micros();
    float dt = started_ ? (now - lastUs_) * 1e-6f : 0.0f;
    lastUs_ = now;
    started_ = true;
    if (m_ == nullptr || dt <= 0.0f) return;

    pid_.setOutputLimit(m_->getLimits().maxTorque);  // 力矩限幅随电机限幅
    out_ = pid_.update(target_, m_->getVelocity(), dt);
    m_->setTorqueTarget(out_);
  }

  float output() const { return out_; }

private:
  IMotor* m_ = nullptr;
  PID pid_;
  float target_ = 0.0f;
  float out_ = 0.0f;
  uint32_t lastUs_ = 0;
  bool started_ = false;
};

} // namespace fockit
