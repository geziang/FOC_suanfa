#pragma once
// FocKit 中间件 —— 位置环节点（三环级联的外层）
//
// 结构：位置PID → 速度目标 → VelocityNode（速度PID）→ 力矩目标 → IMotor。
// 级联输出受电机限幅约束：位置环出速度（≤maxVelocity），速度环出力矩（≤maxTorque）。

#include <stdint.h>
#include "core/IMotor.h"
#include "control/PID.h"
#include "control/VelocityNode.h"

namespace fockit {

class PositionNode {
public:
  void begin(IMotor* m, float kp = 5.0f, float ki = 0.0f, float kd = 0.05f) {
    m_ = m;
    pid_.setGains(kp, ki, kd);
    vel_.begin(m);  // 内层速度环用默认增益，可通过 velocity() 单独整定
  }

  void setTarget(float rad) {
    if (m_ == nullptr) return;
    MotorLimits lim = m_->getLimits();
    if (rad < lim.minPosition) rad = lim.minPosition;
    if (rad > lim.maxPosition) rad = lim.maxPosition;
    target_ = rad;
  }
  float target() const { return target_; }

  void setGains(float kp, float ki, float kd) { pid_.setGains(kp, ki, kd); }
  PID& pid() { return pid_; }
  VelocityNode& velocity() { return vel_; }  // 内层速度环（整定入口）

  void update() {
    uint32_t now = micros();
    float dt = started_ ? (now - lastUs_) * 1e-6f : 0.0f;
    lastUs_ = now;
    started_ = true;
    if (m_ == nullptr || dt <= 0.0f) return;

    pid_.setOutputLimit(m_->getLimits().maxVelocity);  // 外层出速度目标
    float v = pid_.update(target_, m_->getAngle(), dt);
    vel_.setTarget(v);
    vel_.update();
  }

private:
  IMotor* m_ = nullptr;
  PID pid_;
  VelocityNode vel_;
  float target_ = 0.0f;
  uint32_t lastUs_ = 0;
  bool started_ = false;
};

} // namespace fockit
