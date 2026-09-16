#pragma once
// FocKit 中间件 —— 通用 PID（三环级联的基础件）
// 特性：积分抗饱和（积分项贡献限幅）、对测量量微分（抑制目标突变冲击）、
//       dt 异常保护。增益按“参数辨识→公式整定”的实验室纪律使用（docs/00 §5）。

#include <stdint.h>

namespace fockit {

class PID {
public:
  PID() {}
  PID(float kp, float ki, float kd, float outputLimit = 0.0f)
      : kp_(kp), ki_(ki), kd_(kd), outLim_(outputLimit) {}

  void setGains(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
  }
  void setOutputLimit(float lim) { outLim_ = lim; }

  /// @param sp 目标值  @param fb 反馈量  @param dt 步长 [s]
  float update(float sp, float fb, float dt) {
    if (dt <= 0.0f || dt > 0.5f) return lastOut_;  // dt 异常保护

    float err = sp - fb;
    if (first_) { prevMeas_ = fb; first_ = false; }
    float dTerm = kd_ * (prevMeas_ - fb) / dt;  // 对测量量微分
    prevMeas_ = fb;

    integral_ += ki_ * err * dt;
    if (outLim_ > 0.0f) {
      // 积分项贡献不超过输出限幅（抗饱和）
      float iLim = (ki_ > 1e-12f) ? outLim_ : 0.0f;
      if (integral_ > iLim) integral_ = iLim;
      if (integral_ < -iLim) integral_ = -iLim;
    }

    float out = kp_ * err + integral_ + dTerm;
    if (outLim_ > 0.0f) {
      if (out > outLim_) out = outLim_;
      if (out < -outLim_) out = -outLim_;
    }
    lastOut_ = out;
    return out;
  }

  void reset() {
    integral_ = 0.0f;
    lastOut_ = 0.0f;
    first_ = true;
  }

  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }
  float lastOutput() const { return lastOut_; }

private:
  float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
  float outLim_ = 0.0f;     // 0 = 不限幅
  float integral_ = 0.0f;
  float prevMeas_ = 0.0f;
  float lastOut_ = 0.0f;
  bool first_ = true;
};

} // namespace fockit
