#pragma once
// FocKit 适配层 —— MotorManager：双电机切换控制
// P0 需求是“不做同步双控，但能一键切换受控对象”（docs/00 §2）。

#include "core/IMotor.h"

namespace fockit {

class MotorManager {
public:
  static constexpr int kSlots = 2;  // DengFOC V4 双路

  bool attach(int slot, IMotor* m) {
    if (slot < 0 || slot >= kSlots || m == nullptr) return false;
    motors_[slot] = m;
    return true;
  }

  /// 切换当前受控电机：先失能旧电机（安全），再返回新电机
  IMotor* select(int slot) {
    if (slot < 0 || slot >= kSlots || motors_[slot] == nullptr) return active_;
    if (active_ != nullptr && active_ != motors_[slot]) active_->disable();
    active_ = motors_[slot];
    activeSlot_ = slot;
    return active_;
  }

  IMotor* active() { return active_; }
  int activeSlot() const { return activeSlot_; }
  int count() const {
    int n = 0;
    for (int i = 0; i < kSlots; ++i)
      if (motors_[i] != nullptr) ++n;
    return n;
  }

  /// 周期任务：只刷新当前受控电机（P0 语义）
  void update() {
    if (active_ != nullptr) active_->update();
  }

private:
  IMotor* motors_[kSlots] = {nullptr, nullptr};
  IMotor* active_ = nullptr;
  int activeSlot_ = -1;
};

} // namespace fockit
