#pragma once
// FocKit 适配层 —— StudioBridge：SimpleFOC Studio 上位机会话桥
//
// 协议（对齐官方 16 课例程）：Commander 'M' 全权委托电机命令集
// （目标/模式/PID/限幅，见 docs.simplefoc.com）+ motor.monitor 数据流
// （_MON_TARGET | _MON_VEL | _MON_ANGLE，downsample 初始静默由 Studio 端开启）。
//
// 约束与纪律：
// 1. Commander/SimpleFOC 类型只允许存在于本文件与 SimpleFocMotor（契约边界，ARC-01 §2）；
// 2. Studio 在线改的 PID/限幅是 RAM 值，重启即失——调好后必须抄回代码配置，
//    NVS 只固化编码器标定（这是定位：临时调参会话，不是参数存储通道）；
// 3. 单实例假设：一个会话一个桥。

#include <SimpleFOC.h>

#include "core/ISerialSession.h"
#include "adapter/SimpleFocMotor.h"

namespace fockit {

class StudioBridge : public ISerialSession {
public:
  explicit StudioBridge(Stream& port = Serial) : cmd_(port) {}

  /// 绑定电机并注册 Commander 通道
  void begin(SimpleFocMotor* m) {
    motor_ = m;
    self_ = this;
    BLDCMotor& bm = m->rawMotor();
    bm.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
    bm.monitor_downsample = 0;  // 初始静默，由 Studio 端开启数据流
    cmd_.add('M', StudioBridge::onMotorCmd_, "motor");
  }

  // ISerialSession：会话期独占串口收发
  void update() override {
    if (motor_ == nullptr) return;
    cmd_.run();
    motor_->rawMotor().monitor();
  }

private:
  // Commander 回调是裸函数指针，用单实例静态指针转发
  static void onMotorCmd_(char* cmd) {
    if (self_ != nullptr && self_->motor_ != nullptr) {
      self_->cmd_.motor(&self_->motor_->rawMotor(), cmd);
    }
  }
  static StudioBridge* self_;

  Commander cmd_;
  SimpleFocMotor* motor_ = nullptr;
};

} // namespace fockit
