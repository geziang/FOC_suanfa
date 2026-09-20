#pragma once
// FocKit 板级层 —— 母线电压监测与欠压保护
// 行为对齐官方例程 board_check()：欠压多次确认后置故障，恢复后自动解除。

#include <Arduino.h>
#include "bsp/DengFocBoard.h"

namespace fockit {

class PowerMonitor {
public:
  /// @param underVolt  欠压阈值 [V]，默认板级 11.1V
  /// @param periodMs   周期检测间隔
  void begin(float underVolt = dengfoc_v4::VIN_UNDERVOLT, uint32_t periodMs = 1000);

  /// 上电阻塞等待电压就绪（打印等待信息，官方例程同款流程），用于 setup()
  void waitReady();

  /// 非阻塞周期检测。欠压时 ok() 变 false —— 上层典型用法：
  ///   power.update(); if (!power.ok()) motor.disable();
  void update();

  /// 周期探针开关（默认关）：开时每个检测周期打印 readVin/threshold/ok 两行详细日志。
  /// 由 `dbg on`（SerialShell 的 verbose 钩子）打开，仅用于排查；
  /// 稳态保持静默 —— 周期打印收敛为应用层 1 Hz 心跳一行（DD-04 §4.3 节流）。
  void setPeriodicVerbose(bool on) { periodicVerbose_ = on; }
  bool periodicVerbose() const { return periodicVerbose_; }

  float voltage() const { return lastVin_; }
  bool ok() const { return ok_; }

private:
  float underVolt_ = dengfoc_v4::VIN_UNDERVOLT;
  uint32_t periodMs_ = 1000;
  uint32_t lastMs_ = 0;
  float lastVin_ = 0.0f;
  bool ok_ = true;
  bool periodicVerbose_ = false;
};

} // namespace fockit
