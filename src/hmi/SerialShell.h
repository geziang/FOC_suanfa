#pragma once
// FocKit 人机接口 —— 串口调参命令行（115200）
// 设计为非阻塞：update() 在 loop() 里随控制拍调用。

#include <Arduino.h>
#include "core/IMotor.h"
#include "core/ISerialSession.h"
#include "control/VelocityNode.h"
#include "control/PositionNode.h"
#include "adapter/MotorManager.h"

namespace fockit {

class SerialShell {
public:
  void begin(IMotor* motor, Stream& port = Serial);
  void attachVelocityNode(VelocityNode* v) { vel_ = v; }
  void attachPositionNode(PositionNode* p) { pos_ = p; }
  void attachManager(MotorManager* mgr) { mgr_ = mgr; }
  /// 绑定上位机会话桥（如 StudioBridge）；`studio` 命令进入会话后 shell 让位
  void attachStudio(ISerialSession* s) { studio_ = s; }
  /// 注册用户自定义命令：未识别命令以已解析的 argc/argv 转发，
  /// 返回 true 表示已处理（整定程序等示例专用，命令集由示例自定义）
  void attachUserCommand(bool (*cb)(int argc, char* argv[])) { userCb_ = cb; }
  /// 详细日志钩子：`dbg on|off` 时同步通知固件其它模块（如 PowerMonitor 周期探针）。
  /// hmi 层不依赖 bsp —— 由应用层接线（示例把钩子指向 power.setPeriodicVerbose）
  void attachVerboseHook(void (*cb)(bool)) { verboseCb_ = cb; }

  void update();

private:
  void dispatch_();
  void printHelp_();
  void printState_(bool withHeader);
  /// 是否为 shell 自己的命令字（用于区分"大写开头"里的人工命令与上位机协议命令）
  static bool isShellCommand_(const char* cmd);
  /// 自动让位：置会话态并把整行转交给会话桥（幂等：已在会话态则只转交）
  void autoEnterStudio_(const char* rawLine);

  IMotor* motor_ = nullptr;
  Stream* port_ = nullptr;
  VelocityNode* vel_ = nullptr;
  PositionNode* pos_ = nullptr;
  MotorManager* mgr_ = nullptr;
  ISerialSession* studio_ = nullptr;
  bool studioMode_ = false;
  bool (*userCb_)(int argc, char* argv[]) = nullptr;
  void (*verboseCb_)(bool) = nullptr;

  char buf_[48] = {0};
  int len_ = 0;
  bool streaming_ = false;
  uint32_t lastStreamMs_ = 0;
  uint32_t lastUnknownMs_ = 0;      // 未知命令回执节流窗口起点
  uint32_t unknownSuppressed_ = 0;  // 当前窗口内被静默压掉的条数
};

} // namespace fockit
