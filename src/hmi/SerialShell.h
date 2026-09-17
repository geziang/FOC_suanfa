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

  void update();

private:
  void dispatch_();
  void printHelp_();
  void printState_(bool withHeader);

  IMotor* motor_ = nullptr;
  Stream* port_ = nullptr;
  VelocityNode* vel_ = nullptr;
  PositionNode* pos_ = nullptr;
  MotorManager* mgr_ = nullptr;
  ISerialSession* studio_ = nullptr;
  bool studioMode_ = false;

  char buf_[48] = {0};
  int len_ = 0;
  bool streaming_ = false;
  uint32_t lastStreamMs_ = 0;
};

} // namespace fockit
