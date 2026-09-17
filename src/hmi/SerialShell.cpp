#include "hmi/SerialShell.h"
#include <stdlib.h>
#include <string.h>

namespace fockit {

void SerialShell::begin(IMotor* motor, Stream& port) {
  motor_ = motor;
  port_ = &port;
  port_->println(F("FocKit shell 就绪，输入 help 查看命令"));
}

void SerialShell::update() {
  if (port_ == nullptr || motor_ == nullptr) return;

  // Studio 会话期：串口处理权让位给会话桥（Commander 独占解析）
  if (studioMode_) {
    if (studio_ != nullptr) studio_->update();
    return;
  }

  while (port_->available() > 0) {
    char c = (char)port_->read();
    if (c == '\n' || c == '\r') {
      if (len_ > 0) dispatch_();
      len_ = 0;
      buf_[0] = 0;
    } else if (len_ < (int)(sizeof(buf_) - 1)) {
      buf_[len_++] = c;
      buf_[len_] = 0;
    }
  }

  if (streaming_ && millis() - lastStreamMs_ >= 100) {  // 10Hz 状态流
    lastStreamMs_ = millis();
    printState_(false);
  }
}

void SerialShell::dispatch_() {
  char* argv[6];
  int argc = 0;
  char* tok = strtok(buf_, " \t");
  while (tok != nullptr && argc < 6) {
    argv[argc++] = tok;
    tok = strtok(nullptr, " \t");
  }
  if (argc == 0) return;
  for (char* p = argv[0]; *p != 0; ++p) *p = (char)tolower(*p);

  const char* cmd = argv[0];

  if (!strcmp(cmd, "help")) {
    printHelp_();
  } else if (!strcmp(cmd, "on")) {
    motor_->enable();
  } else if (!strcmp(cmd, "off")) {
    motor_->disable();
  } else if (!strcmp(cmd, "idle")) {
    motor_->setMode(ControlMode::Idle);
    port_->println(F("已切空载（保持使能，零输出）"));
  } else if (!strcmp(cmd, "t") && argc >= 2) {
    motor_->setTorqueTarget(strtof(argv[1], nullptr));
  } else if (!strcmp(cmd, "v") && argc >= 2) {
    float v = strtof(argv[1], nullptr);
    if (vel_ != nullptr) vel_->setTarget(v);          // 中间件路径
    else motor_->setVelocityTarget(v);                // 内置对照组路径
  } else if (!strcmp(cmd, "p") && argc >= 2) {
    float p = strtof(argv[1], nullptr);
    if (pos_ != nullptr) pos_->setTarget(p);
    else motor_->setPositionTarget(p);
  } else if (!strcmp(cmd, "pid") && argc >= 5) {
    float kp = strtof(argv[2], nullptr);
    float ki = strtof(argv[3], nullptr);
    float kd = strtof(argv[4], nullptr);
    if (argv[1][0] == 'v' && vel_ != nullptr) {
      vel_->setGains(kp, ki, kd);
      port_->println(F("速度环增益已更新"));
    } else if (argv[1][0] == 'p' && pos_ != nullptr) {
      pos_->setGains(kp, ki, kd);
      port_->println(F("位置环增益已更新"));
    } else {
      port_->println(F("未绑定对应节点（见 attachVelocityNode/attachPositionNode）"));
    }
  } else if (!strcmp(cmd, "st")) {
    printState_(true);
  } else if (!strcmp(cmd, "stream")) {
    streaming_ = !streaming_;
    port_->println(streaming_ ? F("状态流开（10Hz，串口绘图器可用）")
                               : F("状态流关"));
  } else if (!strcmp(cmd, "save")) {
    motor_->saveCalibration();
  } else if (!strcmp(cmd, "studio")) {
    if (studio_ != nullptr) {
      streaming_ = false;
      studioMode_ = true;
      port_->println(F("进入 SimpleFOC Studio 会话（独占串口），可在上位机连接 115200"));
      port_->println(F("退出：按板上 EN/RST 复位；调好的增益请抄回代码（Studio 改动不落 NVS）"));
    } else {
      port_->println(F("未绑定 StudioBridge（见 attachStudio）"));
    }
  } else if (!strcmp(cmd, "sel") && argc >= 2 && mgr_ != nullptr) {
    IMotor* m = mgr_->select(atoi(argv[1]));
    if (m != nullptr) {
      motor_ = m;
      port_->println(F("已切换受控电机（注意：已绑定的PID节点仍指向原电机）"));
    }
  } else if (userCb_ != nullptr && userCb_(argc, argv)) {
    // 用户自定义命令已消费（处理器自行打印反馈）
  } else {
    port_->println(F("未知命令，输入 help 查看"));
  }
}

void SerialShell::printHelp_() {
  port_->println(F("命令（目标指令会自动切模式；失能后先 on）："));
  port_->println(F("  on / off        使能 / 失能"));
  port_->println(F("  idle            空载（保持使能，零输出）"));
  port_->println(F("  t <N·m>         力矩目标，如 t 0.01"));
  port_->println(F("  v <rad/s>       速度目标（绑定了节点走中间件环，否则走内置环）"));
  port_->println(F("  p <rad>         位置目标"));
  port_->println(F("  pid v <kp> <ki> <kd>  整定中间件速度环"));
  port_->println(F("  pid p <kp> <ki> <kd>  整定中间件位置环"));
  port_->println(F("  st              打印一次状态"));
  port_->println(F("  stream          开关 10Hz 状态流"));
  port_->println(F("  save            固化标定到 NVS"));
  port_->println(F("  studio          进入 SimpleFOC Studio 上位机会话（退出按复位）"));
  port_->println(F("  sel <0|1>       切换受控电机（需绑定 MotorManager）"));
}

void SerialShell::printState_(bool withHeader) {
  MotorState s = motor_->getState();
  if (withHeader) {
    port_->println(F("angle[rad], vel[rad/s], target, iq[A], torque[N·m], mode, en"));
  }
  char line[96];
  snprintf(line, sizeof(line), "%.3f, %.3f, %.3f, %.3f, %.4f, %s, %d",
           (double)s.angle, (double)s.velocity, (double)motor_->getTarget(),
           (double)s.iq, (double)s.torque, controlModeName(s.mode),
           s.enabled ? 1 : 0);
  port_->println(line);
}

} // namespace fockit
