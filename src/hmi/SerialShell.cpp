#include "hmi/SerialShell.h"
#include <stdlib.h>
#include <string.h>

namespace fockit {

void SerialShell::begin(IMotor* motor, Stream& port) {
  motor_ = motor;
  port_ = &port;
  port_->println(F("[FW SHELL] begin: motor/serial bound"));
  port_->println(F("FocKit shell 就绪，输入 help 查看命令"));
}

void SerialShell::update() {
  if (port_ == nullptr || motor_ == nullptr) return;

  // Studio 会话期：串口处理权让位给会话桥（Commander 独占解析）
  if (studioMode_) {
    if (studio_ != nullptr) {
      studio_->update();  // 存活心跳由应用层 1 行/秒的 [FW LOOP] alive 承担，此处不再刷行
    } else {
      port_->println(F("[FW SHELL] studio mode set but bridge is null"));
    }
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
  // strtok 与随后的小写化都会改写 buf_，先留存原始行，供未知命令回显/大写协议检测
  char rawLine[48];
  strncpy(rawLine, buf_, sizeof(rawLine) - 1);
  rawLine[sizeof(rawLine) - 1] = 0;
  bool upperHead = (rawLine[0] >= 'A' && rawLine[0] <= 'Z');

  char* tok = strtok(buf_, " \t");
  while (tok != nullptr && argc < 6) {
    argv[argc++] = tok;
    tok = strtok(nullptr, " \t");
  }
  if (argc == 0) return;
  for (char* p = argv[0]; *p != 0; ++p) *p = (char)tolower(*p);

  const char* cmd = argv[0];

  // ── 自动接管（2026-09-20）：上位机协议命令不再换回"未知命令"5 行 ──
  // 1) 本拍内已让位：同一批到达的后续行一律转交会话，shell 不再插话；
  // 2) 大写开头且不是 shell 自己的命令字 = 上位机协议命令（绑定/轮询/pull config）→
  //    自动进会话并转交。判别依据：本 shell 命令集全为小写，大写要么是人工误触 CapsLock
  //    （命令字仍在白名单内，照常走 shell），要么就是协议命令。
  if (studioMode_) {
    autoEnterStudio_(rawLine);
    return;
  }
  if (upperHead && !isShellCommand_(cmd)) {
    autoEnterStudio_(rawLine);
    return;
  }

  port_->print(F("[FW SHELL] dispatch raw='"));
  port_->print(rawLine);
  port_->print(F("' argc="));
  port_->println(argc);

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
    port_->println(F("[FW SHELL] entering Studio mode"));
    if (studio_ != nullptr) {
      streaming_ = false;
      studioMode_ = true;
      port_->println(F("进入 SimpleFOC Studio 会话（独占串口），可在上位机连接 115200"));
      port_->println(F("退出：按板上 EN/RST 复位；调好的增益请抄回代码（Studio 改动不落 NVS）"));
      port_->println(F("[FW SHELL] studio mode enabled; waiting for Commander M..."));
    } else {
      port_->println(F("未绑定 StudioBridge（见 attachStudio）"));
      port_->println(F("[FW SHELL] studio attach is null"));
    }
  } else if (!strcmp(cmd, "dbg") && argc >= 2) {
    bool on = (!strcmp(argv[1], "on") || !strcmp(argv[1], "1"));
    if (studio_ != nullptr) {
      studio_->setTrace(on);
    } else {
      port_->println(F("未挂载上位机会话（attachStudio 为空）"));
    }
    if (verboseCb_ != nullptr) verboseCb_(on);  // 同步固件周期探针（如 PowerMonitor）
    port_->println(on ? F("详细日志已开（周期探针 1s 逐条输出）")
                      : F("详细日志已关（常态仅 1 行/秒心跳）"));
  } else if (!strcmp(cmd, "sel") && argc >= 2 && mgr_ != nullptr) {
    IMotor* m = mgr_->select(atoi(argv[1]));
    if (m != nullptr) {
      motor_ = m;
      port_->println(F("已切换受控电机（注意：已绑定的PID节点仍指向原电机）"));
    }
  } else if (userCb_ != nullptr && userCb_(argc, argv)) {
    // 用户自定义命令已消费（处理器自行打印反馈）
  } else {
    // 节流（2026-09-20）：未知命令回执每秒最多一次。命令源异常（未进会话的上位机、
    // 噪声、脚本）连发时不再每条换回 2~5 行，只在窗口边界报告一次并给出被压掉的数量。
    uint32_t now = millis();
    bool windowOpen = (lastUnknownMs_ != 0) && (now - lastUnknownMs_ < 1000);
    if (windowOpen) {
      ++unknownSuppressed_;
    } else {
      if (unknownSuppressed_ > 0) {
        port_->printf("[FW SHELL] 上一秒内另有 %lu 条未知命令被静默节流\n",
                      (unsigned long)unknownSuppressed_);
      }
      unknownSuppressed_ = 0;
      lastUnknownMs_ = now;
      port_->print(F("未知命令："));
      port_->println(rawLine);
      // 大写字母开头是 SimpleFOC Studio 协议命令的特征（MC/MVP/ME1…）。
      // 正常已由自动接管消化；走到这里说明它恰好与 shell 命令字重名（如 T 缺参数），
      // 或会话桥未挂载（attachStudio 为空）。
      if (upperHead) {
        port_->println(F("[Studio] 该大写命令未被识别（与 shell 命令字重名或缺参数）"));
        port_->println(F("[Studio] 处理：「设备」页连接按钮旁的[命令ID:] 框须填 M"));
        port_->println(F("[Studio] 注意：「设置」弹窗里的[连接ID]不下发，填它无效"));
      } else {
        port_->println(F("输入 help 查看全部命令"));
      }
    }
  }
}

bool SerialShell::isShellCommand_(const char* cmd) {
  static const char* kOwn[] = {
      "help", "on", "off", "idle", "t", "v", "p", "pid",
      "st", "stream", "save", "dbg", "sel", "studio",
  };
  for (unsigned i = 0; i < sizeof(kOwn) / sizeof(kOwn[0]); ++i) {
    if (!strcmp(cmd, kOwn[i])) return true;
  }
  return false;
}

void SerialShell::autoEnterStudio_(const char* rawLine) {
  if (studio_ == nullptr) {  // 无会话可交：一行说明即止，不回 5 行
    port_->println(F("[FW SHELL] 收到上位机协议命令，但未绑定会话（attachStudio 为空）"));
    return;
  }
  if (!studioMode_) {
    streaming_ = false;
    studioMode_ = true;
    port_->println(F("[FW SHELL] 检测到上位机协议命令，自动进入 Studio 会话（shell 让位；复位可返回）"));
  }
  // 补齐行尾 eol：Commander 的 isSentinel() 靠行尾字符判定 GET/SET（缺了就变成"写 0"），
  // 而 shell 的 buf_ 里没有换行符，必须在这里补回。
  char proto[56];
  snprintf(proto, sizeof(proto), "%s\n", rawLine);
  studio_->handleLine(proto);
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
  port_->println(F("  dbg on|off      详细日志开关（Studio 探针 + 固件周期探针）；默认关，常态仅 1 行/秒心跳"));
  port_->println(F("  studio          进入 SimpleFOC Studio 上位机会话（退出按复位）"));
  port_->println(F("  sel <0|1>       切换受控电机（需绑定 MotorManager）"));
  port_->println(F("说明：上位机协议命令（M 开头）会自动接管串口并进入会话，无需先手工输 studio。"));
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
