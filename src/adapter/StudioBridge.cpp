#include "adapter/StudioBridge.h"

namespace fockit {

// 单实例静态指针（Commander 回调为裸函数指针）
StudioBridge* StudioBridge::self_ = nullptr;

void StudioBridge::setTrace(bool on) {
  trace_ = on;
  if (!dbgPort_) return;
  dbgPort_->print(F("[FW STUDIO] 调试探针已"));
  dbgPort_->println(on ? F("开启：逐条命令回显 + 1s 状态快照")
                       : F("关闭：仅保留按钮设置确认"));
}

void StudioBridge::update() {
  if (motor_ == nullptr) {
    if (dbgPort_) dbgPort_->println(F("[FW STUDIO] update skipped: motor is null"));
    return;
  }
  ++updateCount_;
  bool serialPending = dbgPort_ && dbgPort_->available() > 0;
  if (trace_ && serialPending) dbgPort_->println(F("[FW STUDIO] cmd.run enter"));
  cmd_.run();
  if (trace_ && serialPending) dbgPort_->println(F("[FW STUDIO] cmd.run returned"));
  // 曲线流限速闸（2026-09-20）：两次 monitor() 放行最小间隔 10ms → 硬上限 ~100 行/秒。
  // 主循环可达 ~15kHz，仅靠 Studio 的降采样参数挡不住激进配置（100 时需求 ~150 行/s，
  // 贴着 115200 天花板）；TX 通道打满时 HardwareSerial 会阻塞调用线程（即 FOC 主循环）。
  // 闸门保证无论界面怎么配，曲线流都不会压住主循环与心跳/应答。
  uint32_t nowMs = millis();
  if (nowMs - lastMonitorGateMs_ >= 10) {
    lastMonitorGateMs_ = nowMs;
    motor_->rawMotor().monitor();
  }

  // 测速链诊断探针（2026-09-21，速度环失控案）：速度/位置模式期间 500ms 一行。
  // 只读无副作用字段（不调 getVelocity——它有内部状态副作用）。
  // 判读：相邻两行 (rot*2π+ang) 差 / 0.5s = 真实角速度，与 sv 对比即知测速链好坏；
  // lim 应=4.125、mds 应=0、P/I 应=速度环写入值——偏离即 RAM 污染。
  if (nowMs - lastVelDbgMs_ >= 500) {
    lastVelDbgMs_ = nowMs;
    const BLDCMotor& bm = motor_->rawMotor();
    if (bm.controller == MotionControlType::velocity ||
        bm.controller == MotionControlType::angle) {
      const float mechAngle =
          bm.sensor != nullptr ? bm.sensor->getMechanicalAngle() : 0.0f;
      const long fullRot =
          bm.sensor != nullptr ? (long)bm.sensor->getFullRotations() : 0;
      dbgPort_->printf(
          "[VEL DBG] ctrl=%d tt=%d sv=%.3f ang=%.3f rot=%ld vq=%.3f",
          (int)bm.controller, (int)bm.torque_controller,
          (double)bm.shaft_velocity, (double)mechAngle, fullRot,
          (double)bm.voltage.q);
      dbgPort_->printf(" P=%.4f I=%.4f lim=%.3f mds=%d\n",
                       (double)bm.PID_velocity.P, (double)bm.PID_velocity.I,
                       (double)bm.PID_velocity.limit,
                       (int)bm.motion_downsample);
    }
  }
  if (trace_ && dbgPort_) {
    uint32_t now = millis();
    if (now - lastUpdateHeartbeatMs_ >= 1000) {
      lastUpdateHeartbeatMs_ = now;
      dbgPort_->print(F("[FW STUDIO] update heartbeat count="));
      dbgPort_->println(updateCount_);
    }
  }
  if (trace_) {
    uint32_t now = millis();
    if (now - lastSnapMs_ >= 1000) {
      lastSnapMs_ = now;
      printSnapshot_();
    }
  }
}

void StudioBridge::onMotorCmd_(char* cmd) {
  if (self_ == nullptr) return;
  // Commander 回调只传"去掉注册 ID"之后的命令体（Commander.cpp run(): &user_input[1]）
  self_->handleCmd_(cmd);
}

bool StudioBridge::isCmdLetter_(char c) {
  // commands.h 的 CMD_* 全集；'?' / '@' / '#' 是 Commander 自身的扫描/输出模式命令，
  // 不经电机命令集，故不列入（它们不该出现在本会话的协议流里）。
  switch (c) {
    case CMD_C_D_PID:      // 'D'
    case CMD_C_Q_PID:      // 'Q'
    case CMD_V_PID:        // 'V'
    case CMD_A_PID:        // 'A'
    case CMD_STATUS:       // 'E'
    case CMD_LIMITS:       // 'L'
    case CMD_MOTION_TYPE:  // 'C'
    case CMD_TORQUE_TYPE:  // 'T'
    case CMD_SENSOR:       // 'S'
    case CMD_MONITOR:      // 'M'
    case CMD_RESIST:       // 'R'
    case CMD_PWMMOD:       // 'W'
      return true;
    default:
      return false;
  }
}

void StudioBridge::handleLine(char* line) {
  // shell 转交的是整行（含注册 ID 与行尾 eol）。宽容解析两种上位机约定（详见头注释）：
  //   已配 ID 'M' → 线缆为 `M<命令体>`，需剥掉首位 ID 再交 Commander（官方语义）
  //   ID 为空串   → 线缆就是命令体本身，整行投递
  // 不做字符猜测：用两条铁证判定并自我修正，跨越"上位机中途改配置"也无需复位。
  if (line == nullptr || line[0] == 0) return;

  // 铁证 ①：`MM…` 只能是 ID('M') + 命令体('M…')——命令体没有 "MM" 形式。
  if (line[0] == kCmdId && line[1] == kCmdId) {
    idAbsent_ = false;
  }
  // 铁证 ②：非 'M' 打头的命令字母行只能是裸命令体——配了 ID 的行必以 'M' 开头。
  else if (line[0] != kCmdId && isCmdLetter_(line[0])) {
    idAbsent_ = true;
  }

  const bool stripId = (line[0] == kCmdId) && !idAbsent_;
  char* body = stripId ? &line[1] : line;

  // 收敛后仍不是合法命令体（首位既非命令字母、也非数字/正负号）→ 静默丢弃，
  // 与 Commander 的分发语义一致：无人认领不制造 "unknown cmd err"（那正是刷屏的源头）。
  if (!isCmdLetter_(body[0]) && !isDigit((int)body[0]) && body[0] != '-' && body[0] != '+') return;

  if (trace_ && dbgPort_) {
    dbgPort_->print(F("[FW STUDIO] handleLine id="));
    dbgPort_->print(stripId ? F("stripped") : (idAbsent_ ? F("absent") : F("none")));
    dbgPort_->print(F(" body='"));
    dbgPort_->print(body);
    dbgPort_->println(F("'"));
  }
  handleCmd_(body);
}

void StudioBridge::handleCmd_(char* cmd) {
  if (motor_ == nullptr) {
    if (dbgPort_) dbgPort_->println(F("[FW STUDIO] command dropped: motor is null"));
    return;
  }

  // 先拷贝可打印副本：Commander.target() 内部 strtok 会破坏原串
  char raw[24];
  size_t i = 0;
  for (; cmd[i] != 0 && cmd[i] != '\n' && cmd[i] != '\r' && i + 1 < sizeof(raw); ++i) {
    raw[i] = cmd[i];
  }
  raw[i] = 0;

  bool isGet = isGetCmd_(raw);
  if (trace_ && dbgPort_) {
    dbgPort_->print(F("[FW STUDIO] callback received raw='M"));
    dbgPort_->print(raw);
    dbgPort_->println(F("'"));
    dbgPort_->print(F("[FW STUDIO] command class="));
    dbgPort_->println(isGet ? F("get") : F("set"));
    dbgPort_->println(F("[FW STUDIO] forwarding to SimpleFOC"));
  }
  cmd_.motor(&motor_->rawMotor(), cmd);  // 先执行（生效）
  if (trace_ && dbgPort_) dbgPort_->println(F("[FW STUDIO] forwarding returned"));

  // Studio 的 MC 直通 SimpleFOC 不更新 FocKit 的 mode_（EXP-02/REF-12 §6.2
  // "直通绕过防御"家族）——不回写则 Idle 覆写分支在速度/位置模式下仍每拍
  // 改写 shaft_velocity，与 move() 双路径同写。此处只同步枚举，不触碰
  // controller/torque_controller（MT 力矩类型由 Studio 命令自己管）。
  if (raw[0] == 'C' && raw[1] >= '0' && raw[1] <= '4') {
    motor_->syncStudioControlMode(raw[1] - '0');
  }

  if (!isGet) describeSet_(raw);  // 再回读，打印确认
}

bool StudioBridge::isGetCmd_(const char* r) {
  char a = r[0];
  if (a == 0) return true;                                  // 裸 M：无操作
  if (isDigit((int)a) || a == '-' || a == '+') return false;  // 目标设置
  char b = r[1];
  switch (a) {
    case 'Q': case 'D': case 'V': case 'A':  // PID/LPF：<组><子>[值]
      return b == 0 || r[2] == 0;
    case 'L':                                // 限幅：LV/LU/LC
      return b == 0 || r[2] == 0;
    case 'C':                                // C 模式；CD 控制环下采样
      if (b == 'D') return r[2] == 0;
      return b == 0;                         // C0..C4 为设置
    case 'T': case 'E': case 'R':            // 后随数字即设置
      return b == 0;
    case 'S':                                // SM/SE 传感器零点
      return b == 0 || r[2] == 0;
    case 'M':                                // monitor：MGx 单次查询；MC 清屏动作
      if (b == 'G') return true;
      if (b == 'C') return false;
      return b == 0 || r[2] == 0;            // MD/MS
    case 'W':                                // WT/WC 调制
      return b == 0 || r[2] == 0;
    default:
      return true;
  }
}

void StudioBridge::describeSet_(const char* r) {
  if (!dbgPort_ || !motor_) return;
  BLDCMotor& m = motor_->rawMotor();
  char a = r[0];

  // 目标值（数字开头，当前是什么控制环就设什么目标）
  if (isDigit((int)a) || a == '-' || a == '+') {
    dbgPort_->printf("[Studio] 目标 <- %.3f（控制：%s）\n",
                     (double)m.target, modeName_(m.controller));
    return;
  }

  switch (a) {
    case 'Q': case 'D': case 'V': case 'A': {
      const char* grp;
      PIDController* pid;
      LowPassFilter* lpf;
      if (a == 'Q') {
        grp = "电流Q环"; pid = &m.PID_current_q; lpf = &m.LPF_current_q;
      } else if (a == 'D') {
        grp = "电流D环"; pid = &m.PID_current_d; lpf = &m.LPF_current_d;
      } else if (a == 'V') {
        grp = "速度环"; pid = &m.PID_velocity; lpf = &m.LPF_velocity;
      } else {
        grp = "位置环"; pid = &m.P_angle; lpf = &m.LPF_angle;
      }
      if (r[1] == 'F') {  // 滤波时间常数
        dbgPort_->printf("[Studio] %s 滤波时间 <- %.4f s\n", grp, (double)lpf->Tf);
        return;
      }
      float v = 0.0f;
      switch (r[1]) {
        case 'P': v = pid->P; break;
        case 'I': v = pid->I; break;
        case 'D': v = pid->D; break;
        case 'R': v = pid->output_ramp; break;
        case 'L': v = pid->limit; break;
        default: return;
      }
      dbgPort_->printf("[Studio] %s %s <- %.4f\n", grp, pidParamName_(r[1]), (double)v);
      break;
    }
    case 'L':
      if (r[1] == 'V') {
        dbgPort_->printf("[Studio] 速度限幅 <- %.3f\n", (double)m.velocity_limit);
      } else if (r[1] == 'U') {
        dbgPort_->printf("[Studio] 电压限幅 <- %.3f V\n", (double)m.voltage_limit);
      } else if (r[1] == 'C') {
        dbgPort_->printf("[Studio] 电流限幅 <- %.3f A\n", (double)m.current_limit);
      }
      break;
    case 'C':
      if (r[1] == 'D') {
        dbgPort_->printf("[Studio] 控制环下采样 <- %u\n", m.motion_downsample);
      } else {
        dbgPort_->printf("[Studio] 控制模式 <- %s\n", modeName_(m.controller));
      }
      break;
    case 'T':
      dbgPort_->printf("[Studio] 力矩模式 <- %s\n", torqueName_(m.torque_controller));
      break;
    case 'E':
      dbgPort_->printf("[Studio] 电机已%s\n", m.enabled ? "使能" : "失能");
      break;
    case 'R':
      dbgPort_->printf("[Studio] 相电阻 <- %.4f Ω\n", (double)m.phase_resistance);
      break;
    case 'S':
      if (r[1] == 'M') {
        dbgPort_->printf("[Studio] 传感器机械零点 <- %.3f\n", (double)m.sensor_offset);
      } else if (r[1] == 'E') {
        dbgPort_->printf("[Studio] 传感器电气零位 <- %.3f\n", (double)m.zero_electric_angle);
      }
      break;
    case 'M':
      if (r[1] == 'D') {
        dbgPort_->printf("[Studio] 曲线下采样 <- %u\n", m.monitor_downsample);
      } else if (r[1] == 'S') {
        dbgPort_->printf("[Studio] 曲线变量位图 <- 0x%02X\n",
                         (unsigned)(unsigned char)m.monitor_variables);
      } else if (r[1] == 'C') {
        dbgPort_->println(F("[Studio] 曲线缓存已清零"));
      }
      break;
    case 'W':
      if (r[1] == 'T') {
        dbgPort_->printf("[Studio] PWM调制方式 <- %d\n", (int)m.foc_modulation);
      } else if (r[1] == 'C') {
        dbgPort_->printf("[Studio] PWM中心对齐 <- %d\n", (int)m.modulation_centered);
      }
      break;
    default:
      break;
  }
}

void StudioBridge::printSnapshot_() {
  if (!dbgPort_ || !motor_) return;
  BLDCMotor& m = motor_->rawMotor();
  dbgPort_->printf(
      "[Studio] 快照 使能=%d 控制=%s 力矩=%s 目标=%.3f 限速=%.2f 限压=%.2f 限流=%.3f 曲线分频=%u\n",
      (int)m.enabled, modeName_(m.controller), torqueName_(m.torque_controller),
      (double)m.target, (double)m.velocity_limit, (double)m.voltage_limit,
      (double)m.current_limit, m.monitor_downsample);
}

const char* StudioBridge::modeName_(MotionControlType c) {
  switch (c) {
    case MotionControlType::torque: return "力矩";
    case MotionControlType::velocity: return "速度";
    case MotionControlType::angle: return "位置";
    case MotionControlType::velocity_openloop: return "速度开环";
    case MotionControlType::angle_openloop: return "位置开环";
    default: return "未知";
  }
}

const char* StudioBridge::torqueName_(TorqueControlType t) {
  switch (t) {
    case TorqueControlType::voltage: return "电压";
    case TorqueControlType::dc_current: return "直流电流";
    case TorqueControlType::foc_current: return "FOC电流";
    default: return "未知";
  }
}

const char* StudioBridge::pidParamName_(char sub) {
  switch (sub) {
    case 'P': return "比例P";
    case 'I': return "积分I";
    case 'D': return "微分D";
    case 'R': return "输出斜坡";
    case 'L': return "输出限幅";
    default: return "参数";
  }
}

} // namespace fockit
