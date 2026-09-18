#include "adapter/StudioBridge.h"

namespace fockit {

// 单实例静态指针（Commander 回调为裸函数指针）
StudioBridge* StudioBridge::self_ = nullptr;

void StudioBridge::setTrace(bool on) {
  trace_ = on;
  if (!dbgPort_) return;
  dbgPort_->print(F("[Studio] 调试探针已"));
  dbgPort_->println(on ? F("开启：逐条命令回显 + 1s 状态快照")
                       : F("关闭：仅保留按钮设置确认"));
}

void StudioBridge::update() {
  if (motor_ == nullptr) return;
  cmd_.run();
  motor_->rawMotor().monitor();
  if (trace_) {
    uint32_t now = millis();
    if (now - lastSnapMs_ >= 1000) {
      lastSnapMs_ = now;
      printSnapshot_();
    }
  }
}

void StudioBridge::onMotorCmd_(char* cmd) {
  if (self_ == nullptr || self_->motor_ == nullptr) return;

  // 先拷贝可打印副本：Commander.target() 内部 strtok 会破坏原串
  char raw[24];
  size_t i = 0;
  for (; cmd[i] != 0 && cmd[i] != '\n' && cmd[i] != '\r' && i + 1 < sizeof(raw); ++i) {
    raw[i] = cmd[i];
  }
  raw[i] = 0;

  bool isGet = isGetCmd_(raw);
  if (self_->trace_ && self_->dbgPort_) {
    self_->dbgPort_->print(F("[Studio] 收到 M"));
    self_->dbgPort_->println(raw);
  }

  self_->cmd_.motor(&self_->motor_->rawMotor(), cmd);  // 先执行（生效）

  if (!isGet) self_->describeSet_(raw);  // 再回读，打印确认
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
