#pragma once
// ============================================================
// tsl2.h —— T-SL② 一键对照实验状态机（独立头文件）
//
// enc 黄金段与 obs 无感段各跑一遍同激励速度方波（30↔lo），离线对比 σ/恢复时间判"退化≤2×"。
// 纪律：不新增控制行为——每一步经 slCommands() 走与手敲命令完全相同的代码路径（处理器
// 自带的 [SL CFG]/[SL ID] 回显即 CSV 分段标记）；本状态机只负责等待、判据与安全退避。
// 口径偏离留痕：TST-02 原版 ±方波穿越零速，无感不可穿盲区——改 hi-lo 方波（两段激励
// 相同，对照公平性不变）。
//
// 为什么是独立头文件：同代码 xtensa g++ 直接编译通过，但 Arduino .ino 预处理器
// （旧 ctags 原型生成器）被 struct 内嵌套的 enum class 绊住，把成员函数当文件级
// 函数生成原型 → "non-member function cannot have cv-qualifier"（2026-09-29
// 首烧实测）。.ino 才会被预处理，头文件原样交给 g++——故移出。
//
// 包含位置契约：仅在 FocKit_sensorless.ino 中包含，且须位于 FocKit.h 引入
// （.ino 第 67 行）与 VfSensor/ObsSensor/SrcType/probe 全局定义之后——本文件
// 不自带 include，类型与 Arduino 环境（Serial/millis/constrain）均来自包含点
// 之前（Arduino 构建对 .ino 自动前置 Arduino.h）。
// ============================================================

bool slCommands(int argc, char* argv[]);   // 定义于 .ino：状态机复用手敲命令路径

extern SimpleFocMotor motor;
extern SrcType srcType;
extern VfSensor vfSensor;
extern uint32_t probePeriodMs;
extern bool probeOn;
extern bool stepOn;

static bool tsl2Cmd(const char* a, const char* b = nullptr) {
  char b0[12], b1[12];
  strncpy(b0, a, sizeof(b0) - 1); b0[sizeof(b0) - 1] = 0;
  char* argv[2] = {b0, nullptr};
  int argc = 1;
  if (b != nullptr) {
    strncpy(b1, b, sizeof(b1) - 1); b1[sizeof(b1) - 1] = 0;
    argv[1] = b1; argc = 2;
  }
  return slCommands(argc, argv);
}

struct Tsl2Runner {
  enum class Ph : uint8_t { Off,
    EncRamp, EncSquare, EncStop,             // 阶段A：有感黄金对照
    VfLock, ObsSettle, ObsSquare, ObsBrake,  // 阶段B：无感对照
    EncFinal                                 // 编码器收尾刹停
  };
  Ph ph = Ph::Off;
  uint32_t tPhaseMs = 0;      // 当前阶段起点
  uint32_t tBeatMs = 0;       // 最近判据拍 / 连续判据窗口起点
  uint16_t beats = 0;         // 锁定判据连续拍（20ms 一拍 ×20）
  uint16_t halfIdx = 0;       // 方波半周期计数
  float amp = 10.0f;          // hi-lo 幅差（钳 5~15：下限 ≥15 rad/s 避盲区）
  uint32_t halfMs = 2000;
  uint16_t cycles = 5;
  uint16_t savedProbeMs = 50;
  bool savedProbeOn = true;
  float lastOdomA = 0;       // 里程表斜率采样（判据速度源）
  uint32_t lastOdomMs = 0;
  float slopeV = 0;          // 真速度 = Δθ里程/Δt（20ms 窗）

  float hiV() const { return 30.0f; }
  float loV() const { return hiV() - amp; }
  bool active() const { return ph != Ph::Off; }
  void emit(const char* s) { Serial.printf("[TSL2] %s\n", s); }
  void setPh(Ph p, uint32_t now) { ph = p; tPhaseMs = now; tBeatMs = now; beats = 0; halfIdx = 0; }

  // 判据速度源：里程表斜率。readEncoderVelocity()（SimpleFOC getVelocity）在 ~0.4ms
  // 微分窗下量化 ~3.69 rad/s 档位——真速度 30 时读数只跳 25.8/29.5/33.2/36.9，到速
  // 判据 |we−30|≤1 永假（20260929_212114 首跑 ABORT"A 段到速超时"根因）；里程表
  // 20ms 窗差分的量化仅 1 count/20ms≈0.077 rad/s，细 48 倍
  void sampleSlope(uint32_t now) {
    if (now - lastOdomMs < 20) return;
    float a = motor.readEncoderAngle();
    float dt = (now - lastOdomMs) * 0.001f;
    if (dt > 0.0f) slopeV = (a - lastOdomA) / dt;
    lastOdomA = a;
    lastOdomMs = now;
  }

  void begin(float a, uint32_t hm) {
    if (active()) { emit("已在运行（tsl2 off 取消）"); return; }
    if (stepOn) { emit("拒绝：step 在运行，先 step off"); return; }
    amp = constrain(a, 5.0f, 15.0f);
    if (hm < 500UL) hm = 500UL;
    if (hm > 10000UL) hm = 10000UL;
    halfMs = hm;
    savedProbeMs = (uint16_t)probePeriodMs; savedProbeOn = probeOn;
    probeOn = true; probePeriodMs = 20;   // 判据实验采样防呆（chirp 先例），结束恢复
    Serial.printf("[SL CFG] probe=%lums\n", (unsigned long)probePeriodMs);
    Serial.printf("[TSL2] 开始：hi=%.0f lo=%.0f 半周期=%lums ×%u 圈\n",
                  (double)hiV(), (double)loV(), (unsigned long)halfMs, (unsigned)cycles);
    emit("阶段A·有感黄金对照：src enc + loop v + 30");
    tsl2Cmd("src", "enc");
    tsl2Cmd("loop", "v");
    tsl2Cmd("30");
    lastOdomA = motor.readEncoderAngle();   // 斜率判据预置（首拍有分母）
    lastOdomMs = millis(); slopeV = 0;
    ph = Ph::EncRamp; tPhaseMs = tBeatMs = millis();
  }

  void finish(const char* why) {
    probePeriodMs = savedProbeMs; probeOn = savedProbeOn;
    Serial.printf("[SL CFG] probe=%lu%s\n", (unsigned long)probePeriodMs, probeOn ? "ms" : "off");
    Serial.printf("[TSL2] 结束：%s\n", why);
    ph = Ph::Off;
  }

  void bail(const char* why) {   // 安全退避：编码器速度环刹车到 0，vf ghost 同步撤
    Serial.printf("[TSL2] ABORT：%s（编码器刹车中）\n", why);
    tsl2Cmd("vf", "off");
    tsl2Cmd("src", "enc");
    tsl2Cmd("loop", "v");
    tsl2Cmd("0");
    finish(why);
  }

  void update(uint32_t now, bool powerOk) {
    if (ph == Ph::Off) return;
    if (!powerOk) { bail("欠压"); return; }
    if (stepOn) { bail("手动接管(step)"); return; }
    // 阶段前提检查：手动发命令视为接管取消（不与自动序列抢方向盘）
    switch (ph) {
      case Ph::EncRamp: case Ph::EncSquare: case Ph::EncStop: case Ph::EncFinal:
        if (srcType != SrcType::Enc) { bail("手动接管(src)"); return; }
        break;
      case Ph::VfLock:
        if (srcType != SrcType::Vf || !vfSensor.running) { bail("手动接管(vf)"); return; }
        break;
      case Ph::ObsSettle: case Ph::ObsSquare: case Ph::ObsBrake:
        if (srcType != SrcType::Obs) { bail("手动接管(src)"); return; }
        break;
      default: break;
    }
    sampleSlope(now);   // 判据速度源（见 sampleSlope 注释）
    switch (ph) {
      case Ph::EncRamp:
        if (fabsf(slopeV - hiV()) <= 1.5f) {
          if (now - tBeatMs >= 1000) { emit("阶段A·方波开始"); setPh(Ph::EncSquare, now); tsl2Cmd("30"); }
        } else tBeatMs = now;
        if (now - tPhaseMs > 20000) { bail("A 段到速超时"); return; }
        break;

      case Ph::EncSquare: case Ph::ObsSquare: {
        if (now - tPhaseMs >= halfMs) {
          tPhaseMs = now;
          char buf[8];
          snprintf(buf, sizeof(buf), "%.0f", (double)((++halfIdx & 1) ? loV() : hiV()));
          tsl2Cmd(buf);   // 裸数字路径：setTarget + 回显（兼作 CSV 切片标记）
          if (halfIdx >= 2 * cycles) {
            if (ph == Ph::EncSquare) { emit("阶段A·减速停"); tsl2Cmd("0"); setPh(Ph::EncStop, now); }
            else { emit("阶段B·减速（8 以下切编码器收尾）"); tsl2Cmd("0"); setPh(Ph::ObsBrake, now); }
          }
        }
        break; }

      case Ph::EncStop:
        if (fabsf(slopeV) <= 0.5f) {
          if (now - tBeatMs >= 500) {
            emit("阶段B·无感对照：obs smo + vf 30 + 等锁");
            tsl2Cmd("obs", "smo");
            tsl2Cmd("vf", "30");
            setPh(Ph::VfLock, now);
          }
        } else tBeatMs = now;
        if (now - tPhaseMs > 15000) { bail("A 段停机超时"); return; }
        break;

      case Ph::VfLock: {
        if (now - tPhaseMs > 60000) { bail("B 段等锁超时"); return; }
        if (now - tBeatMs < 20) break;   // 判据 20ms 一拍（同无感台 UI 三条件口径）
        tBeatMs = now;
        float vw = vfSensor.wMech;
        bool ok = vw >= fmaxf(10.0f, 0.9f * vfSensor.wTarget)
               && obsLocked()
               && obsWe() / PP >= 0.5f * vw;
        beats = ok ? (uint16_t)(beats + 1) : 0;
        if (beats >= 20) {
          tsl2Cmd("src", "obs");   // 无扰交接（打印 offEst 吸收量）
          tsl2Cmd("loop", "v");    // 无扰入环（目标=当前速度 + 积分预置）
          tsl2Cmd("30");
          emit("阶段B·稳定至 30");
          setPh(Ph::ObsSettle, now);
        }
        break; }

      case Ph::ObsSettle:
        if (fabsf(slopeV - hiV()) <= 1.5f) {
          if (now - tBeatMs >= 1000) { emit("阶段B·方波开始"); setPh(Ph::ObsSquare, now); tsl2Cmd("30"); }
        } else tBeatMs = now;
        if (now - tPhaseMs > 15000) { bail("B 段到速超时"); return; }
        break;

      case Ph::ObsBrake:
        if (slopeV <= 8.0f) {   // 8 rad/s 时 BEMF≈0.26V≫0.08V 锁阈，全程有感知区切编码器
          tsl2Cmd("vf", "off");   // 撤 vf ghost
          tsl2Cmd("src", "enc");  // 速度环仍在、目标 0 → 编码器刹停
          emit("编码器收尾刹停中");
          setPh(Ph::EncFinal, now);
        }
        if (now - tPhaseMs > 15000) { bail("B 段减速超时"); return; }
        break;

      case Ph::EncFinal:
        if (fabsf(slopeV) <= 0.5f) {
          if (now - tBeatMs >= 500) { finish("两段采集完毕，可导 CSV"); return; }
        } else tBeatMs = now;
        if (now - tPhaseMs > 15000) { bail("收尾停机超时"); return; }
        break;

      default: break;
    }
  }
};
Tsl2Runner tsl2;
