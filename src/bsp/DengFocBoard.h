#pragma once
// FocKit 板级层 —— DengFOC V4 板级档案（单一真值来源）
//
// 数值来源（不得凭记忆改动）：
//   v4/2、ESP有感FOC历程资料/Simplefoc库例程/ 第 1/3/8 号例程（SimpleFOC 2.2.1 实测）
//   2208 电机规格书（docs/00 §6.2）
// 换板只改本文件。

#include <Arduino.h>
#include <Wire.h>

namespace fockit {
namespace dengfoc_v4 {

// ---- 电机相线 BLDCDriver3PWM(A, B, C, EN) ----
constexpr int M0_PIN_A = 32;
constexpr int M0_PIN_B = 33;
constexpr int M0_PIN_C = 25;
constexpr int M1_PIN_A = 26;
constexpr int M1_PIN_B = 27;
constexpr int M1_PIN_C = 14;
constexpr int PIN_DRIVER_ENABLE = 12;  // 双路共用使能

// ---- AS5600 编码器（I2C 地址固定 0x36，双编码器必须走两路独立总线）----
constexpr int M0_SDA = 19;
constexpr int M0_SCL = 18;             // TwoWire(0)
constexpr int M1_SDA = 23;
constexpr int M1_SCL = 5;              // TwoWire(1)
constexpr uint32_t I2C_HZ = 400000UL;

// ---- 电源检测（官方例程同款分压）----
constexpr int PIN_VIN_SENSE = 13;
constexpr float VIN_SCALE = 8.5f / 1000.0f;  // mV → V
constexpr float VIN_UNDERVOLT = 11.1f;       // 欠压阈值 [V]

// ---- 2208-80T 云台电机档案（官方规格 2026-09-17 版，替代旧推算值）----
struct MotorProfile {
  int polePairs;         // 极对数
  float lineResistance;  // [Ω] 线间（=2×相电阻，万用表验证用）
  float phaseResistance; // [Ω] 相电阻（官方确认：单相绕组电阻）
  float lineInductance;  // [H]  线间（星形串联推算 2×相电感，验证用）
  float phaseInductance; // [H]  相电感（官方明确）
  float kv;              // [rpm/V]
  float kt;              // [N·m/A] 公式初值 = 8.27/KV（SI 下 Ke=KT），T-P1-4 实测收口
  float ratedCurrent;    // [A] 持续电流上限（官方 200~500mA 取上限）
  float maxCurrent;      // [A] 峰值（官方未给，按持续上限执行）
  float ratedVoltage;    // [V]
};

constexpr MotorProfile MOTOR_2208 = {
    7,
    16.5f, 8.25f,
    0.0085f, 0.00425f,
    100.0f,
    0.0827f,
    0.5f, 0.5f,
    12.0f};

// ---- 安全默认限幅 ----
// 电流上限取官方持续范围上限 0.5A（峰值官方未给，按持续执行；瞬时过载须另立实验依据）
// 电压上限 = 0.5A × 8.25Ω ≈ 4.1V，同时受实测母线电压钳制
constexpr float DEFAULT_CURRENT_LIMIT = MOTOR_2208.ratedCurrent;
constexpr float DEFAULT_TORQUE_LIMIT  = 0.03f;  // [N·m] 官方标称扭力（≈0.36A，落在持续带内）

/// 上电早期初始化：复刻官方例程的启动仪式（相线输入上拉 + 12bit ADC），
/// 必须在任何 driver.init() 之前调用一次。
inline void earlyInit() {
  pinMode(M0_PIN_A, INPUT_PULLUP);
  pinMode(M0_PIN_B, INPUT_PULLUP);
  pinMode(M0_PIN_C, INPUT_PULLUP);
  pinMode(M1_PIN_A, INPUT_PULLUP);
  pinMode(M1_PIN_B, INPUT_PULLUP);
  pinMode(M1_PIN_C, INPUT_PULLUP);
  analogReadResolution(12);
}

/// 母线电压实测 [V]（引脚13 分压，官方例程同款）
inline float readVin() {
  return analogReadMilliVolts(PIN_VIN_SENSE) * VIN_SCALE;
}

/// M0 编码器总线（两路编码器地址相同，必须独立总线）
inline TwoWire& m0EncoderBus() { static TwoWire w(0); return w; }
/// M1 编码器总线
inline TwoWire& m1EncoderBus() { static TwoWire w(1); return w; }

} // namespace dengfoc_v4
} // namespace fockit
