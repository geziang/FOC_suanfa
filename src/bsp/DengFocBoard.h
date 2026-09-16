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

// ---- 2208 云台电机档案（规格书）----
struct MotorProfile {
  int polePairs;         // 极对数（官方例程 BLDCMotor(7) 一致）
  float lineResistance;  // [Ω] 线间（规格书“绕线电阻”）
  float phaseResistance; // [Ω] 相电阻（星形：线间/2；歧义登记见 docs/00 §6.3）
  float lineInductance;  // [H]  线间
  float phaseInductance; // [H]  相电感（星形：线间/2）
  float kv;              // [rpm/V]
  float kt;              // [N·m/A]（规格书 0.03N·m 解读；KV 换算约 0.012~0.02，P1 辨识收口）
  float ratedCurrent;    // [A]
  float maxCurrent;      // [A]
  float ratedVoltage;    // [V]
};

constexpr MotorProfile MOTOR_2208 = {
    7,
    21.2f, 10.6f,
    0.0053f, 0.00265f,
    110.0f,
    0.03f,
    0.8f, 4.5f,
    12.0f};

// ---- 安全默认限幅 ----
// 电流上限取额定 0.8A（保护优先，需动态余量时由上层放宽，勿超 4.5A 峰值）
// 电压上限 = 额定电流 × 相电阻 ≈ 8.5V，同时受实测母线电压钳制
constexpr float DEFAULT_CURRENT_LIMIT = MOTOR_2208.ratedCurrent;
constexpr float DEFAULT_TORQUE_LIMIT  = 0.03f;  // [N·m] ≈ 额定电流×KT

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
