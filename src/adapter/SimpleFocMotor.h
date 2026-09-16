#pragma once
// FocKit 适配层 —— SimpleFocMotor：用官方 SimpleFOC 库实现 IMotor 契约
//
// 本文件是 core/、control/ 层唯一允许“看见”SimpleFOC 的地方之下界：
// SimpleFOC 的类型不越过本头文件向外泄漏（P4 换 LabCanMotor 时上层无感）。
//
// 内环说明（docs/00 §2.2）：P0 力矩通道用电压力矩近似
//   T → iq = T/KT → U = iq·R（稳态），SimpleFOC 侧再按 voltage_limit 钳位。

#include <SimpleFOC.h>
#include <Wire.h>

#include "core/IMotor.h"
#include "persist/CalibrationStore.h"
#include "bsp/DengFocBoard.h"

namespace fockit {

struct SimpleFocMotorConfig {
  const char* id = "m0";              // 标定存储键（≤15字符）与调试名
  int pinA = -1;
  int pinB = -1;
  int pinC = -1;
  int pinEnable = dengfoc_v4::PIN_DRIVER_ENABLE;
  TwoWire* wire = nullptr;            // 编码器总线（契约模式均需反馈）
  int sda = -1;
  int scl = -1;                       // 总线引脚，init() 内执行 wire->begin
  dengfoc_v4::MotorProfile profile = dengfoc_v4::MOTOR_2208;
  float currentLimit = dengfoc_v4::DEFAULT_CURRENT_LIMIT;  // [A]
  MotorLimits limits;                 // 目标限幅（默认见 core/MotorState.h）
  bool monitor = true;                // SimpleFOC 初始化/标定日志
};

class SimpleFocMotor : public IMotor {
public:
  explicit SimpleFocMotor(const SimpleFocMotorConfig& cfg);

  // ---- IMotor ----
  bool init() override;
  void enable() override;
  void disable() override;
  bool isEnabled() override;

  void setMode(ControlMode mode) override;
  ControlMode getMode() override;
  void setTarget(float target) override;
  void setTorqueTarget(float torqueNm) override;
  void setVelocityTarget(float radPerSec) override;
  void setPositionTarget(float rad) override;
  float getTarget() override;

  float getAngle() override;
  float getVelocity() override;
  float getIq() override;
  float getTorque() override;
  MotorState getState() override;

  void setLimits(const MotorLimits& lim) override;
  MotorLimits getLimits() override;

  void update() override;

  bool saveCalibration() override;
  bool loadCalibration() override;
  bool hasSavedCalibration() override;

private:
  float torqueToVolts_(float torqueNm) const;

  // 声明顺序即构造顺序：cfg_ 必须先于使用它的成员
  SimpleFocMotorConfig cfg_;
  MagneticSensorI2C sensor_;
  BLDCDriver3PWM driver_;
  BLDCMotor motor_;
  CalibrationStore store_;

  MotorLimits limits_;
  ControlMode mode_ = ControlMode::Idle;
  float target_ = 0.0f;       // SI 单位，按 mode_ 解释
  float voltageLimit_ = 0.0f;
  bool inited_ = false;
  MotorState state_;
};

/// DengFOC V4 M0 通道默认配置（引脚真值见 bsp/DengFocBoard.h）
SimpleFocMotorConfig defaultM0Config();
/// DengFOC V4 M1 通道默认配置
SimpleFocMotorConfig defaultM1Config();

} // namespace fockit
