#include "adapter/SimpleFocMotor.h"

namespace fockit {

SimpleFocMotorConfig defaultM0Config() {
  SimpleFocMotorConfig c;
  c.id = "m0";
  c.pinA = dengfoc_v4::M0_PIN_A;
  c.pinB = dengfoc_v4::M0_PIN_B;
  c.pinC = dengfoc_v4::M0_PIN_C;
  c.pinEnable = dengfoc_v4::PIN_DRIVER_ENABLE;
  c.wire = &dengfoc_v4::m0EncoderBus();
  c.sda = dengfoc_v4::M0_SDA;
  c.scl = dengfoc_v4::M0_SCL;
  c.profile = dengfoc_v4::MOTOR_2208;
  c.currentLimit = dengfoc_v4::DEFAULT_CURRENT_LIMIT;
  c.useCurrentSense = true;
  c.csPinA = dengfoc_v4::M0_CS_PIN_A;
  c.csPinB = dengfoc_v4::M0_CS_PIN_B;
  return c;
}

SimpleFocMotorConfig defaultM1Config() {
  SimpleFocMotorConfig c;
  c.id = "m1";
  c.pinA = dengfoc_v4::M1_PIN_A;
  c.pinB = dengfoc_v4::M1_PIN_B;
  c.pinC = dengfoc_v4::M1_PIN_C;
  c.pinEnable = dengfoc_v4::PIN_DRIVER_ENABLE;
  c.wire = &dengfoc_v4::m1EncoderBus();
  c.sda = dengfoc_v4::M1_SDA;
  c.scl = dengfoc_v4::M1_SCL;
  c.profile = dengfoc_v4::MOTOR_2208;
  c.currentLimit = dengfoc_v4::DEFAULT_CURRENT_LIMIT;
  c.useCurrentSense = true;
  c.csPinA = dengfoc_v4::M1_CS_PIN_A;
  c.csPinB = dengfoc_v4::M1_CS_PIN_B;
  return c;
}

SimpleFocMotor::SimpleFocMotor(const SimpleFocMotorConfig& cfg)
    : cfg_(cfg),
      sensor_(AS5600_I2C),
      driver_(cfg.pinA, cfg.pinB, cfg.pinC, cfg.pinEnable),
      currentSense_(dengfoc_v4::CURRENT_SENSE_SHUNT, dengfoc_v4::CURRENT_SENSE_GAIN,
                    cfg.csPinA, cfg.csPinB),
      motor_(cfg.profile.polePairs),
      limits_(cfg.limits) {}

bool SimpleFocMotor::init() {
  if (inited_) return true;

  if (!cfg_.wire) {
    Serial.println("[FocKit] 缺少编码器总线，契约模式均需反馈，拒绝初始化");
    return false;
  }
  if (cfg_.sda >= 0 && cfg_.scl >= 0) {
    cfg_.wire->begin(cfg_.sda, cfg_.scl, dengfoc_v4::I2C_HZ);
  }
  sensor_.init(cfg_.wire);
  motor_.linkSensor(&sensor_);

  float vin = dengfoc_v4::readVin();
  driver_.voltage_power_supply = vin;
  if (!driver_.init()) return false;
  motor_.linkDriver(&driver_);

  // 板载电流采样（15/16 课接法）：链接后 MT1/MT2 会话可用，id/iq 实测
  if (cfg_.useCurrentSense && cfg_.csPinA >= 0 && cfg_.csPinB >= 0) {
    currentSense_.init();
    motor_.linkCurrentSense(&currentSense_);
  }

  motor_.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // 电压上限 = 电流上限 × 相电阻，且不超过实测母线的 95%
  voltageLimit_ = cfg_.currentLimit * cfg_.profile.phaseResistance;
  if (voltageLimit_ > vin * 0.95f) voltageLimit_ = vin * 0.95f;
  motor_.voltage_limit = voltageLimit_;
  motor_.velocity_limit = limits_.maxVelocity;

  // 官方 2208 速度环基线（电压域），内置对照组环的起点（docs/00 §6.1）
  motor_.PID_velocity.P = 0.021f;
  motor_.PID_velocity.I = 0.12f;
  motor_.LPF_velocity.Tf = 0.01f;

  if (cfg_.monitor) motor_.useMonitoring(Serial);
  motor_.init();

  // NVS 已有标定则注入，initFOC() 检测到有效零位/方向后跳过标定
  MotorCalib cal;
  if (store_.load(cfg_.id, cal)) {
    motor_.zero_electric_angle = cal.zeroElectricAngle;
    motor_.sensor_direction = static_cast<Direction>(cal.sensorDirection);
    Serial.printf("[FocKit] %s 注入历史标定：零位=%.3frad 方向=%d\n",
                  cfg_.id, cal.zeroElectricAngle, cal.sensorDirection);
  }

  if (!motor_.initFOC()) {
    Serial.printf("[FocKit] %s initFOC 失败（检查编码器与相线）\n", cfg_.id);
    return false;
  }
  // 标定结果固化：断电重启免标定（对标 STM32 底层 Flash 双页注入）
  saveCalibration();

  motor_.disable();
  setMode(ControlMode::Idle);
  inited_ = true;
  update();
  Serial.printf("[FocKit] %s 就绪（限压 %.2fV ≈ %.2fA）\n",
                cfg_.id, voltageLimit_, cfg_.currentLimit);
  return true;
}

void SimpleFocMotor::enable()  { if (inited_) motor_.enable(); }
void SimpleFocMotor::disable() { if (inited_) motor_.disable(); }
bool SimpleFocMotor::isEnabled() { return inited_ && motor_.enabled; }

void SimpleFocMotor::setMode(ControlMode mode) {
  if (!inited_) return;
  mode_ = mode;
  switch (mode) {
    case ControlMode::Velocity:
      motor_.controller = MotionControlType::velocity;
      break;
    case ControlMode::Position:
      motor_.controller = MotionControlType::angle;
      break;
    case ControlMode::Torque:
    case ControlMode::Idle:
    default:
      motor_.torque_controller = TorqueControlType::voltage;
      motor_.controller = MotionControlType::torque;
      break;
  }
  target_ = 0.0f;  // 切模式即清目标，避免旧目标串档
  if (mode_ == ControlMode::Idle) motor_.target = 0.0f;
}

ControlMode SimpleFocMotor::getMode() { return mode_; }

void SimpleFocMotor::setTorqueTarget(float torqueNm) {
  if (mode_ != ControlMode::Torque) setMode(ControlMode::Torque);
  target_ = constrain(torqueNm, -limits_.maxTorque, limits_.maxTorque);
  motor_.target = torqueToVolts_(target_);
}

void SimpleFocMotor::setVelocityTarget(float radPerSec) {
  if (mode_ != ControlMode::Velocity) setMode(ControlMode::Velocity);
  target_ = constrain(radPerSec, -limits_.maxVelocity, limits_.maxVelocity);
  motor_.target = target_;
}

void SimpleFocMotor::setPositionTarget(float rad) {
  if (mode_ != ControlMode::Position) setMode(ControlMode::Position);
  target_ = constrain(rad, limits_.minPosition, limits_.maxPosition);
  motor_.target = target_;
}

void SimpleFocMotor::setTarget(float target) {
  switch (mode_) {
    case ControlMode::Torque:   setTorqueTarget(target); break;
    case ControlMode::Velocity: setVelocityTarget(target); break;
    case ControlMode::Position: setPositionTarget(target); break;
    case ControlMode::Idle:     break;
  }
}

float SimpleFocMotor::getTarget() { return target_; }

float SimpleFocMotor::torqueToVolts_(float torqueNm) const {
  float iq = torqueNm / cfg_.profile.kt;           // N·m → A
  float u = iq * cfg_.profile.phaseResistance;     // A → V（稳态近似）
  return constrain(u, -voltageLimit_, voltageLimit_);
}

float SimpleFocMotor::getAngle()    { return state_.angle; }
float SimpleFocMotor::getVelocity() { return state_.velocity; }
float SimpleFocMotor::getIq()       { return state_.iq; }
float SimpleFocMotor::getTorque()   { return state_.torque; }
MotorState SimpleFocMotor::getState() { return state_; }

void SimpleFocMotor::setLimits(const MotorLimits& lim) {
  limits_ = lim;
  motor_.velocity_limit = limits_.maxVelocity;
}

MotorLimits SimpleFocMotor::getLimits() { return limits_; }

void SimpleFocMotor::setLoopGains(LoopType loop, const LoopGains& g) {
  PIDController* pid = nullptr;
  LowPassFilter* lpf = nullptr;
  switch (loop) {
    case LoopType::CurrentQ: pid = &motor_.PID_current_q; lpf = &motor_.LPF_current_q; break;
    case LoopType::CurrentD: pid = &motor_.PID_current_d; lpf = &motor_.LPF_current_d; break;
    case LoopType::Velocity: pid = &motor_.PID_velocity;  lpf = &motor_.LPF_velocity;  break;
    case LoopType::Position: pid = &motor_.P_angle;       lpf = &motor_.LPF_angle;     break;
  }
  if (pid != nullptr) {
    if (g.kp >= 0.0f) pid->P = g.kp;   // -1 = 不变更
    if (g.ki >= 0.0f) pid->I = g.ki;
    if (g.kd >= 0.0f) pid->D = g.kd;   // Position 环为 P 控制，仅 kp 生效
  }
  if (lpf != nullptr && g.lpfTf >= 0.0f) lpf->Tf = g.lpfTf;
}

void SimpleFocMotor::update() {
  if (!inited_) return;

  motor_.loopFOC();
  motor_.move();

  // SimpleFOC 的 move() 仅在 velocity/angle 模式维护轴状态；
  // 力矩/空闲模式下中间件（速度PID等）仍需反馈，此处按其内部算法自行刷新。
  if (mode_ == ControlMode::Torque || mode_ == ControlMode::Idle) {
    motor_.shaft_velocity = motor_.LPF_velocity(sensor_.getVelocity()) *
                            static_cast<float>(motor_.sensor_direction);
    motor_.shaft_angle = sensor_.getAngle() *
                         static_cast<float>(motor_.sensor_direction);
  }

  state_.angle = motor_.shaft_angle;
  state_.velocity = motor_.shaft_velocity;
  // iq 估算：Uq/R 稳态近似（P0 无电流采样；接采样后此处换测量值，接口不变）
  state_.iq = motor_.voltage.q / cfg_.profile.phaseResistance;
  state_.torque = state_.iq * cfg_.profile.kt;
  state_.mode = mode_;
  state_.enabled = motor_.enabled;
  state_.stampUs = micros();
}

bool SimpleFocMotor::saveCalibration() {
  MotorCalib c;
  c.zeroElectricAngle = motor_.zero_electric_angle;
  c.sensorDirection = static_cast<int8_t>(motor_.sensor_direction);
  c.magic = MotorCalib::MAGIC;
  if (store_.save(cfg_.id, c)) {
    Serial.printf("[FocKit] %s 标定已固化（零位=%.3frad 方向=%d）\n",
                  cfg_.id, c.zeroElectricAngle, c.sensorDirection);
    return true;
  }
  return false;
}

bool SimpleFocMotor::loadCalibration() {
  // 注意：注入只对下一次 init()/initFOC() 生效
  MotorCalib c;
  if (!store_.load(cfg_.id, c)) return false;
  motor_.zero_electric_angle = c.zeroElectricAngle;
  motor_.sensor_direction = static_cast<Direction>(c.sensorDirection);
  return true;
}

bool SimpleFocMotor::hasSavedCalibration() {
  MotorCalib c;
  return store_.load(cfg_.id, c);
}

} // namespace fockit
