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
  bool useCurrentSense = true;        // 板载 inline 电流采样（链接后 MT1/MT2 可用，id/iq 实测）
  int csPinA = -1;                    // 电流采样 A 相引脚
  int csPinB = -1;                    // 电流采样 B 相引脚
};

class SimpleFocMotor : public IMotor {
public:
  explicit SimpleFocMotor(const SimpleFocMotorConfig& cfg);

  /// 在 Arduino setup() 中、Serial.begin() 之后初始化 NVS 标定存储。
  bool beginStorage();

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

  void setLoopGains(LoopType loop, const LoopGains& g) override;

  /// Studio MC 命令的 mode_ 回写（仅同步 FocKit 内部枚举，不触碰 SimpleFOC
  /// controller/torque_controller——那些由 MC/MT 命令本身生效）。
  /// 消除"直通不同步"缺陷：否则 Idle 覆写分支在速度/位置模式下仍每拍改写
  /// shaft_velocity，与 move() 双路径同写（速度环失控排查中发现的确定缺陷）。
  void syncStudioControlMode(int simplefocControl);

  // ---- 无感战役扩展（2026-09-29，P4 前家庭平台无感观测器矩阵）----
  // 角度源挂接：ext 语义与内部编码器一致（机械轴角 rad / rad·s⁻¹，方向由
  /// SimpleFOC sensor_direction 统一处理）。attach 后三环反馈全吃 ext，
  /// detach 切回内部 AS5600；标定零位（zero_electric_angle/NVS）不受影响。
  /// 纯新增路径：extSensor_ 为空时 update() 行为与历史逐行等价（01 号冻结纪律）。
  void attachExternalSensor(Sensor* ext);
  void detachExternalSensor();
  bool externalSensorAttached() const { return extSensor_ != nullptr; }

  /// 观测器输入只读访问器（无感观测器吃相电流/指令电压，均为纯读不触碰控制）：
  PhaseCurrent_s readPhaseCurrents();              ///< 相电流（InlineCurrentSense 实测）
  float readUq() const { return motor_.voltage.q; }  ///< 指令电压 Uq [V]
  float readUd() const { return motor_.voltage.d; }  ///< 指令电压 Ud [V]
  float readZeroElectricAngle() const { return motor_.zero_electric_angle; }

  /// 速度环 PID 积分预置（无扰入环的另一半：目标无扰＋PID 状态无扰）——电压力矩
  /// 模式下 PID 输出即电压，预置当前输出电压使入环第一拍力矩连续不断档（185818 案：
  /// 入环清力矩→摩擦滑停→BEMF 消失→观测器幻觉→环被幻觉喂瞎）。纯新增路径，01 号
  /// 冻结纪律合规。
  /// 访问途径：SimpleFOC PIDController::integral_prev（Tustin 持久积分）为 protected
  /// ——不改上游库，经派生类取成员指针（protected 规则允许"通过派生类"访问，所得
  /// float PIDController::* 可合法作用于任何真实 PIDController 对象）。已用本机
  /// xtensa-esp32-elf-g++ 8.4（与 Arduino 编译同款）验证编译通过。
  struct PidIntegralKey : public PIDController {
    static float PIDController::*member() { return &PidIntegralKey::integral_prev; }
  };
  void preloadVelocityIntegral(float volts) {
    motor_.PID_velocity.*PidIntegralKey::member() = volts;
  }
  /// 对账真值（无感挂接期间 state.angle 已是估计值，编码器真值走此独立出口，纯读）：
  /// 须先自刷缓存——挂接外部传感器后 loopFOC 只 update 挂接者，AS5600 的
  /// Sensor 基类缓存会冻结（getAngle/getVelocity 只回放缓存值），真值裁判失明。
  /// update() 为一次 I2C 读，探针周期级调用开销可忽略；两访问器各自刷新，
  /// getVelocity 的差分窗口=两次调用间隔（探针 50ms），数学上不受中间 update 次数影响。
  float readEncoderAngle() {
    sensor_.update();
    return sensor_.getAngle() * static_cast<float>(motor_.sensor_direction);
  }
  float readEncoderVelocity() {
    sensor_.update();
    return sensor_.getVelocity() * static_cast<float>(motor_.sensor_direction);
  }

  void update() override;

  bool saveCalibration() override;
  bool loadCalibration() override;
  bool hasSavedCalibration() override;

  /// 仅供适配层内部使用（StudioBridge 等）。
  /// 上层（control/app）触碰即违反分层契约（ARC-01 §2），不要调用。
  BLDCMotor& rawMotor() { return motor_; }

private:
  float torqueToVolts_(float torqueNm) const;

  // 声明顺序即构造顺序：cfg_ 必须先于使用它的成员
  SimpleFocMotorConfig cfg_;
  MagneticSensorI2C sensor_;
  BLDCDriver3PWM driver_;
  InlineCurrentSense currentSense_;
  BLDCMotor motor_;
  CalibrationStore store_;

  MotorLimits limits_;
  ControlMode mode_ = ControlMode::Idle;
  float target_ = 0.0f;       // SI 单位，按 mode_ 解释
  float voltageLimit_ = 0.0f;
  bool inited_ = false;
  Sensor* extSensor_ = nullptr;  // 外部角度源（无感观测器/VF 适配器）；空=内部 AS5600
  MotorState state_;
};

/// DengFOC V4 M0 通道默认配置（引脚真值见 bsp/DengFocBoard.h）
SimpleFocMotorConfig defaultM0Config();
/// DengFOC V4 M1 通道默认配置
SimpleFocMotorConfig defaultM1Config();

} // namespace fockit
