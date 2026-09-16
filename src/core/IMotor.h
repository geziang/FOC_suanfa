#pragma once
// FocKit 契约层 —— IMotor 运动接口（★移植边界）
//
// 分层契约（docs/00 §3.1）：
//   允许：目标指令（力矩/速度/位置）、状态读取、模式切换、限幅、标定存取、周期 update()
//   禁止：直接操作相电压、PWM、d/q 轴电流、绕组时序、寄存器
//
// 本文件不得依赖任何具体硬件库（SimpleFOC/Arduino 皆不依赖）。
// 未来实验室硬件到位后，新增一个 IMotor 实现（如 LabCanMotor），
// 全部上层算法零改动平移。

#include "core/ControlMode.h"
#include "core/MotorState.h"

namespace fockit {

class IMotor {
public:
  virtual ~IMotor() {}

  // ---------- 生命周期 ----------
  /// 硬件初始化 + 标定加载（NVS 已有则注入免标定，无则现场标定并固化）
  virtual bool init() = 0;
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual bool isEnabled() = 0;

  // ---------- 目标指令 ----------
  virtual void setMode(ControlMode mode) = 0;
  virtual ControlMode getMode() = 0;
  /// 按当前模式解释单位：Torque→N·m，Velocity→rad/s，Position→rad
  virtual void setTarget(float target) = 0;
  virtual void setTorqueTarget(float torqueNm) = 0;
  virtual void setVelocityTarget(float radPerSec) = 0;
  virtual void setPositionTarget(float rad) = 0;
  virtual float getTarget() = 0;

  // ---------- 状态读取 ----------
  virtual float getAngle() = 0;     // [rad] 多圈机械角
  virtual float getVelocity() = 0;  // [rad/s]
  virtual float getIq() = 0;        // [A]（P0 为指令/估算值）
  virtual float getTorque() = 0;    // [N·m] 估计
  /// 一次性快照，避免多次读取间状态不一致
  virtual MotorState getState() = 0;

  // ---------- 限幅 ----------
  virtual void setLimits(const MotorLimits& lim) = 0;
  virtual MotorLimits getLimits() = 0;

  // ---------- 周期任务 ----------
  /// 每个控制拍调用一次：执行底层闭环并刷新状态缓存
  virtual void update() = 0;

  // ---------- 标定持久化 ----------
  virtual bool saveCalibration() = 0;
  virtual bool loadCalibration() = 0;
  virtual bool hasSavedCalibration() = 0;
};

} // namespace fockit
