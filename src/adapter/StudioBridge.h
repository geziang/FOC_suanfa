#pragma once
// FocKit 适配层 —— StudioBridge：SimpleFOC Studio 上位机会话桥
//
// 协议（对齐官方 16 课例程）：Commander 'M' 全权委托电机命令集
// （目标/模式/PID/限幅，见 docs.simplefoc.com）+ motor.monitor 数据流
// （_MON_TARGET | _MON_VEL | _MON_ANGLE，downsample 初始静默由 Studio 端开启）。
//
// 可观测性（按钮级回执，定位"上位机点了没反应"）：
// - 每条"设置类"命令在 Commander 生效后，回读实际成员打印一行中文确认（[Studio] 前缀），
//   证明按钮点击已到达固件且确实改了参数；查询/轮询类（pull config、MG0~6）静默不刷屏。
// - setTrace(true)（shell 命令 dbg on）后追加：每条原始命令回显 + 1 秒一次状态快照。
// - 回执行一律 '[' 开头、纯中文标签：SimpleFOC Studio 解析器对非数字开头、且不含
//   PID/Motion/Torque/Status/Limits/Monitor 等英文标记的行会直接忽略，不污染参数与曲线。
//
// 约束与纪律：
// 1. Commander/SimpleFOC 类型只允许存在于本文件与 SimpleFocMotor（契约边界，ARC-01 §2）；
// 2. Studio 在线改的 PID/限幅是 RAM 值，重启即失——调好后必须抄回代码配置，
//    NVS 只固化编码器标定（这是定位：临时调参会话，不是参数存储通道）；
// 3. 单实例假设：一个会话一个桥。

#include <SimpleFOC.h>

#include "core/ISerialSession.h"
#include "adapter/SimpleFocMotor.h"

namespace fockit {

class StudioBridge : public ISerialSession {
public:
  explicit StudioBridge(Stream& port = Serial) : cmd_(port), dbgPort_(&port) {}

  /// 绑定电机并注册 Commander 通道
  void begin(SimpleFocMotor* m) {
    motor_ = m;
    self_ = this;
    BLDCMotor& bm = m->rawMotor();
    bm.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
    bm.monitor_downsample = 0;  // 初始静默，由 Studio 端开启数据流
    cmd_.add('M', StudioBridge::onMotorCmd_, "motor");
  }

  // ISerialSession：会话期独占串口收发
  void update() override;
  /// true：逐条命令回显 + 1s 状态快照；false（默认）：仅设置类按钮中文确认
  void setTrace(bool on) override;

private:
  // Commander 回调是裸函数指针，用单实例静态指针转发
  static void onMotorCmd_(char* cmd);

  // 探针实现
  void describeSet_(const char* raw);   // 设置类命令生效后回读确认
  void printSnapshot_();                // 1s 周期状态快照
  static bool isGetCmd_(const char* r); // 判定查询/轮询命令（不打印确认）
  static const char* modeName_(MotionControlType c);
  static const char* torqueName_(TorqueControlType t);
  static const char* pidParamName_(char sub);

  static StudioBridge* self_;

  Commander cmd_;
  SimpleFocMotor* motor_ = nullptr;
  Stream* dbgPort_ = nullptr;
  bool trace_ = false;
  uint32_t lastSnapMs_ = 0;
};

} // namespace fockit
