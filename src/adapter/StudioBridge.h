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
// 自动接管（2026-09-20 增）：上位机连上后不等人工输 `studio` —— SerialShell 识别到
// 注册 ID 开头的协议命令即置会话态并把整行转交 handleLine()，shell 立刻让位。
// 目的：上位机在 shell 态持续发 MG0~MG6/pull config 时，不再换来每条 5 行的"未知命令"回执。
//
// 宽容解析（2026-09-20 增，handleLine）：Studio 端"命令ID"填错是高频事故 —— 上位机把
// ID 与命令体直接拼成一行（simpleFOCConnector.py 的 sendCommand(devCommandID + command)），
// 于是线缆上只有两种可能：`M<命令体>`（ID 已配 'M'）或 `<命令体>`（ID 为空串）。
// 麻烦在于命令体自身也可能以 'M' 开头（CMD_MONITOR：MG0/MS…/MC/MD0），所以
// "剥首位"在 ID 为空时会把 `MG0` 剥成 `G0` → Commander 一路 unknown cmd err。
//
// 解法：不做字符猜测（会误伤 MDP…/MC2/MSM… 这些 ID 已配的行），改用**两条铁证**
// 判定本会话的约定，且随证据自我修正：
//   ① 见到 `MM…`  ⇒ 只可能是 ID('M') + 命令体('M…') ⇒ 约定 = 已配 ID
//      （反证：命令体没有 "MM" 形式 —— CMD_MONITOR 的子命令只有 G/D/C/S）
//   ② 见到「非 M 打头的命令字母」行 ⇒ 只可能是裸命令体 ⇒ 约定 = 无 ID
//      （反证：配了 ID 的行必然以 'M' 开头）
// 默认取"已配 ID"（官方约定，也是文档要求填的值）。上位机连上先跑 pullConfiguration(),
// 其 QR/LV/LU/LC/SE/SM/CD/C2/WC/R/E 全是非 'M' 打头 ⇒ ID 为空时几乎立刻命中 ②。
// 代价：掩盖上位机配置错误（故 shell/文档仍要求填 M，并在 dbg 下打印判定结果）；
// 收益：两种约定都能工作，不再因一个界面字段填错就整条链路瘫痪。
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

  /// Commander 注册 ID（上位机"命令ID"须与之对应；shell 自动接管按此字符甄别）
  static constexpr char kCmdId = 'M';

  /// 绑定电机并注册 Commander 通道
  void begin(SimpleFocMotor* m) {
    motor_ = m;
    self_ = this;
    if (dbgPort_) dbgPort_->println(F("[FW STUDIO] begin: registering Commander ID=M"));
    BLDCMotor& bm = m->rawMotor();
    bm.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
    bm.monitor_downsample = 0;  // 初始静默，由 Studio 端开启数据流
    cmd_.add(kCmdId, StudioBridge::onMotorCmd_, "motor");
    if (dbgPort_) dbgPort_->println(F("[FW STUDIO] begin complete"));
  }

  // ISerialSession：会话期独占串口收发
  void update() override;
  /// true：逐条命令回显 + 1s 状态快照；false（默认）：仅设置类按钮中文确认
  void setTrace(bool on) override;
  /// 整行协议命令转交（含注册 ID 字符与行尾 eol）：shell 自动接管入口
  /// 宽容解析：ID 已配 'M'（`MMG0`）与 ID 为空（`MG0`）两种上位机约定都接受
  void handleLine(char* line) override;

private:
  // Commander 回调是裸函数指针，用单实例静态指针转发
  static void onMotorCmd_(char* cmd);

  // 命令执行主体：两条入口（Commander 回调 / shell 转交）共用；
  // 入参为去掉注册 ID 之后的命令体，且需保留行尾 eol 字符（Commander 以此判 GET/SET）
  void handleCmd_(char* body);

  // 探针实现
  void describeSet_(const char* raw);   // 设置类命令生效后回读确认
  void printSnapshot_();                // 1s 周期状态快照
  static bool isGetCmd_(const char* r); // 判定查询/轮询命令（不打印确认）
  static bool isCmdLetter_(char c);     // 该字符是否可能是命令体首字母（commands.h 的 CMD_*，不含 '?' / '@' / '#'）
  static const char* modeName_(MotionControlType c);
  static const char* torqueName_(TorqueControlType t);
  static const char* pidParamName_(char sub);

  static StudioBridge* self_;

  Commander cmd_;
  SimpleFocMotor* motor_ = nullptr;
  Stream* dbgPort_ = nullptr;
  bool trace_ = false;
  /// 宽容解析：本会话"上位机未配命令ID"是否已被铁证确认（默认 false = 按已配 ID 处理）
  bool idAbsent_ = false;
  uint32_t lastSnapMs_ = 0;
  uint32_t lastUpdateHeartbeatMs_ = 0;
  uint32_t updateCount_ = 0;
};

} // namespace fockit
