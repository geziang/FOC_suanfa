#pragma once
// FocKit 契约层 —— 串口会话接口
//
// 用途：上位机会话（如 SimpleFOC Studio 桥）需要独占串口收发时，
// 通过本接口接入 SerialShell 的"让位"机制：会话期间 shell 停止自身解析，
// 把每拍的串口处理权交给会话实现。
// 实现放在适配层（StudioBridge，含 SimpleFOC 依赖）；本接口不依赖任何硬件库。

namespace fockit {

class ISerialSession {
public:
  virtual ~ISerialSession() {}
  /// 会话期独占串口收发，每控制拍调用一次
  virtual void update() = 0;
  /// 诊断探针开关（默认空实现；会话实现可覆盖）。
  /// 关闭时仍应保留最低限度的"设置类按钮确认打印"，开启后追加逐条命令回显与周期快照。
  virtual void setTrace(bool /*on*/) {}
  /// 把一条原始协议行整行交给本会话处理——由 SerialShell 在检测到上位机协议命令时调用，
  /// 用于"shell 自动让位给会话"：行须含注册 ID 与行尾 eol 字符（见 Commander 的哨兵语义）。
  /// 默认空实现：非协议会话静默丢弃。
  virtual void handleLine(char* /*line*/) {}
};

} // namespace fockit
