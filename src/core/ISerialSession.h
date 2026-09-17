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
};

} // namespace fockit
