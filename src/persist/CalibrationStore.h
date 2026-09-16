#pragma once
// FocKit 持久化层 —— 标定固化
// 对标实验室 STM32 底层的“Flash 双页固化、断电重启自动注入”：
// ESP32 实现为 NVS(Preferences)。换平台时仅重写本文件，接口不变。

#include <Arduino.h>
#include <Preferences.h>

namespace fockit {

/// 单台电机的标定数据
struct MotorCalib {
  float zeroElectricAngle = 0.0f;  // [rad] 电角度零位
  int8_t sensorDirection = 0;      // +1/-1，0=未知
  uint32_t magic = 0;              // 写入有效性标记

  static constexpr uint32_t MAGIC = 0xF0CC1A11UL;  // "FOC CAL!"
  bool valid() const { return magic == MAGIC && sensorDirection != 0; }
};

class CalibrationStore {
public:
  explicit CalibrationStore(const char* ns = "fockit");
  ~CalibrationStore();

  bool save(const char* key, const MotorCalib& c);   // key ≤ 15 字符，如 "m0"
  bool load(const char* key, MotorCalib& out);
  void remove(const char* key);

private:
  Preferences prefs_;
};

} // namespace fockit
