#include "bsp/PowerMonitor.h"

namespace fockit {

void PowerMonitor::begin(float underVolt, uint32_t periodMs) {
  underVolt_ = underVolt;
  periodMs_ = periodMs;
  lastMs_ = millis();
  lastVin_ = dengfoc_v4::readVin();
}

void PowerMonitor::waitReady() {
  float vin = dengfoc_v4::readVin();
  while (vin <= underVolt_) {
    Serial.printf("等待上电, 当前电压%.2fV\n", vin);
    delay(100);
    vin = dengfoc_v4::readVin();
  }
  lastVin_ = vin;
  Serial.printf("电源就绪 %.2fV\n", vin);
}

void PowerMonitor::update() {
  uint32_t now = millis();
  if (now - lastMs_ < periodMs_) return;
  lastMs_ = now;
  lastVin_ = dengfoc_v4::readVin();

  if (lastVin_ < underVolt_) {
    // 连续5次复测确认，避免毛刺误触发（官方例程同款）
    uint8_t count = 5;
    while (count--) {
      float v = dengfoc_v4::readVin();
      if (v > underVolt_) { lastVin_ = v; return; }
    }
    if (ok_) Serial.printf("[保护] 欠压 %.2fV < %.2fV，请立即失使能电机\n",
                           lastVin_, underVolt_);
    ok_ = false;
  } else if (!ok_) {
    ok_ = true;
    Serial.printf("[保护] 电压恢复 %.2fV\n", lastVin_);
  }
}

} // namespace fockit
