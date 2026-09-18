#include "bsp/PowerMonitor.h"

namespace fockit {

void PowerMonitor::begin(float underVolt, uint32_t periodMs) {
  underVolt_ = underVolt;
  periodMs_ = periodMs;
  lastMs_ = millis();
  lastVin_ = dengfoc_v4::readVin();
}

void PowerMonitor::waitReady() {
  uint32_t dropped = 0;
  float vin = dengfoc_v4::readVin();
  while (vin <= underVolt_) {
    Serial.printf("等待上电, 当前电压%.2fV\n", vin);
    delay(100);
    // 未就绪期间 shell/Commander 均未启动，若上位机此刻已连接并连发命令
    // （如 SimpleFOC Studio 的 pull config），接收缓冲会被灌满并导致对端 Write timeout。
    // 这里直接排空丢弃：未就绪阶段收到的任何命令本就无法处理。
    while (Serial.available()) {
      Serial.read();
      ++dropped;
    }
    vin = dengfoc_v4::readVin();
  }
  lastVin_ = vin;
  Serial.printf("电源就绪 %.2fV\n", vin);
  if (dropped > 0) {
    Serial.printf("[提示] 电源就绪前丢弃 %lu 字节串口数据；请在“主程序就绪”后再连接上位机\n",
                  (unsigned long)dropped);
  }
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
