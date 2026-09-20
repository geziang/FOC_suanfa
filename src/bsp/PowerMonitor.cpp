#include "bsp/PowerMonitor.h"

namespace fockit {

void PowerMonitor::begin(float underVolt, uint32_t periodMs) {
  Serial.printf("[EXP POWER] begin enter underVolt=%.3f periodMs=%lu\n",
                (double)underVolt, (unsigned long)periodMs);
  underVolt_ = underVolt;
  periodMs_ = periodMs;
  lastMs_ = millis();
  lastVin_ = dengfoc_v4::readVin();
  Serial.printf("[EXP POWER] readVin ret=%.4f\n", (double)lastVin_);
  Serial.println(F("[EXP POWER] begin returned"));
}

void PowerMonitor::waitReady() {
  Serial.println(F("[EXP POWER] waitReady enter"));
  uint32_t dropped = 0;
  float vin = dengfoc_v4::readVin();
  Serial.printf("[EXP POWER] waitReady readVin=%.4f threshold=%.4f\n",
                (double)vin, (double)underVolt_);
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
    Serial.printf("[EXP POWER] waitReady loop readVin=%.4f\n", (double)vin);
  }
  lastVin_ = vin;
  Serial.printf("电源就绪 %.2fV\n", vin);
  if (dropped > 0) {
    Serial.printf("[提示] 电源就绪前丢弃 %lu 字节串口数据；请在“主程序就绪”后再连接上位机\n",
                  (unsigned long)dropped);
  }
  Serial.println(F("[EXP POWER] waitReady returned"));
}

void PowerMonitor::update() {
  uint32_t now = millis();
  if (now - lastMs_ < periodMs_) return;
  lastMs_ = now;
  lastVin_ = dengfoc_v4::readVin();
  // 常态静默：周期打印收敛为应用层 1 行/秒心跳（loop() 的 [FW LOOP] alive 携带 vin/ok），
  // 此处仅在状态翻转时打印（触发式），不再每次检测都刷两行（DD-04 §4.3 节流）。
  // 需要逐次读数排查时调用 setPeriodicVerbose(true)（随 shell 的 dbg on 同步）。
  if (periodicVerbose_) {
    Serial.printf("[EXP POWER] update readVin=%.4f threshold=%.4f\n",
                  (double)lastVin_, (double)underVolt_);
  }

  if (lastVin_ < underVolt_) {
    // 连续5次复测确认，避免毛刺误触发（官方例程同款）
    uint8_t count = 5;
    while (count--) {
      float v = dengfoc_v4::readVin();
      if (periodicVerbose_) {
        Serial.printf("[EXP POWER] recheck=%u vin=%.4f\n", (unsigned)(5 - count), (double)v);
      }
      if (v > underVolt_) {
        lastVin_ = v;
        if (periodicVerbose_) {
          Serial.println(F("[EXP POWER] update returned: transient recovery"));
        }
        return;
      }
    }
    if (ok_) Serial.printf("[保护] 欠压 %.2fV < %.2fV，请立即失使能电机\n",
                           lastVin_, underVolt_);
    ok_ = false;
  } else if (!ok_) {
    ok_ = true;
    Serial.printf("[保护] 电压恢复 %.2fV\n", lastVin_);
  }
  if (periodicVerbose_) {
    Serial.printf("[EXP POWER] update returned ok=%d\n", ok_ ? 1 : 0);
  }
}

} // namespace fockit
