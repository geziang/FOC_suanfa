#include "persist/CalibrationStore.h"

namespace fockit {

// C++11：constexpr 静态成员需要类外定义（odr-use）
constexpr uint32_t MotorCalib::MAGIC;

CalibrationStore::CalibrationStore(const char* ns) {
  bool ok = prefs_.begin(ns, false);
  Serial.printf("[EXP PREFS] begin ns='%s' ret=%d\n", ns, ok ? 1 : 0);
}

CalibrationStore::~CalibrationStore() {
  Serial.println(F("[EXP PREFS] end enter"));
  prefs_.end();
  Serial.println(F("[EXP PREFS] end returned"));
}

bool CalibrationStore::save(const char* key, const MotorCalib& c) {
  size_t n = prefs_.putBytes(key, &c, sizeof(MotorCalib));
  Serial.printf("[EXP PREFS] putBytes key='%s' bytes=%u expected=%u ret=%d\n",
                key, (unsigned)n, (unsigned)sizeof(MotorCalib),
                n == sizeof(MotorCalib) ? 1 : 0);
  return n == sizeof(MotorCalib);
}

bool CalibrationStore::load(const char* key, MotorCalib& out) {
  size_t n = prefs_.getBytesLength(key);
  Serial.printf("[EXP PREFS] getBytesLength key='%s' ret=%u expected=%u\n",
                key, (unsigned)n, (unsigned)sizeof(MotorCalib));
  if (n != sizeof(MotorCalib)) {
    Serial.println(F("[EXP PREFS] load invalid length"));
    return false;
  }
  size_t got = prefs_.getBytes(key, &out, sizeof(MotorCalib));
  bool valid = got == sizeof(MotorCalib) && out.valid();
  Serial.printf("[EXP PREFS] getBytes ret=%u valid=%d zero=%.6f dir=%d magic=0x%08lX\n",
                (unsigned)got, valid ? 1 : 0, (double)out.zeroElectricAngle,
                (int)out.sensorDirection, (unsigned long)out.magic);
  return valid;
}

void CalibrationStore::remove(const char* key) {
  prefs_.remove(key);
}

} // namespace fockit
