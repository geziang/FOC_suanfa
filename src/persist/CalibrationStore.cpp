#include "persist/CalibrationStore.h"

namespace fockit {

// C++11：constexpr 静态成员需要类外定义（odr-use）
constexpr uint32_t MotorCalib::MAGIC;

CalibrationStore::CalibrationStore(const char* ns) {
  prefs_.begin(ns, false);
}

CalibrationStore::~CalibrationStore() {
  prefs_.end();
}

bool CalibrationStore::save(const char* key, const MotorCalib& c) {
  size_t n = prefs_.putBytes(key, &c, sizeof(MotorCalib));
  return n == sizeof(MotorCalib);
}

bool CalibrationStore::load(const char* key, MotorCalib& out) {
  size_t n = prefs_.getBytesLength(key);
  if (n != sizeof(MotorCalib)) return false;
  prefs_.getBytes(key, &out, sizeof(MotorCalib));
  return out.valid();
}

void CalibrationStore::remove(const char* key) {
  prefs_.remove(key);
}

} // namespace fockit
