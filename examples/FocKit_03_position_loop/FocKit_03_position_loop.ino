// FocKit 示例 03 —— 级联位置环（PositionNode：位置环 → 速度环 → 力矩）
//
// 三环级联全路径：
//   位置PID → 速度目标 → 速度PID(内层, velocity() 可单独整定) → 力矩目标 → IMotor
// 行为：每 4 秒在 0 ↔ π rad 之间切换位置目标（可先跑 02 整定好速度环再上来）。
// 串口：p <rad> 直接给位置目标；pid p / pid v 分别整定外/内环。

#include <FocKit.h>

using namespace fockit;

SimpleFocMotor motor(defaultM0Config());
PositionNode pos;   // 级联位置环（内含 VelocityNode）
SerialShell shell;
PowerMonitor power;

uint32_t lastSweepMs = 0;
float sweepTarget = 0.0f;

void setup() {
  Serial.begin(115200);
  delay(300);
  dengfoc_v4::earlyInit();
  power.begin();
  power.waitReady();

  if (!motor.init()) {
    while (true) delay(1000);
  }

  motor.enable();
  pos.begin(&motor);
  pos.setTarget(0.0f);

  shell.begin(&motor);
  shell.attachPositionNode(&pos);
  Serial.println("级联位置环运行：4 秒一拍 0 ↔ π rad");
}

void loop() {
  uint32_t now = millis();
  if (now - lastSweepMs >= 4000) {
    lastSweepMs = now;
    sweepTarget = (sweepTarget == 0.0f) ? 3.14159f : 0.0f;
    pos.setTarget(sweepTarget);
  }

  power.update();
  if (!power.ok()) motor.disable();

  motor.update();
  pos.update();
  shell.update();
}
