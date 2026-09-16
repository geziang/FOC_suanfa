// FocKit 示例 02 —— 中间件速度环（VelocityNode，P0 验收 ③）
//
// 三环的中间件路径：速度PID(本库 control/) → 力矩目标 → IMotor(力矩模式)
//   → SimpleFOC 电压力矩（适配层内完成 N·m→A→V 换算并限幅）。
// 注意与内置对照组的区别：本示例的 v 指令走本库 VelocityNode；
// 若不 attach 节点，v 指令会走 SimpleFOC 内置速度环（可自行对比整定结果）。
//
// 串口：pid v <kp> <ki> <kd> 在线整定；v <rad/s> 改目标；stream 看状态流。

#include <FocKit.h>

using namespace fockit;

SimpleFocMotor motor(defaultM0Config());
VelocityNode vel;   // 中间件速度环
SerialShell shell;
PowerMonitor power;

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
  vel.begin(&motor);    // 切入力矩模式；默认增益 = 官方2208电压域基线换算
  vel.setTarget(5.0f);  // [rad/s]

  shell.begin(&motor);
  shell.attachVelocityNode(&vel);
  Serial.println("中间件速度环运行：v <rad/s> 改目标，pid v <kp> <ki> <kd> 整定");
}

void loop() {
  power.update();
  if (!power.ok()) motor.disable();

  motor.update();
  vel.update();   // 中间件速度环一拍
  shell.update();
}
