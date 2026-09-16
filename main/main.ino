// main.ino —— FocKit 开发主入口（当前 = 示例01 电压力矩模式的副本）
//
// 用法：本仓库即 Arduino 库（根目录有 library.properties）。
//   若 IDE 未自动识别 FocKit，做一次目录联接（管理员 cmd）：
//   mklink /J "%USERPROFILE%\Documents\Arduino\libraries\FocKit" "C:\Users\21153\Desktop\simplefoc"
// 完整示例与验收标准见 docs/00_项目目标与总体方案.md §4。

#include <FocKit.h>

using namespace fockit;

SimpleFocMotor motor(defaultM0Config());
SerialShell shell;
PowerMonitor power;

float sign = 1.0f;
uint32_t lastStepMs = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  dengfoc_v4::earlyInit();  // 官方启动仪式：相线输入上拉 + 12bit ADC
  power.begin();
  power.waitReady();        // 阻塞等待 12V 就绪

  if (!motor.init()) {      // 首次烧录自动标定并写入 NVS，之后免标定
    Serial.println("电机初始化失败：检查编码器/相线/供电后重新上电");
    while (true) delay(1000);
  }

  motor.enable();
  motor.setMode(ControlMode::Torque);
  motor.setTorqueTarget(0.01f);

  shell.begin(&motor);
}

void loop() {
  uint32_t now = millis();
  if (now - lastStepMs >= 3000) {  // 3 秒一拍：±0.01 N·m 力矩台阶
    lastStepMs = now;
    sign = -sign;
    motor.setTorqueTarget(0.01f * sign);
  }

  power.update();
  if (!power.ok()) motor.disable();

  motor.update();
  shell.update();
}
