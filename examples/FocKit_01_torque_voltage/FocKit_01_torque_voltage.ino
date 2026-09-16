// FocKit 示例 01 —— 电压力矩模式（P0 验收 ①②）
//
// 硬件：DengFOC V4 + 2208 云台电机(7对极) + AS5600（接 M0 编码器口：SDA19/SCL18）
// 供电：DC 12V（≥11.1V，欠压自动失使能）
//
// 行为：上电等待 → 初始化（首次烧录自动做编码器零位/方向标定并写入 NVS，
//       断电重启自动注入，不再重复标定）→ 使能 → 每 3 秒 ±0.01 N·m 力矩台阶。
// 串口：115200，输入 help 查看命令，stream 开 10Hz 状态流（串口绘图器可用）。

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
  power.waitReady();        // 阻塞等待 12V 就绪（官方例程同款流程）

  if (!motor.init()) {
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
  if (!power.ok()) motor.disable();  // 欠压保护（契约内的安全行为）

  motor.update();  // 底层闭环一拍 + 状态刷新
  shell.update();  // 串口调参
}
