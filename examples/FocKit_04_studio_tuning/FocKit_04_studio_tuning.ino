// FocKit 示例 04 —— SimpleFOC Studio 上位机调参会话（P0 验收 ⑥）
//
// 上位机：SimpleFOCStudio（本地 EXE 版，位于仓库根目录 SimpleFOCStudio_EXE*，不入 git）
// 用法：烧录后串口 115200 输入 studio → shell 让位 → 打开 SimpleFOCStudio
//       选对应串口连接 → 实时曲线（目标/速度/角度）+ 在线改内置环 PID/限幅/模式。
// 纪律：Studio 改的是 RAM 值，重启即失；调好后把增益抄回代码/配置，
//       NVS 只固化编码器标定（详见 docs/02 ARC-01 与 StudioBridge.h 头注释）。
// 退出会话：按板上 EN/RST 复位，重启后回到 FocKit shell。

#include <FocKit.h>

using namespace fockit;

SimpleFocMotor motor(defaultM0Config());
SerialShell shell;
PowerMonitor power;
StudioBridge studio;  // Commander(Serial)，'M' 电机命令集

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
  motor.setMode(ControlMode::Idle);

  studio.begin(&motor);   // 注册 Commander 'M' + monitor 变量
  shell.begin(&motor);
  shell.attachStudio(&studio);
  Serial.println("输入 studio 进入 SimpleFOC Studio 会话（退出按复位）");
}

void loop() {
  power.update();
  if (!power.ok()) motor.disable();

  motor.update();  // 会话期间底层闭环照常运行
  shell.update();  // studio 模式下自动转为会话通道
}
