// FocKit 唯一示例 —— 电机三环整定主程序
// （commissioning：工业伺服上电调试惯例，由内到外 电流环 → 速度环 → 位置环）
//
// 硬件：DengFOC V4 + 2208 云台电机(7对极) + AS5600（M0 编码器口：SDA19/SCL18）
// 供电：DC 12V（≥11.1V，欠压自动失使能）；首次烧录自动做编码器零位/方向标定并写入
//       NVS，断电重启自动注入，不再重复标定。
// 本程序集成全部上机场景：电压力矩冒烟、速度/位置/电流环整定、SimpleFOC Studio 上位机会话。
//
// ============================================================
// 方法论：计算优先（SPEC-T §1 / REF-12 §4.2）
//   建模 → 取标称参数 → 选带宽 → 公式算增益 → 上机在计算值附近微调 → 抄回代码
// 参数来源优先级：① 官方标称(HW-DENG §2) ② 辨识(本平台仅交叉验证/未知量收口)
//                ③ 反推估算(须标注来源) —— 非高精度系统，标称优先，忽略批次差异
//
// —— 建模与带宽计算区依据（可溯源）——
// 级联带宽分配（内→外递减，工业惯例 5~10 倍）：
//   电流环 ωc = 125 rad/s（τc=8ms；2026-09-21 定档，见下方抄回注记）
//   速度环 ωv = 20 rad/s（受电压力矩通道与速度 LPF(10ms) 约束，取保守）
//   位置环 ωp = ωv/5 = 4 rad/s
// 电流环（对象 G=1/(Ls+R)，PI 零极对消法）：
//   kp_i = L·ωc = 4.25mH×125 = 0.53 [V/A]
//   ki_i = R·ωc = 8.25Ω×125  = 1031 [V/A/s]（校验：ki/kp=R/L ✓）
// 速度环（级联版 2026-09-21：对象 = 冻结电流环(ωc=125，近似直通) × KT × 1/(Js)）：
//   kp_v = ωv·J/KT = 0.0025 [A/(rad/s)]（R 由电流内环接管，建模不再含 R）
//   J 来源②：2026-09-21 随 KT 实测定标等比更正（③反推值 ×KT实/KT表）
//     → J ≈ 4.06e-6 kg·m²（取值保 kpV=0.00254 与在线整定零漂移；P3 spin-up 交叉验证待做）
//   ki_v = N×kp_v，N=10 → 0.0254（ζ=½√(ωv/N)=0.707；2026-09-21 探针验证写入
//     落地 + 10~40 rad/s 平滑收敛；低速边界 5~10 rad/s 停-走区见 T-P1-6）
//   输出限幅 = 0.5A 持续红线（速度 PID 输出即 Iq 指令，级联安全钳位）
//   （电压域旧版 kp=0.021/ki=0.105 复现官方基线，为对照历史存档，见 REF-12 §4.2）
// 位置环（内环近似理想积分器，P 控制）：
//   kp_p = ωp = 4 [1/s]（官方 P=20 对应更激进 ωp，空载可用；加载按超调回退）
// ============================================================
//
// 使用（详见 docs REF-12 §4.2）：
//   loop t        电压力矩会话：step on 跑 ±0.01 N·m 方波（3s 一拍，链路冒烟）
//   loop v        速度环会话：step 方波(±3 rad/s, 2s)，Studio 微调 MVP/MVI
//   loop p        位置环会话：step 方波(±π/2 rad, 4s)，Studio 微调 MAP
//   loop i        电流环会话走 Studio：studio 进会话 → 界面切 MT2(foc_current)
//                 → 目标滑杆小幅阶跃(≤0.15A，持续红线 0.5A) → MQP/MQI 在计算值附近微调
//   step on|off / step amp <x> / step period <ms>   自动方波激励（幅值单位随当前会话）
//   gains         重打印计算增益；t/v/p <目标> 直接给目标；stream 开 10Hz 状态流
//   dbg on|off    详细日志开关（默认关）：on 追加 Studio 逐条命令回显 + 1s 状态快照，
//                 并打开固件周期探针（PowerMonitor 逐次读数）。常态串口只有 1 行/秒心跳：
//                 [FW LOOP] alive vin=..V ok=.. mode=.. tgt=..
//   studio        进 SimpleFOC Studio 上位机会话（退出按板上 EN/RST 复位）
//                 注：上位机连上后固件会自动进入会话（识别 M 开头协议命令），手工输仍有效
//
// 上位机连接时序（2026-09-20 零配置版，详见 REF-12 §5.1）：
//   1) 先接 12V，等打印"电源就绪""三环整定主程序就绪"（仅 USB 供电会卡在等待上电）
//   2) 启动 SimpleFOCStudio（Start-SimpleFOCStudio.cmd）——设备页自动打开，
//      前缀 M / 115200-8N1 / 拉取模式均已写死，无需任何界面配置
//   3) 选端口、点连接 —— 连上即自动进会话（固件识别 M 开头协议命令自动让位；
//      要深度探针请在连接前于终端先 dbg on）
//   4) 每次按板上 EN/RST 复位后直接重连即可，不必再手工输 studio（手工输仍然有效）
//
// 整定顺序（由内到外，内环不收敛不要进外环）：
//   loop t 链路冒烟 → loop i 电流环 → loop v 速度环 → loop p 位置环
//
// 纪律：Studio/串口改的是 RAM 值，重启即失；调好后抄回本文件计算区并提交，
//       NVS 只固化编码器标定。

#include <FocKit.h>
#include <stdlib.h>
#include <string.h>

using namespace fockit;

SimpleFocMotor motor(defaultM0Config());  // 电流采样默认接入（M0: 39/36）
SerialShell shell;
PowerMonitor power;
StudioBridge studio;

// ========== 建模与带宽计算区（计算优先，参数全部可溯源） ==========
constexpr float R_PH = dengfoc_v4::MOTOR_2208.phaseResistance;  // 8.25 Ω   ①标称
constexpr float L_PH = dengfoc_v4::MOTOR_2208.phaseInductance;  // 4.25 mH  ①标称
constexpr float KT_M = dengfoc_v4::MOTOR_2208.kt;               // 0.032    ②实测（2026-09-21 四点定标）
constexpr float J_EST = 4.06e-6f;  // kg·m² ②实测 2026-09-21（③反推等比更正；保 kpV 零漂移；P3 spin-up 待做）

// 电流环带宽：2026-09-21 T-P1-3 实测定档（TST-01）——堵转判据全绿
// （ess≤0.3%、Cd≤3%、纹波≤2%、双向对称，EXP-03）；同时受主循环 ~1.35kHz
// 数字控制上限约束（ωc ≤ loop/10）。Kp=L·ωc=0.53、Ki=R·ωc=1031、τc=8ms。
constexpr float WC = 125.0f;  // [rad/s] 电流环带宽
constexpr float WV = 20.0f;    // [rad/s] 速度环带宽
constexpr float WP = 4.0f;     // [rad/s] 位置环带宽（ωv/5）

float kpI, kiI, kpV, kiV, kpP;

void computeGains() {
  Serial.println(F("[FW BOOT] computeGains enter"));
  kpI = L_PH * WC;                   // 4.25  [V/A]
  kiI = R_PH * WC;                   // 8250  [V/A/s]
  kpV = WV * J_EST / KT_M;         // 0.0025 [A/(rad/s)] 级联版：踩冻结电流环，R 已除
  kiV = 10.0f * kpV;               // 0.0254  N=10（ζ=0.707；2026-09-21 定档，低速边界 5~10 见 T-P1-6）
  kpP = WP;                          // 4     [1/s]
  Serial.printf("[整定] 参数: R=%.2fΩ L=%.2fmH KT=%.4f J=%.2e kg·m²(③反推,P3收口)\n",
                (double)R_PH, (double)(L_PH * 1000.0f), (double)KT_M, (double)J_EST);
  Serial.printf("[整定] 带宽: ωc=%.0f ωv=%.0f ωp=%.0f rad/s\n",
                (double)WC, (double)WV, (double)WP);
  Serial.printf("[整定] 电流环 kp=%.2f ki=%.0f（ωc=125 定档，T-P1-3 已验收）\n",
                (double)kpI, (double)kiI);
  Serial.printf("[整定] 速度环 kp=%.4f ki=%.4f A/(rad/s)（级联版：踩冻结电流环，"
                "R 已除，限幅 0.5A；电压域旧基线 0.021 为对照存档）\n",
                (double)kpV, (double)kiV);
  Serial.printf("[整定] 位置环 kp=%.1f（官方 20 激进，空载可用）\n", (double)kpP);
  Serial.println(F("[FW BOOT] computeGains returned"));
}

void applyGains() {
  Serial.println(F("[FW BOOT] applyGains enter"));
  motor.setLoopGains(LoopType::CurrentQ, LoopGains(kpI, kiI, 0.0f, 0.002f));
  motor.setLoopGains(LoopType::CurrentD, LoopGains(kpI, kiI, 0.0f, 0.002f));
  motor.setLoopGains(LoopType::Velocity,
                     LoopGains(kpV, kiV, 0.0f, 0.01f, 0.5f));  // 0.5A=持续红线钳位
  motor.setLoopGains(LoopType::Position, LoopGains(kpP));
  Serial.println("[整定] 三环增益已按计算值注入（Studio 拉取可见）");
  Serial.println(F("[FW BOOT] applyGains returned"));
}

// ========== 阶跃激励（波形可复现、指标可量化，不靠手拖滑杆） ==========
float stepAmp = 3.0f;             // 方波幅值，单位随会话：力矩[N·m]/速度[rad/s]/位置[rad]
uint32_t stepPeriodMs = 2000;
bool stepOn = false;
bool stepHigh = false;
uint32_t lastStepMs = 0;

// ========== 自定义整定命令（经 shell 的 attachUserCommand 转发） ==========
bool tuningCommands(int argc, char* argv[]) {
  if (argc < 1) return false;

  if (!strcmp(argv[0], "loop") && argc >= 2) {
    stepOn = false;  // 换会话先停激励，参数就位后再 step on
    motor.setTarget(0);
    if (argv[1][0] == 't') {
      motor.setMode(ControlMode::Torque);
      stepAmp = 0.01f;      // [N·m] 电压力矩（适配层内完成 N·m→A→V 换算并限幅）
      stepPeriodMs = 3000;
      Serial.println(F("[整定] 电压力矩会话：step on 开始 ±0.01 N·m 方波（3s 一拍）"));
    } else if (argv[1][0] == 'v') {
      motor.setMode(ControlMode::Velocity);
      stepAmp = 3.0f;
      stepPeriodMs = 2000;
      Serial.println(F("[整定] 速度环会话：step on 开始方波；Studio 中微调 MVP/MVI"));
    } else if (argv[1][0] == 'p') {
      motor.setMode(ControlMode::Position);
      stepAmp = 1.5708f;
      stepPeriodMs = 4000;
      Serial.println(F("[整定] 位置环会话：step on 开始方波；Studio 中微调 MAP"));
    } else if (argv[1][0] == 'i') {
      motor.setMode(ControlMode::Idle);
      Serial.println(F("[整定] 电流环会话走 Studio：studio 进会话 → 界面切 MT2(foc_current)"));
      Serial.println(F("       → 目标滑杆小幅阶跃(≤0.15A，持续红线 0.5A) → MQP/MQI 在计算值附近微调"));
    } else {
      return false;
    }
    return true;
  }

  if (!strcmp(argv[0], "step")) {
    if (argc >= 2 && !strcmp(argv[1], "on")) {
      stepOn = true;
      lastStepMs = 0;
      Serial.println(F("[FW TUNE] step enabled"));
    } else if (argc >= 2 && !strcmp(argv[1], "off")) {
      stepOn = false;
      motor.setTarget(0);
      Serial.println(F("[FW TUNE] step disabled"));
    } else if (argc >= 3 && !strcmp(argv[1], "amp")) {
      stepAmp = strtof(argv[2], nullptr);
    } else if (argc >= 3 && !strcmp(argv[1], "period")) {
      stepPeriodMs = (uint32_t)atol(argv[2]);
    } else {
      return false;
    }
    Serial.printf("[整定] step %s amp=%.3f period=%lu ms\n",
                  stepOn ? "on" : "off", (double)stepAmp, (unsigned long)stepPeriodMs);
    return true;
  }

  if (!strcmp(argv[0], "gains")) {
    computeGains();
    return true;
  }
  return false;
}

// ========== 详细日志开关（shell 的 dbg on|off 联动） ==========
// 常态节流：串口只有 loop() 里 1 行/秒的心跳；逐次周期探针按需打开（DD-04 §4.3）。
void applyVerbose(bool on) {
  power.setPeriodicVerbose(on);
}

void setup() {
  Serial.setTxBufferSize(512);  // 突发打印入环形缓冲，不打断 FOC 主循环（须在 begin 前调用）
  Serial.begin(115200);
  Serial.println(F("[FW BOOT] Serial.begin done"));
  delay(300);
  bool storageOk = motor.beginStorage();
  Serial.printf("[FW BOOT] motor.beginStorage returned ok=%d\n", storageOk ? 1 : 0);
  dengfoc_v4::earlyInit();
  Serial.println(F("[FW BOOT] earlyInit done"));
  power.begin();
  Serial.println(F("[FW BOOT] power.begin done"));
  power.waitReady();
  Serial.println(F("[FW BOOT] power.waitReady returned"));

  Serial.println(F("[FW BOOT] motor.init enter"));
  if (!motor.init()) {  // 电流采样已随初始化链接（InlineCurrentSense, 0.5V/A）
    Serial.println(F("[FW BOOT] motor.init failed; halted"));
    while (true) delay(1000);
  }
  Serial.println(F("[FW BOOT] motor.init returned ok"));

  computeGains();  // 计算优先：开机即算、即打印
  applyGains();    // 注入三环（契约 setLoopGains，Studio 拉取即见计算值）

  motor.enable();
  motor.setMode(ControlMode::Idle);

  studio.begin(&motor);
  Serial.println(F("[FW BOOT] studio.begin returned"));
  shell.begin(&motor);
  Serial.println(F("[FW BOOT] shell.begin returned"));
  shell.attachStudio(&studio);
  Serial.println(F("[FW BOOT] shell.attachStudio done"));
  shell.attachUserCommand(tuningCommands);
  Serial.println(F("[FW BOOT] shell.attachUserCommand done"));
  shell.attachVerboseHook(applyVerbose);  // dbg on|off 同步固件周期探针
  Serial.println(F("[FW BOOT] shell.attachVerboseHook done"));
  Serial.println(F("三环整定主程序就绪。命令：loop t|v|p|i / step on|off|amp|period / gains / t|v|p / stream / dbg on|off / studio（help 查全部）"));
  Serial.println(F("[FW BOOT] 常态串口仅 1 行/秒心跳；详细逐次探针用 dbg on 打开"));
  Serial.println(F("[FW BOOT] setup complete"));
}

void loop() {
  static uint32_t lastLoopLogMs = 0;
  static bool lastPowerOk = false;
  power.update();
  bool powerOk = power.ok();
  if (powerOk != lastPowerOk) {
    lastPowerOk = powerOk;
    Serial.print(F("[FW LOOP] power.ok="));
    Serial.println(powerOk ? F("true") : F("false"));
  }
  if (!powerOk) motor.disable();

  if (stepOn && millis() - lastStepMs >= stepPeriodMs) {  // 方波目标激励
    lastStepMs = millis();
    stepHigh = !stepHigh;
    motor.setTarget(stepHigh ? stepAmp : -stepAmp);
    Serial.print(F("[FW LOOP] step target="));
    Serial.println(stepHigh ? stepAmp : -stepAmp, 4);
  }

  motor.update();
  shell.update();  // studio 会话期自动转为上位机通道，step 激励照常运行
  uint32_t now = millis();
  // 稳态唯一周期行：1 行/秒（节流见 DD-04 §4.3）。母线电压/欠压状态/当前模式/目标
  // 一并承载，替代原先分散在 .ino 与 PowerMonitor 里的每秒 3 行。
  // 前缀 '[' 且不含 PID/Motion/Status 等英文标记，SimpleFOC Studio 解析器会忽略，不污染通道。
  if (now - lastLoopLogMs >= 1000) {
    lastLoopLogMs = now;
    Serial.printf("[FW LOOP] alive vin=%.2fV ok=%d mode=%s tgt=%.3f\n",
                  (double)power.voltage(), power.ok() ? 1 : 0,
                  controlModeName(motor.getState().mode), (double)motor.getTarget());
  }
}
