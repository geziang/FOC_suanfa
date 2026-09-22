// FocKit 02 号示例 —— P2 力控主程序（第一轮：骨架 + raw/pend/imp）
// （force_ctrl：交互控制实验台，与三环平级、直接跑力矩通道 MT2+MC0，REF-14）
//
// 硬件：DengFOC V4 + 2208 云台电机(7对极) + AS5600（M0 编码器口：SDA19/SCL18）
// 供电：DC 12V（≥11.1V，欠压自动失使能）；标定沿用 NVS（与 01 号共用，无需重标）。
// 交互：裸电机 + 手拨（无连杆平台，控制算法学习导向，REF-14 §1）。
//
// ============================================================
// 五模式一骨架（REF-14 §6，两轮交付）：
//   全部模式共用 "读 θ/θ̇ → 算力律 T(·) → setTorqueTarget"，
//   每个模式 = 一条十几行的力律；本文件为第一轮三模式：
//     raw   裸力矩   T = tq                      —— setTorque 通道抽测（T-P2-1①）
//     pend  虚拟摆   T = -Gv·sin(θ+th0)           —— 虚拟重力前馈（T-P2-1②③④）
//     imp   阻抗     T = K(θ*-θ) - D·θ̇           —— 虚拟弹簧+阻尼（T-P2-2）
//   第二轮将追加（枚举与参数位已预留）：
//     wall  虚拟墙   单边弹簧（T-P2-3）
//     traj  柔顺轨迹 阻抗绕梯形 θ*(t)（T-P2-5，还 T-P1-8 斜坡挂账）
//
// —— 计算优先（与 01 号同源，SPEC-T §1 / REF-14 §3）——
// 电流环：沿用 TST-01 定档 ωc=125（kp=L·ωc=0.53、ki=R·ωc=1031、Tf=2ms）
// 阻抗起步档（REF-14 §3.2，摩擦地板推窗口上沿）：
//   ωn=25（ωc/5）、ζ=1 → K=J·ωn²=2.54e-3 N·m/rad、D=2ζ√(KJ)=2.03e-4 N·m·s/rad
//   破摩死区 θ_b=τ_f/K≈0.35 rad(20°)；饱和校核 θerr_max=0.016/K≈6.3 rad 全行程线性
// 虚拟摆默认 Gv=5 mN·m（连续力矩 31%）：
//   ω_pend=√(Gv/J)≈35 rad/s(5.6Hz)；半摆衰减 ΔA=2τ_f/Gv≈0.36 rad；停摆角 asin(τ_f/Gv)≈10°
// 上述预测量开机即算即打印（pred 命令重看）——判据全部是预测-实测对照（REF-14 §4）。
//
// 使用：
//   mode raw|pend|imp   切力律（imp 请在 θ≈θ* 附近开：多圈差 2π 会饱和到限幅，拨回即可）
//   gv|th0|kk|kd|tp|tq <值>  在线调参（Gv/θ0/K/D/θ*/裸力矩），写入即回显读回值
//   pred                重打印预测量计算链
//   probe <ms>|off      [FC DBG] 探针周期（默认 50ms；摆频 5.6Hz 需 ≥10 点/周期）
//   tq ±2e-3/±5e-3/±1e-2  T-P2-1① 堵转对账阶梯（手按转子看 [FC DBG] iq 列）
//   stream / studio / dbg / help —— 沿用 01 号全套（上位机一级零改动，REF-14 §5）
//
// 纪律：
//   ① 写入必读回（EXP-04：参数卡/命令显示不作数，回显的存储值才算数——本程序所有
//      调参命令内置回显）；
//   ② 力律是唯一权威：每控制拍重算重发目标，Studio 滑杆/shell t 命令会被本循环
//      覆盖（速度环失控案的三重防线思想，REF-12 §4.2）；
//   ③ 力矩限幅 0.016 N·m 双保险（本文件 constrain + 适配层 DEFAULT_TORQUE_LIMIT）；
//   ④ 探针行以 '[' 开头，SimpleFOC Studio 解析器忽略，不污染绘图通道（同 01 号心跳）。
// ============================================================

#include <FocKit.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

using namespace fockit;

SimpleFocMotor motor(defaultM0Config());  // 电流采样默认接入（M0: 39/36），标定与 01 号共用 NVS
SerialShell shell;
PowerMonitor power;

// ========== 计算区（参数全部可溯源，HW-DENG / REF-14 §3） ==========
constexpr float L_PH  = dengfoc_v4::MOTOR_2208.phaseInductance;  // 4.25 mH  ①标称
constexpr float R_PH  = dengfoc_v4::MOTOR_2208.phaseResistance;  // 8.25 Ω   ①标称
constexpr float KT_M  = dengfoc_v4::MOTOR_2208.kt;               // 0.032    ②实测四点定标
constexpr float J_EST = 4.06e-6f;   // kg·m² ②实测（与 01 号同源，TST-02）
constexpr float TAU_F = 0.9e-3f;    // [N·m] 静摩擦 ②实测（卡死段 vq≈0.24V→29mA×KT，EXP-05）
constexpr float T_LIM = dengfoc_v4::DEFAULT_TORQUE_LIMIT;  // 0.016 N·m = 0.5A×KT 持续红线
constexpr float WC    = 125.0f;     // [rad/s] 电流环带宽（TST-01 定档，力矩通道地基）

// ========== 力律参数（全部在线可调，默认=REF-14 §3 定档） ==========
float gvPend  = 5.0e-3f;    // [N·m] 虚拟重力幅值 Gv（连续力矩 31%）
float th0Pend = 0.0f;       // [rad] 虚拟摆悬垂偏置 θ0（虚拟量任选，无需标定）
float kImp    = 2.54e-3f;   // [N·m/rad] 阻抗刚度 K（ωn=25 起步档）
float dImp    = 2.03e-4f;   // [N·m·s/rad] 阻尼 D（ζ=1）
float thStar  = 0.0f;       // [rad] 阻抗目标 θ*
float tqRaw   = 0.0f;       // [N·m] raw 模式裸力矩命令

// ========== 模式（第二轮追加 FC_WALL/FC_TRAJ，REF-14 §6） ==========
enum class FcMode : uint8_t { Raw, Pend, Imp };
FcMode fcMode = FcMode::Raw;  // 安全启动：raw+零力矩（pend 开机即摆动，由用户显式切入）
const char* fcModeName(FcMode m) {
  switch (m) {
    case FcMode::Raw:  return "raw";
    case FcMode::Pend: return "pend";
    case FcMode::Imp:  return "imp";
  }
  return "?";
}

// ========== [FC DBG] 探针（时基锚点，EXP-06 纪律：波形时间轴以探针节拍标定） ==========
uint32_t probePeriodMs = 50;   // 摆频 5.6Hz 需 ≥10 点/周期 → ≤18ms；50ms 日常够用，精细判读调小
bool probeOn = true;
uint32_t lastProbeMs = 0;

// ========== 预测量计算链（计算优先：开机即算即打印，pred 重看） ==========
void computePredictions() {
  float omegaPend = sqrtf(gvPend / J_EST);
  float tPend = 2.0f * (float)M_PI / omegaPend;
  float dAmp = 2.0f * TAU_F / gvPend;                       // 半摆振幅衰减（库仑摩擦）
  float thStick = asinf(fminf(TAU_F / gvPend, 1.0f));       // 停摆死区
  float wn = sqrtf(kImp / J_EST);
  float zeta = dImp / (2.0f * sqrtf(kImp * J_EST));
  float thBreak = TAU_F / kImp;                             // 破摩死区
  float thSat = T_LIM / kImp;                               // 弹簧线性全程校核
  float tsEst = 4.0f / (zeta * wn);
  Serial.printf("[FC PRED] 参数源: KT=%.4f J=%.2e τf=%.1e N·m(②实测) 限幅=%.3f N·m\n",
                (double)KT_M, (double)J_EST, (double)TAU_F, (double)T_LIM);
  Serial.printf("[FC PRED] pend: ω=%.1f rad/s(%.2fHz) 半摆衰减ΔA=%.3f rad 停摆角=%.1f°\n",
                (double)omegaPend, (double)(omegaPend / (2.0f * (float)M_PI)),
                (double)dAmp, (double)(thStick * 57.2958f));
  Serial.printf("[FC PRED] imp: ωn=%.1f rad/s ζ=%.2f 破摩死区=%.3f rad(%.0f°) 饱和θerr=%.1f rad ts≈%.2fs\n",
                (double)wn, (double)zeta, (double)thBreak, (double)(thBreak * 57.2958f),
                (double)thSat, (double)tsEst);
  Serial.printf("[FC PRED] raw: tq=%.4f N·m → 账面 Iq=%.3f A（堵转对账基准，T-P2-1①）\n",
                (double)tqRaw, (double)(tqRaw / KT_M));
}

void printParams() {  // 调参回显（写入必读回，EXP-04）
  Serial.printf("[FC CFG] mode=%s gv=%.4e th0=%.3f k=%.4e d=%.4e tp=%.3f tq=%.4e probe=%lu%s\n",
                fcModeName(fcMode), (double)gvPend, (double)th0Pend, (double)kImp,
                (double)dImp, (double)thStar, (double)tqRaw,
                (unsigned long)probePeriodMs, probeOn ? "ms" : "off");
}

// ========== 力律（五模式一骨架的核心：每模式 = 一条 T(θ,θ̇)） ==========
float computeLaw(float th, float sv) {
  switch (fcMode) {
    case FcMode::Raw:  return tqRaw;
    case FcMode::Pend: return -gvPend * sinf(th + th0Pend);
    case FcMode::Imp:  return kImp * (thStar - th) - dImp * sv;
    // 第二轮追加：wall 单边弹簧 / traj 阻抗绕梯形 θ*(t)（REF-14 §4）
  }
  return 0.0f;
}

// ========== 力控命令（经 shell attachUserCommand 转发；命名避开 shell 内建 t/v/p） ==========
void applyCurrentLoopGains() {  // 沿用 TST-01 定档：力矩通道=冻结电流环（REF-14 §1 地基）
  motor.setLoopGains(LoopType::CurrentQ, LoopGains(L_PH * WC, R_PH * WC, 0.0f, 0.002f));
  motor.setLoopGains(LoopType::CurrentD, LoopGains(L_PH * WC, R_PH * WC, 0.0f, 0.002f));
  Serial.printf("[FC BOOT] 电流环增益注入: kp=%.2f ki=%.0f（ωc=%.0f，TST-01 定档）\n",
                (double)(L_PH * WC), (double)(R_PH * WC), (double)WC);
}

bool fcCommands(int argc, char* argv[]) {
  if (argc < 1) return false;

  if (!strcmp(argv[0], "mode") && argc >= 2) {
    if (argv[1][0] == 'r')      fcMode = FcMode::Raw;
    else if (argv[1][0] == 'p') fcMode = FcMode::Pend;
    else if (argv[1][0] == 'i') fcMode = FcMode::Imp;
    else return false;
    if (fcMode == FcMode::Raw) tqRaw = 0.0f;  // 切回 raw 先归零（防旧 tq 突袭）
    Serial.printf("[FC CFG] mode=%s（imp 请在 θ≈θ* 附近开）\n", fcModeName(fcMode));
    computePredictions();
    return true;
  }

  // 调参：写入即回显存储值（EXP-04 纪律：回显才算数）。两段式：`gv 0.005` → argc=2、值在 argv[1]
  if (argc >= 2) {
    float v = strtof(argv[1], nullptr);
    bool hit = true;
    if      (!strcmp(argv[0], "gv"))  gvPend  = v;
    else if (!strcmp(argv[0], "th0")) th0Pend = v;
    else if (!strcmp(argv[0], "kk"))  kImp    = v;
    else if (!strcmp(argv[0], "kd"))  dImp    = v;
    else if (!strcmp(argv[0], "tp"))  thStar  = v;
    else if (!strcmp(argv[0], "tq"))  tqRaw   = v;
    else hit = false;
    if (hit) { printParams(); return true; }
  }

  if (!strcmp(argv[0], "pred")) { computePredictions(); printParams(); return true; }

  if (!strcmp(argv[0], "probe")) {
    if (argc >= 2 && !strcmp(argv[1], "off")) { probeOn = false; }
    else if (argc >= 2) {
      long p = atol(argv[1]);
      probeOn = p >= 10;                 // 下限 10ms（防 0/负值刷爆串口）
      if (probeOn) probePeriodMs = (uint32_t)p;
    }
    Serial.printf("[FC CFG] probe=%lu%s\n", (unsigned long)probePeriodMs, probeOn ? "ms" : "off");
    return true;
  }
  return false;
}

void applyVerbose(bool on) { power.setPeriodicVerbose(on); }

// ========== 力控会话桥（Studio 会话期的小写命令出口，零库改动） ==========
// 背景：shell 让位后串口由会话独占——StudioBridge::update() 内 Commander 自己读口
// （cmd_.run()），小写力控命令（mode/gv/...）会被 Commander 吞掉。上位机协议字母
// 全大写（A~Z），力控命令全小写——判别无歧义。本包装自己做串口泵：
//   小写行 → 力控命令处理器；其余整行 → bridge_.handleLine()（与 shell 触发让位
//   同一条通路，宽容解析照旧）。bridge_.update() 仍每拍调用（保留曲线流限速闸与
//   探针侧效应；此时串口已被本泵读空，其内部 Commander 空转无副作用）。
// 非会话期（纯终端）不经本类：shell dispatch_ 直达 userCb_，两路同归 fcCommands。
class ForceSession : public ISerialSession {
public:
  void begin(SimpleFocMotor* m) { bridge_.begin(m); }
  void update() override {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (bufLen_ > 0) { buf_[bufLen_] = 0; routeLine_(buf_); bufLen_ = 0; }
      } else if (bufLen_ < (int)(sizeof(buf_) - 1)) {
        buf_[bufLen_++] = c; buf_[bufLen_] = 0;
      }
    }
    bridge_.update();
  }
  void setTrace(bool on) override { bridge_.setTrace(on); }
  void handleLine(char* line) override { if (line != nullptr) routeLine_(line); }
private:
  void routeLine_(char* line) {
    if (line[0] >= 'a' && line[0] <= 'z') {  // 小写 = 力控命令（协议字母全大写，无歧义）
      char* argv[6];
      int argc = 0;
      char* tok = strtok(line, " \t\r\n");
      while (tok != nullptr && argc < 6) { argv[argc++] = tok; tok = strtok(nullptr, " \t\r\n"); }
      if (argc > 0) {
        for (char* p = argv[0]; *p != 0; ++p) *p = (char)tolower(*p);  // 与 shell 同款小写化
        fcCommands(argc, argv);
      }
      return;
    }
    bridge_.handleLine(line);  // 大写/数字行：SimpleFOC 协议原路进 Commander
  }
  StudioBridge bridge_;
  char buf_[48] = {0};
  int bufLen_ = 0;
};

ForceSession forceSession;

void setup() {
  Serial.setTxBufferSize(512);  // 突发打印入环形缓冲（须在 begin 前调用，同 01 号）
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
  if (!motor.init()) {  // 标定与 01 号共用 NVS，已固化则注入免标定
    Serial.println(F("[FW BOOT] motor.init failed; halted"));
    while (true) delay(1000);
  }
  Serial.println(F("[FW BOOT] motor.init returned ok"));

  applyCurrentLoopGains();  // 力矩通道地基：冻结电流环（阻抗带宽 ≤ωc/5 的分离比前提）
  computePredictions();     // 计算优先：预测量开机即算即打印

  motor.enable();
  motor.setMode(ControlMode::Torque);  // MT2+MC0 力矩模式打底（全模式共用，不再切走）
  motor.setTorqueTarget(0.0f);         // raw+0 安全启动

  forceSession.begin(&motor);  // 会话桥自带 StudioBridge（小写力控命令在会话期放行）
  Serial.println(F("[FW BOOT] forceSession.begin returned"));
  shell.begin(&motor);
  Serial.println(F("[FW BOOT] shell.begin returned"));
  shell.attachStudio(&forceSession);
  shell.attachUserCommand(fcCommands);
  Serial.println(F("[FW BOOT] shell attaches done"));
  shell.attachVerboseHook(applyVerbose);
  Serial.println(F("[FW BOOT] shell.attachVerboseHook done"));
  Serial.println(F("P2 力控主程序（02 号·第一轮）就绪。命令：mode raw|pend|imp / gv|th0|kk|kd|tp|tq <值> / pred / probe <ms>|off / stream / studio（help 查全部）"));
  Serial.println(F("[FW BOOT] boot 默认 raw+零力矩；[FC DBG] 探针 50ms 已开（时基锚点）"));
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

  // ---- 力律（唯一权威：每拍重算重发，覆盖一切旁路写入） ----
  MotorState st = motor.getState();
  float tau = computeLaw(st.angle, st.velocity);
  if (tau > T_LIM) tau = T_LIM;          // 双保险之一（适配层另有 DEFAULT_TORQUE_LIMIT）
  if (tau < -T_LIM) tau = -T_LIM;
  motor.setTorqueTarget(tau);

  motor.update();
  shell.update();  // studio 会话期自动转上位机通道，力律照常运行（同 01 号 step 激励）

  // ---- [FC DBG] 探针：θ/θ̇/Iq/τ_cmd（判据量化与时基标定的数据源，REF-14 §4） ----
  uint32_t now = millis();
  if (probeOn && now - lastProbeMs >= probePeriodMs) {
    lastProbeMs = now;
    Serial.printf("[FC DBG] ms=%lu mode=%s th=%.3f sv=%.3f iq=%.4f tc=%.4f\n",
                  (unsigned long)now, fcModeName(fcMode),
                  (double)st.angle, (double)st.velocity, (double)st.iq, (double)tau);
  }

  // ---- 稳态心跳：1 行/秒（同 01 号节流；'[' 开头 Studio 忽略） ----
  if (now - lastLoopLogMs >= 1000) {
    lastLoopLogMs = now;
    Serial.printf("[FW LOOP] alive vin=%.2fV ok=%d law=%s tc=%.4f\n",
                  (double)power.voltage(), power.ok() ? 1 : 0,
                  fcModeName(fcMode), (double)tau);
  }
}
