// FocKit 03 号示例 —— 无感 FOC 观测器矩阵对账台（第一轮，P4 前家庭平台无感战役）
//
// 硬件：DengFOC V4 + 2208 云台电机(7对极) + AS5600（M0 编码器口）+ 板载电流采样（M0: 39/36）
// 供电：DC 12V（≥11.1V 欠压失使能）；标定沿用 NVS（与 01/02 号共用）。
//
// ============================================================
// 战役定位与三条设计约束（2026-09-29 与用户确认）：
//   ① 底层=三环 FOC 定档冻结（TST-01/02/03 增益原样沿用，观测器为唯一自变量——
//      闭环任何退化可唯一归因于观测器误差，对账的控制变量法）；
//   ② AS5600 降役为真值裁判：控制链吃观测器角度（真无感），探针同时记录
//      编码器真值/控制链角/观测器估计三列，dth 误差离线对账；
//   ③ 观测器矩阵（本轮 SMO+PLL × 磁链；EKF 框架位已留，第二轮进矩阵）。
//
// —— 底座增益（计算优先，与 01 号同源，全部可溯源）——
//   电流环 ωc=125（kp=0.53/ki=1031，TST-01 定档）
//   速度环 ωv=20·N=10（kp=0.0025/ki=0.0254，TST-02 定档；输出限幅 0.5A）
//   位置环 ωp=4（P-only，TST-03 定档）
//   —— 本文件三环增益一字不改地抄 01 号：无感实验唯一自变量=角度源。
//
// —— 无感层设计 ——
//   角度源三态（src 命令）：
//     enc  AS5600 有感（黄金对照——同增益跑 TST-02/03 阶跃即无感的对照组）
//     vf   V-F 开环启动（电压矢量随自积分电角旋转，观测器并行收敛）
//     obs  观测器（真无感；VF 期间估计的零位偏差 off 已吸收进角度换算）
//   观测器两实现（obs 命令切换，控制拍常跑——即使 src=enc 也在后台跑供对账）：
//     smo  滑模观测器+PLL（边界层饱和替代 sign 抗抖振；LPF 提取反电动势；
//          PLL 锁相提角——PLL 带宽=本役整定参数，带宽法：≥ωv、≪ωc）
//     flux 磁链观测器（电压积分−电阻压降，一阶漂移补偿；隐极电机 ψ=λ−L·i 直接取角）
//   挂接机制：库 attachExternalSensor/detachExternalSensor（纯新增路径，
//   01 号冻结纪律不破坏：不挂接时库行为逐行等价）。
//
// —— 物理预算（[SL PRED] 开机打印，判据对账用）——
//   BEMF 幅值 = KT·ω_mech（KE_e·ω_e 恒等）；滑模增益 k > max|E|≈KT·ω_max
//   低速盲区 ω_min ≈ R·Δi_纹波/KT ≈ 8.25×0.01/0.032 ≈ 2.6 rad/s（电流纹波 2%=10mA，
//   TST-01 口径）——实测预计 5~10 rad/s，与 EXP-05 平滑边界可能同域汇合（留痕点）
//   VF 启动电压 U = I·R + KT·ω（I_VF=0.15A≈30% 持续红线）
//
// 使用：
//   vf <rads>        V-F 启动至目标机械速度（自动 src vf；观测器并行收敛）
//                    —— 无感交付域=中高速（盲区预算 ω_min≈2.6、实测预计 5~10 rad/s，
//                       行业通病不补丁：切无感建议 ≥10，低速域留作盲区判据实验）
//   src enc|vf|obs   切角度源（vf 转起来观测器锁定后 src obs=真无感闭环；
//                    VF→obs 无扰切换：offEst 吸收瞬时帧差，控制角连续不跳变，
//                    loop v 无感路径目标=当前速度起步——均防切换/入环反刹）
//   obs smo|flux|off 切观测器
//   vf off           VF 减速停止（回 Idle）
//   loop v|p|t|off   三环会话（同 01 号；速度/位置阶跃判据=有感黄金对照复跑）
//   step on|off|amp <x>|period <ms>   方波激励（同 01 号）
//   sk <V> / st <ms>           SMO 滑模增益 / 抖振 LPF
//   sp <kp> <ki>               PLL 锁相增益（带宽整定主参数）
//   sd <s>                     磁链漂移补偿时间常数
//   gains / probe <ms>|off     计算链重打印 / 探针周期（判据实验 10ms）
//   stream / studio / dbg / help —— 沿 01 号全套
//
// 探针（[SL DBG]，对账数据源）：
//   the=编码器真值(机械 rad) thc=控制链角 tho=估计角——thc/tho 显示值=the−各自真误差
//   （三线对齐真值零点：线间间隙=真误差；enc 源 err=0 / vf 源=滑差 / obs 源=dth）
//   dth=电域对账误差÷PP(机械域)
//   we=编码器速度 wo=观测器速度 vfW=VF 当前机械速度（到速判据源） iq=电流 uq=指令电压
//
// 纪律：
//   ① 三环增益冻结不调（要调回 01 号调完抄回）；
//   ② vf 启动期间 off 估计持续进行；src obs 后 off 冻结（不再吃 VF 信息）；
//   ③ 切换角度源时电机应在转动中（静止切 obs 无 BEMF 会失锁——物理边界）。
// ============================================================

#include <FocKit.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

using namespace fockit;

SimpleFocMotor motor(defaultM0Config());
SerialShell shell;
PowerMonitor power;

// ========== 底座：三环定档（01 号原样抄回，冻结） ==========
constexpr float L_PH  = dengfoc_v4::MOTOR_2208.phaseInductance;  // 4.25 mH  ①标称
constexpr float R_PH  = dengfoc_v4::MOTOR_2208.phaseResistance;  // 8.25 Ω   ①标称
constexpr float KT_M  = dengfoc_v4::MOTOR_2208.kt;               // 0.032    ②实测
constexpr float J_EST = 4.06e-6f;   // kg·m² ②实测（TST-02；P3 辨识夹逼 3.4~5.1e-6 居中一致）
constexpr float WC    = 125.0f;     // 电流环带宽（TST-01 定档）
constexpr float WV    = 20.0f;      // 速度环带宽（TST-02 定档）
constexpr float WP    = 4.0f;       // 位置环带宽（TST-03 定档）
constexpr float PP    = 7.0f;       // 极对数（2208）
constexpr float I_VF  = 0.15f;      // [A] VF 启动电流（≈30% 持续红线）
constexpr float VF_ACCEL = 8.0f;    // [rad/s²] VF 速度爬升加速度
constexpr float T_OFF  = 0.2f;      // [s] VF 期间零位偏差 off 的估计时间常数

void applyGains() {
  motor.setLoopGains(LoopType::CurrentQ, LoopGains(L_PH * WC, R_PH * WC, 0.0f, 0.002f));
  motor.setLoopGains(LoopType::CurrentD, LoopGains(L_PH * WC, R_PH * WC, 0.0f, 0.002f));
  // 速度环输出限幅域修正：0.5A 持续红线换算电压域 0.5·R≈4.13V——电压力矩模式下
  // 速度 PID 输出直接落 voltage.q，旧值 0.5（01 号按 foc_current 的 Iq 域设计）被
  // 错套成 0.5V=仅 60mA，ω≳8.4 rad/s 即无力维持（30 rad/s 巡航需 ≥1.19V；
  // 185818 案死亡段 uq 钉死 0.500V/iq 钉死 0.0606A 的来源）
  motor.setLoopGains(LoopType::Velocity,
                     LoopGains(WV * J_EST / KT_M, 10.0f * WV * J_EST / KT_M, 0.0f, 0.01f,
                               0.5f * R_PH));
  motor.setLoopGains(LoopType::Position, LoopGains(WP));
}

// ========== 无感层 ==========

static inline float wrapPi(float x) {
  while (x > (float)M_PI) x -= 2.0f * (float)M_PI;
  while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
  return x;
}
static inline float sat01(float x) { return constrain(x, -1.0f, 1.0f); }

// ---- SMO+PLL：滑模电流观测器出反电动势，PLL 锁相提角 ----
struct SmoObs {
  float k = 1.5f;          // [V] 滑模增益（须 > max|E|≈KT·ω_max；0.032×30≈1.0）
  float tf = 0.002f;       // [s] 抖振/EMF 提取 LPF
  float pllpKp = 100.0f;   // PLL 比例（≈锁相带宽 [1/s]；带宽法：≥ωv=20、≪ωc=125）
  float pllpKi = 2000.0f;  // PLL 积分
  float ia = 0, ib = 0;    // 估计电流 i_αβ
  float ea = 0, eb = 0;    // 估计反电动势 e_αβ（LPF 后）
  float th = 0;            // 电角估计 θ̂
  float wInt = 0, w = 0;   // PLL 积分项 / 电角速度 ω̂_e
  bool locked = false;     // BEMF 幅值过阈 → 视为锁定（盲区指示）
  void step(float c_a, float c_b, float v_a, float v_b, float dt) {
    if (dt <= 0.0f || dt > 0.01f) return;  // 时基纪律：首拍/长卡顿不推
    float za = k * sat01((c_a - ia) / 0.3f);   // 边界层饱和（ε=0.3A）替代 sign 抗抖振
    float zb = k * sat01((c_b - ib) / 0.3f);
    ia += dt / L_PH * (v_a - R_PH * ia + za);  // î̇=(v+ z−Rî)/L；滑模面 δ→0 时 z→−e
    ib += dt / L_PH * (v_b - R_PH * ib + zb);
    float a = dt / tf;
    if (a > 1.0f) a = 1.0f;
    ea += a * (-za - ea);                      // ê=−z 经一阶 LPF
    eb += a * (-zb - eb);
    float e = -ea * cosf(th) - eb * sinf(th);  // 锁相误差 = |E|·sin(θ−θ̂)
    wInt += pllpKi * e * dt;
    w = wInt + pllpKp * e;
    th = wrapPi(th + w * dt);
    float emf = sqrtf(ea * ea + eb * eb);
    locked = emf > 0.08f;                      // [V] BEMF 阈值≈ω>2.5 rad/s（盲区边界）
  }
};

// ---- 磁链观测器：λ̇=v−Ri（一阶漂移补偿），ψ=λ−Li，θ=atan2 ----
struct FluxObs {
  float td = 0.5f;          // [s] 积分漂移补偿时间常数
  float la = 0, lb = 0;     // 磁链积分 λ_αβ
  float th = 0, w = 0;      // 电角 / 电角速度（差分+LPF）
  bool locked = false;
  void step(float c_a, float c_b, float v_a, float v_b, float dt) {
    if (dt <= 0.0f || dt > 0.01f) return;
    la += (v_a - R_PH * c_a) * dt - la * dt / td;  // 积分 + 高通式漂移抑制
    lb += (v_b - R_PH * c_b) * dt - lb * dt / td;
    float pa = la - L_PH * c_a, pb = lb - L_PH * c_b;  // 永磁磁链（隐极）
    float thNew = atan2f(pb, pa);
    float dw = wrapPi(thNew - th) / dt;
    w += 0.05f * (dw - w);                         // ω 差分一阶 LPF（tf≈... 收敛系数）
    th = wrapPi(thNew);
    float pm = sqrtf(pa * pa + pb * pb);
    locked = pm > 2.0e-3f;                         // [Wb] 磁链幅值阈值
  }
};

// ---- 观测器矩阵（obs 命令切换；控制拍常跑供对账） ----
enum class ObsType : uint8_t { Off, Smo, Flux };
SmoObs smo;
FluxObs flux;
ObsType obsType = ObsType::Off;
const char* obsName() {
  switch (obsType) { case ObsType::Smo: return "smo"; case ObsType::Flux: return "flux"; default: return "off"; }
}
float obsTh()  { return obsType == ObsType::Smo ? smo.th  : (obsType == ObsType::Flux ? flux.th  : 0.0f); }
float obsWe()  { return obsType == ObsType::Smo ? smo.w   : (obsType == ObsType::Flux ? flux.w   : 0.0f); }  // 电角速度
bool obsLocked() { return obsType == ObsType::Smo ? smo.locked : (obsType == ObsType::Flux ? flux.locked : false); }

float offEst = 0.0f;       // 零位偏差（VF 期间估计：off = θ_elec_true − θ̂；src obs 后冻结）
float zeroElecCached = 0;  // attach 时快照的 AS5600 标定电角零位

// ---- VF 传感器：电角自积分的开环角度源（Sensor 语义=机械单圈角 0~2π） ----
struct VfSensor : public Sensor {
  float thE = 0;            // VF 积分电角（±π wrap）
  float wMech = 0;          // 当前 VF 机械速度
  float wTarget = 0;        // 目标机械速度
  bool running = false;
  float uq = 0;             // VF 电压 U = I·R + KT·ω
  void update(float dt) {
    if (dt <= 0.0f || dt > 0.01f) return;
    float dw = wTarget - wMech;
    float step = VF_ACCEL * dt;
    wMech += constrain(dw, -step, step);
    thE = wrapPi(thE + PP * wMech * dt);
    uq = I_VF * R_PH + KT_M * wMech;
    if (!running && fabsf(wMech) < 0.01f) uq = 0;   // 完全停止后撤压
  }
  float getSensorAngle() override {
    float mech = (thE + 0.0f) / PP;   // elec=thE+zeroElec → shaft=thE/PP（零位已在电角域处理）
    mech = fmodf(mech, 2.0f * (float)M_PI);
    if (mech < 0) mech += 2.0f * (float)M_PI;
    return mech;
  }
};

// ---- 观测器传感器：θ̂+off → 机械角（SimpleFOC elec=shaft·PP+zeroElec 反解） ----
// 连续角纪律（194634 案）：观测器电角是 ±π 回卷的"罗盘"，直接 ÷PP 得 0.898 宽
// 锯齿窗——Sensor 基类整圈检测（阈值 0.8·2π）看不见 2π/PP 的回跳，full_rotations
// 永不累加 → getVelocity() 差分出锯齿残渣（真值 +30 读成 -22.85，速度环追噪声
// =高速振动；力矩模式不受影响：0.898 机械回跳×PP=2π≡电角不变，故无感拖动正常）。
// 修法=内部累积器（罗盘换里程表）：首拍旧公式定相位（无扰交接连续保持；电角逐拍
// 恒等于 th+offEst，力矩模式行为不变），此后只累加 wrapPi(Δ电角)/PP。
struct ObsSensor : public Sensor {
  bool primed = false;
  float prevTh = 0;       // 上拍观测器电角
  double mechCont = 0;    // 连续机械角（里程表；double 抗长跑精度损失）
  float getSensorAngle() override {
    float th = obsTh();
    if (!primed) {
      primed = true;
      prevTh = th;
      mechCont = (double)((th + offEst - zeroElecCached) / PP);
    } else {
      mechCont += (double)(wrapPi(th - prevTh) / PP);
      prevTh = th;
    }
    float twoPi = 2.0f * (float)M_PI;
    float m = (float)fmod(mechCont, (double)twoPi);
    if (m < 0.0f) m += twoPi;
    return m;
  }
  void resetBookkeeping() { primed = false; }  // vf 复位块调用：重挂时重定相位
};
VfSensor vfSensor;
ObsSensor obsSensor;

// ---- 角度源状态 ----
enum class SrcType : uint8_t { Enc, Vf, Obs };
SrcType srcType = SrcType::Enc;
const char* srcName() {
  switch (srcType) { case SrcType::Vf: return "vf"; case SrcType::Obs: return "obs"; default: return "enc"; }
}

// ========== 阶跃激励（同 01 号：三环判据复用作有感黄金对照） ==========
float stepAmp = 3.0f;
uint32_t stepPeriodMs = 2000;
bool stepOn = false, stepHigh = false;
uint32_t lastStepMs = 0;

// ========== 探针 ==========
uint32_t probePeriodMs = 50;
bool probeOn = true;
uint32_t lastProbeMs = 0;

void computePredictions() {
  float emfMax = KT_M * 30.0f;
  Serial.printf("[SL PRED] 底座三环冻结: ωc=%.0f ωv=%.0f ωp=%.0f（TST-01/02/03 定档，观测器=唯一自变量）\n",
                (double)WC, (double)WV, (double)WP);
  Serial.printf("[SL PRED] SMO: k=%.2fV(>max|E|=%.2f) tf=%.1fms PLL kp=%.0f ki=%.0f（带宽法: ≥ωv、≪ωc）\n",
                (double)smo.k, (double)emfMax, (double)(smo.tf * 1000.0f),
                (double)smo.pllpKp, (double)smo.pllpKi);
  Serial.printf("[SL PRED] flux: Td=%.2fs；VF: I=%.2fA U=IR+KTω 加速度=%.0f rad/s²\n",
                (double)flux.td, (double)I_VF, (double)VF_ACCEL);
  Serial.printf("[SL PRED] 低速盲区预算 ω_min≈R·Δi/KT=%.1f rad/s（纹波 2%%×0.5A=10mA，TST-01 口径；"
                "实测预计 5~10——与 EXP-05 平滑边界可能同域，留痕对账点）\n",
                (double)(R_PH * 0.01f / KT_M));
}

void printParams() {
  Serial.printf("[SL CFG] src=%s obs=%s vf=%s vfT=%.1f sk=%.2f st=%.1fms sp=%.0f/%.0f sd=%.2f probe=%lu%s\n",
                srcName(), obsName(), vfSensor.running ? "on" : "off",
                (double)vfSensor.wTarget, (double)smo.k, (double)(smo.tf * 1000.0f),
                (double)smo.pllpKp, (double)smo.pllpKi, (double)flux.td,
                (unsigned long)probePeriodMs, probeOn ? "ms" : "off");
}

// ========== 无感命令（小写，经 SensorlessSession 会话期放行——同 02 号 ForceSession 先例） ==========
bool slCommands(int argc, char* argv[]) {
  if (argc < 1) return false;

  // 裸数字=目标值兜底：Studio 桥路径 Commander 认裸数字，但 shell 当班时无人认领
  // （实测 "15" 被拒→速度环目标 0→交接即自刹停进盲区）——两条派发路径必须等价
  char c0 = argv[0][0];
  if ((c0 >= '0' && c0 <= '9') || c0 == '-' || c0 == '+') {
    motor.setTarget(strtof(argv[0], nullptr));
    Serial.printf("[SL CFG] target=%.3g\n", (double)motor.getTarget());
    return true;
  }

  // 角度源切换
  if (!strcmp(argv[0], "src") && argc >= 2) {
    if (argv[1][0] == 'e') {
      motor.detachExternalSensor();
      srcType = SrcType::Enc;
    } else if (argv[1][0] == 'v') {
      Serial.println(F("[SL] 先 vf <rads> 启动（src vf 由 vf 命令自动切入）"));
      return true;
    } else if (argv[1][0] == 'o') {
      if (obsType == ObsType::Off) {
        Serial.println(F("[SL] src obs 拒绝：先 obs smo|flux 选观测器"));
        return true;
      }
      if (!obsLocked()) {
        Serial.println(F("[SL] 警告：观测器未锁定（BEMF 低于阈值/盲区）——静止切入会失锁，"
                         "建议 vf 转起来后再切"));
      }
      if (srcType == SrcType::Vf) {
        // 无扰切换（bumpless）：切换拍把 VF 帧与观测器帧的瞬时差吸进 offEst，
        // 控制角连续 → 电压矢量方向不跳变（实测 135° 电角跳变=全电压反打刹停的根治）。
        // 不清力矩：清了会开摩擦滑停窗（30 rad/s 约 135ms 停死）；方向连续后无乱打可防
        float snap = wrapPi((vfSensor.thE + zeroElecCached) - (obsTh() + offEst));
        offEst = wrapPi(offEst + snap);
        Serial.printf("[SL ID] 无扰切换：offEst 吸收 %.3f rad，控制角连续\n", (double)snap);
      }
      motor.attachExternalSensor(&obsSensor);
      srcType = SrcType::Obs;
    } else return false;
    Serial.printf("[SL CFG] src=%s%s\n", srcName(),
                  srcType == SrcType::Obs ? "（真无感：控制链吃观测器）" :
                  srcType == SrcType::Enc ? "（有感对照）" : "");
    return true;
  }

  // VF 启动/停止
  if (!strcmp(argv[0], "vf")) {
    if (argc >= 2 && !strcmp(argv[1], "off")) {
      vfSensor.wTarget = 0;                      // 减速停（VF 传感器继续跑到 0）
      Serial.println(F("[SL ID] vf 减速停止中（到 0 后可 src enc 收尾）"));
      printParams();
      return true;
    }
    if (argc >= 3 && !strcmp(argv[1], "on")) {   // vf on <rads>：兼容两段式
      vfSensor.wTarget = fabsf(strtof(argv[2], nullptr));
    } else if (argc >= 2) {                      // vf <rads>
      float v = strtof(argv[1], nullptr);
      if (v <= 0.1f || v > 40.0f) {
        Serial.println(F("[SL] 目标速度范围 0.1~40 rad/s（盲区预算 2.6，起步建议 ≥10）"));
        return true;
      }
      vfSensor.wTarget = v;
    } else {
      Serial.println(F("用法：vf <rads> | vf on <rads> | vf off"));
      return true;
    }
    // 状态卫生：每次 VF 启动清观测器积分器与零位估计——上次运行/上轮假锁的
    // 残留（如 PLL 积分跑飞值 wInt）会污染新一轮收敛，重试必须同权从零开始
    smo.ia = 0; smo.ib = 0; smo.ea = 0; smo.eb = 0;
    smo.wInt = 0; smo.w = 0; smo.th = 0; smo.locked = false;
    flux.la = 0; flux.lb = 0; flux.th = 0; flux.w = 0; flux.locked = false;
    offEst = 0;
    obsSensor.resetBookkeeping();  // 连续角里程表重置（重试同权，重挂时重定相位）
    if (obsType == ObsType::Off) {
      obsType = ObsType::Smo;                    // 默认观测器
      Serial.println(F("[SL CFG] obs=smo (vf 自动选默认观测器)"));
    }
    motor.setMode(ControlMode::Torque);          // VF=电压型开环：角度源已锁 VF 电角
    motor.attachExternalSensor(&vfSensor);
    vfSensor.running = true;
    srcType = SrcType::Vf;
    Serial.printf("[SL ID] vf 启动: 目标 %.1f rad/s，加速度 %.0f，观测器并行收敛中\n",
                  (double)vfSensor.wTarget, (double)VF_ACCEL);
    Serial.println(F("[SL ID] 观测器锁定后 src obs 切真无感（off 已自动估计吸收零位差）"));
    printParams();
    return true;
  }

  // 观测器切换
  if (!strcmp(argv[0], "obs") && argc >= 2) {
    if (!strcmp(argv[1], "smo"))       obsType = ObsType::Smo;
    else if (!strcmp(argv[1], "flux")) obsType = ObsType::Flux;
    else if (!strcmp(argv[1], "off"))  obsType = ObsType::Off;
    else return false;
    if (obsType == ObsType::Off && srcType == SrcType::Obs) {
      motor.detachExternalSensor();   // 观测器关闭时若在无感源上，安全切回编码器
      srcType = SrcType::Enc;
      Serial.println(F("[SL CFG] src=enc (obs off 安全切回)"));
    }
    Serial.printf("[SL CFG] obs=%s\n", obsName());
    return true;
  }

  // 观测器参数
  if (argc >= 2) {
    float v = strtof(argv[1], nullptr);
    bool hit = true;
    if      (!strcmp(argv[0], "sk")) smo.k = v;
    else if (!strcmp(argv[0], "st")) smo.tf = v * 0.001f;
    else if (!strcmp(argv[0], "sd")) flux.td = v;
    else if (!strcmp(argv[0], "sp") && argc >= 4) {
      smo.pllpKp = strtof(argv[1], nullptr);
      smo.pllpKi = strtof(argv[2], nullptr);
    }
    else hit = false;
    if (hit) { printParams(); return true; }
  }

  // 三环会话（同 01 号 + off；src=vf 时切三环会话会先撤 VF 源——VF 是启动器不是控制态）
  if (!strcmp(argv[0], "loop") && argc >= 2) {
    stepOn = false;
    if (argv[1][0] == 'v') {
      if (srcType == SrcType::Vf) { motor.detachExternalSensor(); srcType = SrcType::Enc; }
      motor.setMode(ControlMode::Velocity);
      if (srcType == SrcType::Obs) {
        // 无扰闭环进入（bumpless 全套）：目标=当前速度 + PID 积分预置=当前输出电压，
        // 入环第一拍力矩连续不断档。旧公共前缀 setTarget(0) 在力矩域执行=拖动力矩瞬间
        // 归零→摩擦 ~135ms 滑停→BEMF 消失→观测器幻觉→环被幻觉喂瞎（185818 案：电机
        // 原地发抖、目标失效）。有感对照路径保持 0 起步阶跃语义
        motor.preloadVelocityIntegral(motor.readUq());
        motor.setVelocityTarget(motor.getVelocity());
        Serial.printf("[SL ID] 闭环无扰进入：目标=当前速度 %.2f rad/s，积分预置 %.3fV\n",
                      (double)motor.getVelocity(), (double)motor.readUq());
      } else {
        motor.setTarget(0);   // 有感对照：0 起步阶跃语义
      }
      stepAmp = 3.0f; stepPeriodMs = 2000;
      Serial.println(F("[SL] 速度环会话（同增益有感/无感对照：enc 与 obs 各跑一遍 step）"));
    } else if (argv[1][0] == 'p') {
      if (srcType == SrcType::Vf) { motor.detachExternalSensor(); srcType = SrcType::Enc; }
      motor.setTarget(0);
      motor.setMode(ControlMode::Position);
      stepAmp = 1.5708f; stepPeriodMs = 4000;
      Serial.println(F("[SL] 位置环会话"));
    } else if (argv[1][0] == 't') {
      motor.setTarget(0);
      motor.setMode(ControlMode::Torque);
      stepAmp = 0.01f; stepPeriodMs = 3000;
      Serial.println(F("[SL] 电压力矩会话"));
    } else if (argv[1][0] == 'o') {
      motor.setTarget(0);
      motor.setMode(ControlMode::Idle);
      Serial.println(F("[SL] Idle"));
    } else return false;
    return true;
  }

  if (!strcmp(argv[0], "step")) {
    if (argc >= 2 && !strcmp(argv[1], "on")) { stepOn = true; lastStepMs = 0; }
    else if (argc >= 2 && !strcmp(argv[1], "off")) { stepOn = false; motor.setTarget(0); }
    else if (argc >= 3 && !strcmp(argv[1], "amp")) stepAmp = strtof(argv[2], nullptr);
    else if (argc >= 3 && !strcmp(argv[1], "period")) stepPeriodMs = (uint32_t)atol(argv[2]);
    else return false;
    Serial.printf("[SL CFG] step %s amp=%.3f period=%lu ms\n",
                  stepOn ? "on" : "off", (double)stepAmp, (unsigned long)stepPeriodMs);
    return true;
  }

  if (!strcmp(argv[0], "gains")) { computePredictions(); return true; }

  if (!strcmp(argv[0], "probe")) {
    if (argc >= 2 && !strcmp(argv[1], "off")) probeOn = false;
    else if (argc >= 2) {
      long p = atol(argv[1]);
      probeOn = p >= 10;
      if (probeOn) probePeriodMs = (uint32_t)p;
    }
    Serial.printf("[SL CFG] probe=%lu%s\n", (unsigned long)probePeriodMs, probeOn ? "ms" : "off");
    return true;
  }
  return false;
}

void applyVerbose(bool on) { power.setPeriodicVerbose(on); }

// ========== 无感会话桥（02 号 ForceSession 同款：小写命令在 Studio 会话期放行） ==========
class SensorlessSession : public ISerialSession {
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
    if (line[0] >= 'a' && line[0] <= 'z') {
      char* argv[6];
      int argc = 0;
      char* tok = strtok(line, " \t\r\n");
      while (tok != nullptr && argc < 6) { argv[argc++] = tok; tok = strtok(nullptr, " \t\r\n"); }
      if (argc > 0) {
        for (char* p = argv[0]; *p != 0; ++p) *p = (char)tolower(*p);
        slCommands(argc, argv);
      }
      return;
    }
    bridge_.handleLine(line);
  }
  StudioBridge bridge_;
  char buf_[48] = {0};
  int bufLen_ = 0;
};
SensorlessSession slSession;

void setup() {
  Serial.setTxBufferSize(512);
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
  if (!motor.init()) {
    Serial.println(F("[FW BOOT] motor.init failed; halted"));
    while (true) delay(1000);
  }
  Serial.println(F("[FW BOOT] motor.init returned ok"));

  applyGains();            // 三环定档冻结注入（TST-01/02/03）
  zeroElecCached = motor.readZeroElectricAngle();
  Serial.printf("[FW BOOT] zero_electric_angle 快照=%.3f rad（obs 角度换算用）\n",
                (double)zeroElecCached);
  computePredictions();    // 计算优先：预算即打印
  printParams();

  motor.enable();
  motor.setMode(ControlMode::Idle);

  slSession.begin(&motor);
  Serial.println(F("[FW BOOT] slSession.begin returned"));
  shell.begin(&motor);
  Serial.println(F("[FW BOOT] shell.begin returned"));
  shell.attachStudio(&slSession);
  shell.attachUserCommand(slCommands);
  Serial.println(F("[FW BOOT] shell attaches done"));
  shell.attachVerboseHook(applyVerbose);
  Serial.println(F("[FW BOOT] shell.attachVerboseHook done"));
  Serial.println(F("无感观测器对账台（03 号·第一轮：SMO+PLL×磁链，EKF 框架位已留）就绪。"
                   "命令：vf <rads> / src enc|obs / obs smo|flux|off / loop v|p|t / step / sk|st|sp|sd / gains / probe（help 查全部）"));
  Serial.println(F("[FW BOOT] 推荐流程：obs smo → vf 10 → 等 [SL DBG] tho 跟上 → src obs → loop v → step on（同增益对照判据）"));
  Serial.println(F("[FW BOOT] setup complete"));
}

void loop() {
  static uint32_t lastLoopLogMs = 0;
  static bool lastPowerOk = false;
  static uint32_t lastObsUs = 0;
  static bool powerDisabled = false;   // 欠压失能标记：恢复后回使能（对齐 PowerMonitor
                                       // "恢复后自动解除"设计；单向失能曾致无感台全静默瘫）
  power.update();
  bool powerOk = power.ok();
  if (powerOk != lastPowerOk) {
    lastPowerOk = powerOk;
    Serial.printf("[FW LOOP] power.ok=%s\n", powerOk ? "true" : "false");
  }
  if (!powerOk) {
    motor.disable();
    powerDisabled = true;
  } else if (powerDisabled) {
    motor.enable();
    powerDisabled = false;
    Serial.println(F("[SL] 电压恢复，电机已回使能"));
  }

  // ---- 观测器步进（控制拍常跑：i_αβ/v_αβ 从库只读访问器取） ----
  uint32_t nowUs = micros();
  float dt = (float)(nowUs - lastObsUs) * 1e-6f;
  if (lastObsUs != 0) {
    PhaseCurrent_s pc = motor.readPhaseCurrents();
    // Clarke（三相→αβ； InlineCurrentSense 给 a/b/c，offset 已在库内处理）
    float i_a = pc.a, i_b = pc.b, i_c = pc.c;
    float ial = (2.0f / 3.0f) * (i_a - 0.5f * (i_b + i_c));
    float ibe = (1.0f / sqrtf(3.0f)) * (i_b - i_c);
    float uq = motor.readUq(), ud = motor.readUd();
    // 指令电压反 Park 到 αβ（用观测器自身角度——自洽标准做法）：
    // v_α=ud·cosθ−uq·sinθ，v_β=ud·sinθ+uq·cosθ。对齐 SimpleFOC setPhaseVoltage
    // 语义（矢量施加在 θ+atan2(uq,ud)，ud=0 时=θ+90°）。旧版把 uq/ud 的三角
    // 角色对调 → 重建矢量落后真实 90° → 误差项≈|Uq|·√2 淹没真 BEMF，观测器
    // 反向跑飞元凶（2026-09-29 三轮破案定位，自检法：重建后正 Park 回去须还原 (ud,uq)）
    float cth = cosf(obsTh()), sth = sinf(obsTh());
    float v_a = ud * cth - uq * sth;
    float v_b = ud * sth + uq * cth;
    if (obsType == ObsType::Smo)       smo.step(ial, ibe, v_a, v_b, dt);
    else if (obsType == ObsType::Flux) flux.step(ial, ibe, v_a, v_b, dt);

    // VF 期间零位偏差估计：目标=控制角 (obsTh+offEst) 一阶收敛到 VF 帧 (thE+zeroElec)。
    // err 必须含 offEst（旧版漏减 → err 与 offEst 无关 → 开环积分速度失配，
    // offEst 随机游走到 ±3——30 rad/s 实测 off=-2.85 的来源）
    if (srcType == SrcType::Vf && obsType != ObsType::Off && obsLocked()) {
      float err = wrapPi((vfSensor.thE + zeroElecCached) - (obsTh() + offEst));
      offEst = wrapPi(offEst + dt / T_OFF * err);
    }
  }
  lastObsUs = nowUs;

  // ---- VF 推进（角度积分+电压指令） ----
  vfSensor.update(dt);
  if (srcType == SrcType::Vf && vfSensor.running) {
    // 电压型开环：τ=U·KT/R（适配层 V=R·τ/KT 反算）
    motor.setTorqueTarget(vfSensor.uq * KT_M / R_PH);
  }

  // ---- 阶跃激励（三环判据对照） ----
  if (stepOn && millis() - lastStepMs >= stepPeriodMs) {
    lastStepMs = millis();
    stepHigh = !stepHigh;
    motor.setTarget(stepHigh ? stepAmp : -stepAmp);
  }

  motor.update();
  shell.update();

  // ---- [SL DBG] 探针：对账三列（真值/控制链/估计） ----
  uint32_t now = millis();
  if (probeOn && now - lastProbeMs >= probePeriodMs) {
    lastProbeMs = now;
    float the = motor.readEncoderAngle();
    // dth 电域对账：估计电角 ±π 回卷使机械域差 wrapPi(the−tho) 成锯齿混叠伪影
    // （高速 ±2.7 乱摆）；电域差 wrapPi 后 ÷PP 才是真瞬时误差（T-SL ① 判据数据源）
    float dth = obsType != ObsType::Off
        ? wrapPi(the * PP + zeroElecCached - (obsTh() + offEst)) / PP : 0.0f;
    // 三线显示对齐真值零点：thc(案⑦后里程表)与旧公式 tho(回卷贴地)零点不同，波形上
    // 被读成"误差~1000"——实为里程表零点差恒偏置。显示角=the−各自真误差，线间间隙
    // =真误差；纯显示层，判据源 dth 不变
    float tho = the - dth;                                        // 估计线（obs=off 时与真值重合）
    float thcErr = srcType == SrcType::Enc ? 0.0f                // enc：控制链即真值
                 : srcType == SrcType::Vf  ? wrapPi(the * PP - vfSensor.thE) / PP  // vf：滑差
                 : dth;                                           // obs：控制链误差=估计误差
    float thc = the - thcErr;
    float we = motor.readEncoderVelocity();
    float wo = obsType != ObsType::Off ? obsWe() / PP : 0.0f;
    Serial.printf("[SL DBG] ms=%lu src=%s obs=%s vf=%s vfW=%.1f the=%.3f thc=%.3f tho=%.3f dth=%.3f "
                  "we=%.3f wo=%.3f iq=%.4f uq=%.3f lk=%d\n",
                  (unsigned long)now, srcName(), obsName(),
                  srcType == SrcType::Vf ? "on" : "off",
                  (double)vfSensor.wMech,
                  (double)the, (double)thc, (double)tho,
                  (double)dth,
                  (double)we, (double)wo,
                  (double)motor.getState().iq, (double)motor.readUq(),
                  obsLocked() ? 1 : 0);
  }

  // ---- 心跳 1 行/秒 ----
  if (now - lastLoopLogMs >= 1000) {
    lastLoopLogMs = now;
    Serial.printf("[SL LOOP] alive vin=%.2fV ok=%d src=%s obs=%s vfT=%.1f vfW=%.1f off=%.3f\n",
                  (double)power.voltage(), powerOk ? 1 : 0, srcName(), obsName(),
                  (double)vfSensor.wTarget, (double)vfSensor.wMech, (double)offEst);
  }
}
