#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""P3 单电机辨识离线拟合工具（2026-09-28 立项交付；固件 = 02 号 id 模式 + chirp 激励，SPEC-P §4）。

要估的模型（对参数线性）：
    J·θ̈ = τ − τf·sign(θ̇) − b·θ̇        （τ 取探针 tc 列=力律输出，即指令力矩）

θ̈ 双差分噪声是本硬件命门（REF-14 砍惯量整形的同源问题）——辨识靠**离线批量**绕开它，
两套估计器互为交叉验证：

- 法A 冲量法（主，全程零微分）：在 θ̇ 不过零的每个半周期段上积分运动方程
    J·Δθ̇ = ∫τ·dt − τf·s·Δt − b·Δθ       （s = 该段 θ̇ 符号）
  全部是积分量（低频抗噪），每段一个方程 → 最小二乘解 [J, τf, b]。
- 法B 微分法（对照）：θ̈ = d(θ̇)/dt 只做一次差分（sv 已是固件 LPF 后值），逐样本回归；
  掩码剔除 |θ̇| < v_eps 的黏滞段（该处物理是不等式不是方程）。

置信区间：线性回归 σ²·(XᵀX)⁻¹ 对角线，±1.96σ（大样本 95% 近似）。
先验对账：J vs 4.06e-6、τf vs 0.9e-3（TST-02 / EXP-05 实测定标）；b 为首测仅记录。

用法：
  python identFit.py --selftest         # 合成数据自检：已知真值能否收回（上机前先跑这个）
GUI：力控台波形区"辨识拟合"按钮吃当前缓冲；"拟合CSV…"按钮经 QFileDialog 选导出文件离线拟合。
（CLI 不直接收文件路径——路径安全收口，Mimosa：open 的来源限定 QFileDialog，同设备页 JSON 加载模式）
"""
import csv
import math
import sys

import numpy as np

PRIOR_J = 4.06e-6      # [kg·m²]  TST-02 定标
PRIOR_TAUF = 0.9e-3    # [N·m]    EXP-05 定标

try:
    _trapz = np.trapezoid          # numpy ≥2.0
except AttributeError:
    _trapz = np.trapz              # numpy <2.0（弃用警告无害）


def _lstsq_ci(X, y):
    """最小二乘 + 95% 置信半宽。返回 (params, err95, dof, sigma2)。"""
    theta, *_ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ theta
    n, p = X.shape
    dof = max(n - p, 1)
    sigma2 = float(resid @ resid) / dof
    cov = sigma2 * np.linalg.pinv(X.T @ X)
    err95 = 1.96 * np.sqrt(np.maximum(np.diag(cov), 0.0))
    return theta, err95, dof, sigma2


def half_cycles(v, v_hys=0.05, min_pts=4):
    """按 θ̇ 符号切半周期段（滞回阈值防零附近抖动；|θ̇|<v_hys 的黏滞带作段间隔剔除）。

    返回 [(i0, i1, s)]：段内样本下标区间与速度符号。
    """
    segs = []
    cur, i0 = 0, 0
    for i in range(len(v)):
        if cur == 0:
            if abs(v[i]) > v_hys:
                cur, i0 = (1 if v[i] > 0 else -1), i
        else:
            if v[i] * cur < -v_hys:            # 反向越过阈值 → 本段结束
                if i - i0 >= min_pts:
                    segs.append((i0, i, cur))
                cur = 0
    if cur != 0 and len(v) - i0 >= min_pts:
        segs.append((i0, len(v), cur))
    return segs


def fit_impulse(t, th, v, tau, v_hys=0.05):
    """法A 冲量法：每半周期一个方程 J·Δθ̇ + τf·s·Δt + b·Δθ = ∫τ·dt。"""
    rows_X, rows_y = [], []
    for i0, i1, s in half_cycles(v, v_hys=v_hys):
        tt, vv, ths, ts = t[i0:i1], v[i0:i1], th[i0:i1], tau[i0:i1]
        rows_X.append([vv[-1] - vv[0], s * (tt[-1] - tt[0]), ths[-1] - ths[0]])
        rows_y.append(_trapz(ts, tt))
    if len(rows_X) < 6:
        raise ValueError('法A 半周期段不足（%d 段<6）——数据太短或幅值没破摩死区，'
                         '先加大激励幅值/时长' % len(rows_X))
    return _lstsq_ci(np.asarray(rows_X), np.asarray(rows_y))


def fit_diff(t, th, v, tau, v_eps=0.05):
    """法B 微分法：θ̈=d(θ̇)/dt 一次差分，逐样本回归（剔除黏滞带）。"""
    dvdt = np.gradient(v, t)
    mask = np.abs(v) > v_eps
    if mask.sum() < 50:
        raise ValueError('法B 有效样本不足（%d<50，|θ̇|>%.2g）' % (mask.sum(), v_eps))
    X = np.column_stack([dvdt[mask], np.sign(v[mask]), v[mask]])
    return _lstsq_ci(X, tau[mask])


def _fmt(label, theta, err, dof, s2):
    j, tauf, b = theta
    ej, et, eb = err
    dj = 100.0 * (j - PRIOR_J) / PRIOR_J
    dt_ = 100.0 * (tauf - PRIOR_TAUF) / PRIOR_TAUF
    lines = [
        '%s：dof=%d σ²=%.3e' % (label, dof, s2),
        '  J  = %.3e ± %.1e kg·m²   （先验 4.06e-6，偏差 %+0.1f%%）' % (j, ej, dj),
        '  τf = %.3e ± %.1e N·m     （先验 0.9e-3，偏差 %+0.1f%%）' % (tauf, et, dt_),
        '  b  = %.3e ± %.1e N·m·s/rad（首测，无先验）' % (b, eb),
    ]
    return '\n'.join(lines), j, tauf, b


def fitRows(t, th, sv, tc, modes=None):
    """GUI 入口：吃力控台缓冲（列表），返回报告文本。优先仅用 mode=='id' 段。"""
    t = np.asarray(t, dtype=float)
    th = np.asarray(th, dtype=float)
    v = np.asarray(sv, dtype=float)
    tau = np.asarray(tc, dtype=float)
    if modes is not None:
        modes = list(modes)
        keep = [i for i, m in enumerate(modes) if m == 'id']
        if len(keep) > 50:                     # id 段足够才过滤，否则全量（离线 CLI 场景）
            t, th, v, tau = t[keep], th[keep], v[keep], tau[keep]
        else:
            keep = None
    if len(t) < 50:
        raise ValueError('样本不足（%d<50）' % len(t))
    lines = ['样本 %d 点，时长 %.1f s' % (len(t), t[-1] - t[0])]
    repA, jA, fA, bA = _fmt('法A 冲量法（主，零微分）', *fit_impulse(t, th, v, tau))
    repB, jB, fB, bB = _fmt('法B 微分法（对照）', *fit_diff(t, th, v, tau))
    lines += [repA, repB]
    cons = 100.0 * abs(jA - jB) / max(abs(jA), 1e-12)
    lines.append('双法自洽：|ΔJ|/J = %.1f%%（判据 ≤20%%）' % cons)
    lines.append('判据对照：J 偏差 %.1f%%（≤±30%%）｜τf 偏差 %.1f%%（≤±50%%）'
                 % (100.0 * (jA - PRIOR_J) / PRIOR_J,
                    100.0 * (fA - PRIOR_TAUF) / PRIOR_TAUF))
    return '\n'.join(lines)


def _load_csv(path):
    """读力控台导出 CSV（列 t_s,th,sv,tc）。path 由调用方解析：GUI 侧限定 QFileDialog
    来源（Mimosa 路径安全收口，同 workAreaTabbedWidget 的 JSON 加载模式）。"""
    t, th, sv, tc = [], [], [], []
    with open(path, newline='', encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            try:
                t.append(float(row['t_s']))
                th.append(float(row['th']))
                sv.append(float(row['sv']))
                tc.append(float(row['tc']))
            except (KeyError, ValueError, TypeError):
                continue
    if len(t) < 50:
        raise ValueError('有效样本不足：%d（需 ≥50）' % len(t))
    return np.asarray(t), np.asarray(th), np.asarray(sv), np.asarray(tc)


def _selftest():
    """合成数据自检：已知真值仿真（含量化噪声）→ 拟合 → 报回收误差。"""
    J, tauf, b = 4.06e-6, 0.9e-3, 2.0e-6
    A, f0, f1, T = 4e-3, 0.5, 10.0, 20.0
    dt, t = 2e-4, 0.0
    th, v, phase = 0.0, 0.0, 0.0
    buf_t, buf_th, buf_v, buf_tau = [], [], [], []
    next_ms = 0.0
    while t < T:
        f = f0 + (f1 - f0) * (t / T)
        tau = A * math.sin(phase)
        fr = tauf * (1.0 if v > 0 else -1.0) + b * v
        # 半隐式欧拉（摩擦用更新前速度，避免过零数值振铃）
        a = (tau - fr) / J
        v1 = v + a * dt
        th += 0.5 * (v + v1) * dt
        v = v1
        phase += 2.0 * math.pi * f * dt
        t += dt
        if t >= next_ms:                      # 10ms 探针格点 + 量化（th 0.0015rad / sv 0.001rad/s）
            buf_t.append(t)
            buf_th.append(round(th / 0.0015) * 0.0015)
            buf_v.append(round(v / 0.001) * 0.001)
            buf_tau.append(round(tau / 1e-5) * 1e-5)
            next_ms += 0.01
    print('合成真值：J=%.3e τf=%.3e b=%.3e（chirp A=4mN·m 0.5→10Hz/20s，量化噪声已加）'
          % (J, tauf, b))
    rep = fitRows(buf_t, buf_th, buf_v, buf_tau)
    print(rep)


def main(argv):
    if len(argv) >= 2 and argv[1] == '--selftest':
        _selftest()
        return 0
    print(__doc__)
    print('用法：python identFit.py --selftest（数据拟合走力控台"辨识拟合 / 拟合CSV…"按钮）')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
