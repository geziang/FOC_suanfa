#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""三环整定台（2026-09-20 立项，REF-13 裁剪方法论的第一个正面应用）。

定位（EXP-02/REF-13）：
- 本页是"使用面的浓缩"——进页即整定工作区：环选择 → 大波形 → 参数卡 → 目标阶跃；
- 无串口控制台（看串口去「命令行交互」页），无树形全景（在「设备」页）；
- 全部配置面已冻结（前缀 M / 115200-8N1 / 拉取模式），本页只暴露使用面：
  环预设、PID 增益、目标幅值、曲线勾选——这些"随实验而变"的量永不写死。

环预设（一键原子序列，切环安全纪律 REF-12 §6.2 —— 任何一步不可省）：
  ① 目标归零 → ② 力矩类型 MT → ③ 控制模式 MC → ④ 曲线变量位图 →
  ⑤ 降采样 → ⑥ 参数卡/目标单位切换 → ⑦ 未开流则自动开流
"""
from PyQt5 import QtCore, QtWidgets

from src.gui.configtool.connectionControl import ConnectionControlGroupBox
from src.gui.configtool.graphicWidget import SimpleFOCGraphicWidget
from src.gui.sharedcomnponets.sharedcomponets import (WorkAreaTabWidget,
                                                      GUIToolKit)
from src.simpleFOCConnector import SimpleFOCDevice
from src.debugTrace import trace


class PidCard(QtWidgets.QGroupBox):
    """单个 PID 组的参数卡：P/I/D/斜坡/限幅/Tf 六字段 + 读回/写入。

    读回走 connector.pullPIDConf（应答异步到达并更新 device 上的 PID 对象），
    500ms 后从对象回填字段；写入直接顺序下发六条 SET。
    """

    def __init__(self, title, device, pid, lpf, parent=None):
        super().__init__(title, parent)
        self.device = device
        self.pid = pid
        self.lpf = lpf

        self.grid = QtWidgets.QGridLayout(self)
        labels = ['P', 'I', 'D', '斜坡', '限幅', 'Tf']
        self.fields = []
        for col, text in enumerate(labels):
            lab = QtWidgets.QLabel(text)
            self.grid.addWidget(lab, 0, col)
            edit = QtWidgets.QLineEdit()
            edit.setMinimumWidth(58)
            self.grid.addWidget(edit, 1, col)
            self.fields.append(edit)

        self.readBackButton = QtWidgets.QPushButton('读回')
        self.readBackButton.setIcon(GUIToolKit.getIconByName('pull'))
        self.readBackButton.clicked.connect(self.readBack)
        self.grid.addWidget(self.readBackButton, 2, 0, 1, 3)

        self.writeButton = QtWidgets.QPushButton('写入')
        self.writeButton.setIcon(GUIToolKit.getIconByName('push'))
        self.writeButton.clicked.connect(self.write)
        self.grid.addWidget(self.writeButton, 2, 3, 1, 3)

        self.refreshFromDevice()

    def refreshFromDevice(self):
        values = [self.pid.P, self.pid.I, self.pid.D,
                  self.pid.outputRamp, self.pid.outputLimit, self.lpf.Tf]
        for edit, value in zip(self.fields, values):
            edit.setText(str(value))

    def readBack(self):
        trace('[TUNE] pid readback group=%r', self.pid.cmd)
        if not self.device.isConnected:
            return
        self.device.pullPIDConf(self.pid, self.lpf)
        # 应答异步到达（每条间隔 5ms，共 6 条），500ms 后回填
        QtCore.QTimer.singleShot(500, self.refreshFromDevice)

    def write(self):
        if not self.device.isConnected:
            return
        try:
            p, i, d, ramp, limit, tf = [float(f.text()) for f in self.fields]
        except ValueError:
            QtWidgets.QMessageBox.warning(
                None, '写入', '参数字段必须是数字。')
            return
        trace('[TUNE] pid write group=%r P=%r I=%r', self.pid.cmd, p, i)
        d_ = self.device
        d_.sendProportionalGain(self.pid, p)
        d_.sendIntegralGain(self.pid, i)
        d_.sendDerivativeGain(self.pid, d)
        d_.sendOutputRamp(self.pid, ramp)
        d_.sendOutputLimit(self.pid, limit)
        d_.sendLowPassFilter(self.lpf, tf)
        # 写入自动回读（2026-09-22 与力控卡同役合账，EXP-04 纪律）：六条 SET 后
        # 立即 pull 一轮，500ms 后字段回填固件存储值——写入是否落地看得见
        self.readBack()


class ForceCard(QtWidgets.QGroupBox):
    """力控参数卡（2026-09-22 二级方案落地，REF-14 §5；固件 = 02 号 FocKit_force_ctrl）。

    - 模式按钮 raw/虚拟摆/阻抗 → 发 `mode ...`；
    - 六参数 Gv/θ0/K/D/θ*/tq 写入 → 逐条下发；回显解析 [FC CFG] 行显示固件存储值
      ——写入必读回进 UI（EXP-04：界面输入值不作数，回显值才算数）；
    - 读回按钮 → 发 `pred`（固件附带 [FC CFG] 全参数行 + [FC PRED] 预测量三行）；
    - 探针周期 → `probe <ms>` / `probe off`（[FC DBG] 是判据量化的时基锚点）；
    - [FC DBG] 探针行解析 → 卡底实时读数 θ/θ̇/Iq/τ_cmd；
    - 命令全部走 shell 小写命令面：固件 ForceSession 在 Studio 会话期放行小写行
      （上位机协议字母全大写，判别无歧义）。
    """

    # (命令字, 标签, 默认值=REF-14 §3 定档)
    PARAMS = [
        ('gv', 'Gv (N·m)', '0.005'),
        ('th0', 'θ0 (rad)', '0'),
        ('kk', 'K (N·m/rad)', '0.00254'),
        ('kd', 'D (N·m·s/rad)', '0.000203'),
        ('tp', 'θ* (rad)', '0'),
        ('tq', 'tq (N·m)', '0'),
    ]
    MODES = [('raw', 'raw 裸力矩'), ('pend', '虚拟摆'), ('imp', '阻抗')]

    def __init__(self, device, parent=None):
        super().__init__('力控（02 号 FocKit_force_ctrl）', parent)
        self.device = device
        self.grid = QtWidgets.QGridLayout(self)

        # 模式行（互斥三钮 + 固件模式回显）
        self.modeButtons = {}
        for col, (key, text) in enumerate(self.MODES):
            btn = QtWidgets.QPushButton(text)
            btn.setCheckable(True)
            btn.setToolTip('mode %s' % key)
            btn.clicked.connect(lambda checked, k=key: self.onModeButton(k))
            self.modeButtons[key] = btn
            self.grid.addWidget(btn, 0, col)
        self.modeEcho = QtWidgets.QLabel('固件模式：—')
        self.grid.addWidget(self.modeEcho, 0, 3, 1, 3)

        # 参数字段（两行三列：label+edit 成对）
        self.fields = {}
        for idx, (key, label, default) in enumerate(self.PARAMS):
            row, col = 1 + idx // 3, (idx % 3) * 2
            self.grid.addWidget(QtWidgets.QLabel(label), row, col)
            edit = QtWidgets.QLineEdit(default)
            edit.setMinimumWidth(64)
            edit.setToolTip(key)
            self.fields[key] = edit
            self.grid.addWidget(edit, row, col + 1)

        # 写入 / 读回
        self.writeButton = QtWidgets.QPushButton('写入')
        self.writeButton.setIcon(GUIToolKit.getIconByName('push'))
        self.writeButton.clicked.connect(self.write)
        self.grid.addWidget(self.writeButton, 3, 0, 1, 2)
        self.readButton = QtWidgets.QPushButton('读回')
        self.readButton.setIcon(GUIToolKit.getIconByName('pull'))
        self.readButton.clicked.connect(lambda: self.send('pred'))
        self.grid.addWidget(self.readButton, 3, 2, 1, 2)

        # 探针周期（时基锚点，EXP-06 纪律）
        self.grid.addWidget(QtWidgets.QLabel('探针(ms)'), 3, 4)
        self.probeInput = QtWidgets.QLineEdit('50')
        self.probeInput.setMinimumWidth(48)
        self.grid.addWidget(self.probeInput, 3, 5)
        self.probeSet = QtWidgets.QPushButton('设置')
        self.probeSet.clicked.connect(self.onProbeSet)
        self.grid.addWidget(self.probeSet, 4, 0, 1, 2)
        self.probeOff = QtWidgets.QPushButton('关闭')
        self.probeOff.clicked.connect(lambda: self.send('probe off'))
        self.grid.addWidget(self.probeOff, 4, 2, 1, 2)

        # 回显状态（写入必读回的显示面；绿 = 回显已到达 = 写入落地证据）
        self.echoLabel = QtWidgets.QLabel('固件存储：—（写入/读回后刷新）')
        self.echoLabel.setStyleSheet('color:#888;')
        self.grid.addWidget(self.echoLabel, 5, 0, 1, 6)

        # [FC DBG] 实时读数行
        self.liveLabel = QtWidgets.QLabel('θ — · θ̇ — · Iq — · τcmd —')
        liveFont = self.liveLabel.font()
        liveFont.setBold(True)
        self.liveLabel.setFont(liveFont)
        self.grid.addWidget(self.liveLabel, 6, 0, 1, 6)

        # 串口行监听：commandDataReceived 多播信号（[FC CFG]/[FC DBG] 落此通道，
        # 与命令行页/树视图并行接收互不干扰）
        self.device.commProvider.commandDataReceived.connect(self.onLine)

    def send(self, text):
        if self.device.isConnected:
            self.device.sendCommand(text)

    def onModeButton(self, key):
        for k, btn in self.modeButtons.items():
            btn.setChecked(k == key)
        self.send('mode ' + key)

    def onProbeSet(self):
        try:
            ms = int(float(self.probeInput.text()))
        except ValueError:
            QtWidgets.QMessageBox.warning(None, '探针', '探针周期必须是数字(ms)。')
            return
        self.send('probe %d' % ms)

    def write(self):
        for key, _label, _default in self.PARAMS:
            self.send('%s %s' % (key, self.fields[key].text().strip()))
        # 写入是否落地由回显通道自动确认（[FC CFG] 行到达即刷新 echoLabel）

    def onLine(self, line):
        if line.startswith('[FC CFG]'):
            self._onCfgEcho(line[len('[FC CFG]'):].strip())
        elif line.startswith('[FC DBG]'):
            self._onDbg(line[len('[FC DBG]'):].strip())

    @staticmethod
    def _parseKv(body):
        values = {}
        for token in body.split():
            if '=' in token:
                k, v = token.split('=', 1)
                values[k] = v
        return values

    def _onCfgEcho(self, body):
        values = self._parseKv(body)
        mode = values.get('mode')
        if mode:
            for k, btn in self.modeButtons.items():
                btn.setChecked(k == mode)
            self.modeEcho.setText('固件模式：%s' % mode)
        shown = ' '.join('%s=%s' % (k, values[k]) for k in
                         ('gv', 'th0', 'k', 'd', 'tp', 'tq', 'probe') if k in values)
        self.echoLabel.setText('固件存储：%s' % (shown or '—'))
        self.echoLabel.setStyleSheet('color:#2e7d32;')

    def _onDbg(self, body):
        values = self._parseKv(body)

        def g(key):
            try:
                return float(values[key])
            except (KeyError, ValueError):
                return None

        def fmt(v, spec):
            return '—' if v is None else spec % v

        self.liveLabel.setText('θ %s · θ̇ %s · Iq %s · τcmd %s' % (
            fmt(g('th'), '%.3f'), fmt(g('sv'), '%.3f'),
            fmt(g('iq'), '%.4f'), fmt(g('tc'), '%.4f')))


class TuningBenchWidget(WorkAreaTabWidget):
    """整定台：连接条 + 环预设（三环+力控）+ 大波形 + 参数卡 + 目标阶跃 + 大字号读数。"""

    # 环预设表：力矩类型/控制模式/曲线变量勾选（顺序 Target,Vq,Vd,Cq,Cd,Vel,Angle）
    # /降采样/目标单位/默认幅值/红线（电流环 0.5A、力控 0.5A=Iq 域）/参数卡/判据响应通道
    LOOPS = {
        'current': dict(title='电流环', torque=2, motion=0,
                        vars=[True, True, True, True, True, False, True],
                        downsample=100, unit='A', default=0.15, redline=0.5,
                        cards=('currentQ', 'currentD'), response=3),
        'velocity': dict(title='速度环', torque=2, motion=1,   # 级联版(2026-09-21)：MT2+MC1
                         vars=[True, True, False, False, False, True, False],
                         downsample=100, unit='rad/s', default=3.0, redline=None,
                         cards=('velocity',), response=5),
        'position': dict(title='位置环', torque=2, motion=2,   # 级联版(2026-09-21)：MT2+MC2，踩冻结速度环
                         vars=[True, False, False, False, False, True, True],
                         downsample=100, unit='rad', default=1.5708, redline=None,
                         cards=('position',), response=6),
        # 力控（2026-09-22 二级落地，REF-14 §5）：固件 02 号 FocKit_force_ctrl。
        # 命令走小写 shell 面（固件 ForceSession 会话期放行），不走 MT/MC 参数卡；
        # 曲线取 Cq(Iq mA)/Vel/Angle（Iq=电流即力传感器）；原生 target 在力控固件
        # =力律输出的 Iq 指令(A)，故目标单位 A、红线同 0.5A；判据响应通道=Angle。
        'force': dict(title='力控', torque=2, motion=0,
                      vars=[True, False, False, True, False, True, True],
                      downsample=100, unit='A', default=0.0, redline=0.5,
                      cards=('force',), response=6),
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        trace('[TUNE] TuningBenchWidget.__init__ enter')
        self.device = SimpleFOCDevice.getInstance()
        self.activeLoop = None

        self.setObjectName('tuningBench')
        self.verticalLayout = QtWidgets.QVBoxLayout(self)

        # ── 连接条（复用零配置组件：端口下拉 + 获取参数 + 连接）──
        self.connectionControl = ConnectionControlGroupBox()
        self.verticalLayout.addWidget(self.connectionControl)

        # ── 环选择：三个大按钮，互斥 ──
        self.loopBar = QtWidgets.QFrame()
        self.loopLayout = QtWidgets.QHBoxLayout(self.loopBar)
        self.loopButtons = {}
        for key, cfg in self.LOOPS.items():
            btn = QtWidgets.QPushButton(cfg['title'])
            btn.setCheckable(True)
            btn.setMinimumHeight(42)
            btn.clicked.connect(lambda checked, k=key: self.onLoopButton(k))
            self.loopButtons[key] = btn
            self.loopLayout.addWidget(btn)
        self.loopLayout.addStretch(1)
        self.verticalLayout.addWidget(self.loopBar)

        # ── 主区：大波形（左） + 控制列（右）──
        self.mainSplit = QtWidgets.QHBoxLayout()
        self.graphicWidget = SimpleFOCGraphicWidget()
        self.mainSplit.addWidget(self.graphicWidget, 5)

        self.sideColumn = QtWidgets.QWidget()
        self.sideLayout = QtWidgets.QVBoxLayout(self.sideColumn)

        # 参数卡堆：按环显隐（电流环 Q/D 两张，速度/位置各一张，力控一张 ForceCard）
        d_ = self.device
        self.pidCards = {
            'velocity': PidCard('速度环 PID', d_, d_.PIDVelocity, d_.LPFVelocity),
            'position': PidCard('位置环 P', d_, d_.PIDAngle, d_.LPFAngle),
            'currentQ': PidCard('电流 Q 轴 PID（目标即页面目标）', d_, d_.PIDCurrentQ, d_.LPFCurrentQ),
            'currentD': PidCard('电流 D 轴 PID（目标恒 0）', d_, d_.PIDCurrentD, d_.LPFCurrentD),
            'force': ForceCard(d_),
        }
        # D 轴目标由库硬性固定为 0（BLDCMotor::loopFOC 的 foc_current 分支：
        # PID_current_d(-current.d)，正交无弱磁）——D 轴只整定抑制增益
        self.pidCards['currentD'].setToolTip(
            'D 轴电流目标由 SimpleFOC 硬性固定为 0（loopFOC: '
            'voltage.d = PID_current_d(-current.d)），不存在 D 轴目标；'
            '本卡只整定 D 轴电流的抑制增益（把解耦/漏磁电流压回零）。')
        for card in self.pidCards.values():
            self.sideLayout.addWidget(card)
            card.hide()

        # ── 目标/激励控制 ──
        self.targetBox = QtWidgets.QGroupBox('目标 / 阶跃激励')
        self.targetGrid = QtWidgets.QGridLayout(self.targetBox)
        self.targetInput = QtWidgets.QLineEdit()
        self.targetInput.setMinimumWidth(70)
        self.unitLabel = QtWidgets.QLabel('—')
        self.unitLabel.setMinimumWidth(48)
        self.targetGrid.addWidget(QtWidgets.QLabel('幅值'), 0, 0)
        self.targetGrid.addWidget(self.targetInput, 0, 1)
        self.targetGrid.addWidget(self.unitLabel, 0, 2)

        # 当前目标实时回读（MG0 轮询，0.2s 刷新）：显示电机此刻的真实目标，
        # 与"幅值（想设多少）"语义分开；阶跃点下后看它确认命令已生效。
        # 标题+数值合成单标签、横跨整行——窄列里拆两个控件会被网格挤压变形
        self.currentTargetLabel = QtWidgets.QLabel('当前目标：—')
        currentFont = self.currentTargetLabel.font()
        currentFont.setPointSize(14)
        currentFont.setBold(True)
        self.currentTargetLabel.setFont(currentFont)
        self.currentTargetLabel.setStyleSheet('color:#e53935;')
        self.currentTargetLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.currentTargetLabel.setMinimumHeight(26)
        self.targetGrid.addWidget(self.currentTargetLabel, 1, 0, 1, 3)

        self.zeroButton = QtWidgets.QPushButton('归零')
        self.zeroButton.setToolTip('目标设 0（换环/收尾前先归零）')
        self.zeroButton.clicked.connect(lambda: self.sendTarget(0.0))
        self.plusButton = QtWidgets.QPushButton('▶ +幅值')
        self.plusButton.clicked.connect(
            lambda: self.sendTarget(self.targetValue()))
        self.minusButton = QtWidgets.QPushButton('▶ −幅值')
        self.minusButton.clicked.connect(
            lambda: self.sendTarget(-self.targetValue()))
        self.targetGrid.addWidget(self.zeroButton, 2, 0)
        self.targetGrid.addWidget(self.plusButton, 2, 1)
        self.targetGrid.addWidget(self.minusButton, 2, 2)

        self.enableButton = QtWidgets.QPushButton('使能')
        self.enableButton.setCheckable(True)
        self.enableButton.clicked.connect(self.onEnableToggle)
        self.targetGrid.addWidget(self.enableButton, 3, 0, 1, 3)

        self.sideLayout.addWidget(self.targetBox)
        self.sideLayout.addStretch(1)
        self.mainSplit.addWidget(self.sideColumn, 2)
        self.verticalLayout.addLayout(self.mainSplit, 1)

        # ── 大字号读数行（数据来自 MG 轮询，零额外流量）──
        # 颜色与曲线一一对应（Target=红 / Vel=橙 / Angle=绿，同 signalColors），
        # 图上看哪条线、下面就看哪个数（2026-09-20 验收反馈：目标值要显眼、配合图像）
        self.readoutBar = QtWidgets.QFrame()
        self.readoutLayout = QtWidgets.QHBoxLayout(self.readoutBar)
        self.readoutLabels = {}
        self.readoutCaptions = {}
        readoutStyle = {
            'target': ('目标', 'color:#e53935;'),
            'velocity': ('速度 (rad/s)', 'color:#fb8c00;'),
            'angle': ('角度 (rad)', 'color:#43a047;'),
        }
        for key, (name, color) in readoutStyle.items():
            caption = QtWidgets.QLabel(name)
            value = QtWidgets.QLabel('—')
            font = value.font()
            font.setPointSize(20)
            font.setBold(True)
            value.setFont(font)
            value.setStyleSheet(color)
            value.setMinimumWidth(130)
            self.readoutCaptions[key] = caption
            self.readoutLabels[key] = value
            self.readoutLayout.addWidget(caption)
            self.readoutLayout.addWidget(value)
        self.readoutLayout.addStretch(1)
        # 固件模式指示（2026-09-21 失控案防再犯）：与本环不符标红——
        # 数据源 device.controlType/torqueType（MC/MT 应答解析），连接拉取与
        # 原子模式重发都会刷新它
        self.fwModeCaption = QtWidgets.QLabel('固件模式')
        self.fwModeLabel = QtWidgets.QLabel('—')
        fwFont = self.fwModeLabel.font()
        fwFont.setBold(True)
        self.fwModeLabel.setFont(fwFont)
        self.readoutLayout.addWidget(self.fwModeCaption)
        self.readoutLayout.addWidget(self.fwModeLabel)
        # 判据行（阶跃测量自动结算：tr/σ%/ts/ess，本次 vs 上次两轮复现对比）
        self.metricsCaption = QtWidgets.QLabel('判据')
        self.metricsNow = QtWidgets.QLabel('（点 ▶±幅值 后自动测量）')
        metricsFont = self.metricsNow.font()
        metricsFont.setBold(True)
        self.metricsNow.setFont(metricsFont)
        self.metricsNow.setStyleSheet('color:#1565c0;')
        self.metricsPrev = QtWidgets.QLabel('')
        self.metricsPrev.setStyleSheet('color:#888;')
        self.metricsNow.setMinimumWidth(420)
        self.readoutLayout.addWidget(self.metricsCaption)
        self.readoutLayout.addWidget(self.metricsNow)
        self.readoutLayout.addWidget(self.metricsPrev)
        self.verticalLayout.addWidget(self.readoutBar)

        self.readoutTimer = QtCore.QTimer(self)
        self.readoutTimer.setInterval(200)
        self.readoutTimer.timeout.connect(self.refreshReadouts)
        self.readoutTimer.start()

        self.device.addConnectionStateListener(self)
        self.connectionStateChanged(self.device.isConnected)
        trace('[TUNE] TuningBenchWidget.__init__ done')

    # ── 环预设 ─────────────────────────────────────────────
    def onLoopButton(self, key):
        btn = self.loopButtons[key]
        if not btn.isChecked():  # 不允许手动取消选中：换环要点另一个环
            btn.setChecked(True)
            return
        if not self.applyPreset(key):
            btn.setChecked(False)

    def applyPreset(self, key):
        cfg = self.LOOPS[key]
        if not self.device.isConnected:
            QtWidgets.QMessageBox.information(
                None, cfg['title'], '请先在上方连接条选端口并连接，再切环。')
            return False
        trace('[TUNE] apply preset loop=%r', key)
        d_ = self.device
        g = self.graphicWidget
        panel = g.controlPlotWidget

        # ① 目标归零（切环安全纪律，REF-12 §6.2）
        d_.sendTargetValue(0)
        # ②③ 力矩类型 + 控制模式
        d_.sendTorqueType(cfg['torque'])
        d_.sendControlType(cfg['motion'])
        # ④ 曲线变量：程序化勾选，stateChanged 链路会自动重发 MMS 位图
        for checkBox, want in zip(panel.signalCheckBox, cfg['vars']):
            checkBox.setChecked(want)
        # ⑤ 降采样
        panel.downampleValue.setText(str(cfg['downsample']))
        d_.sendMonitorDownsample(cfg['downsample'])
        # ⑥ 参数卡显隐 + 目标单位/默认幅值
        self.activeLoop = key
        for other in self.loopButtons.values():
            other.setChecked(other is self.loopButtons[key])
        for cardKey, card in self.pidCards.items():
            card.setVisible(cardKey in cfg['cards'])
        self.unitLabel.setText(cfg['unit'])
        # 电流环/力控目标是 Q 轴电流（foc_current 力矩模式 target=Iq；D 轴由库恒 0；
        # 力控固件下 target=力律逐拍重发的 Iq 指令——读数区可直接看力律在想什么）
        self.readoutCaptions['target'].setText(
            'Iq目标 (A)' if key in ('current', 'force')
            else '目标 (%s)' % cfg['unit'])
        self.targetInput.setText(str(cfg['default']))
        self._updateTargetGuard()
        # ⑦ 未开流则自动开流（开流动作会再发一遍 MMD+MMS，幂等）
        if g.currentStatus is g.initialConnectedState:
            panel.startStoPlotAction()
        # 力控卡激活即拉一轮参数与预测量回显（pred 附带 [FC CFG] 全参数行）
        if key == 'force':
            d_.sendCommand('pred')
        return True

    # ── 目标 / 使能 ────────────────────────────────────────
    def targetValue(self):
        try:
            return float(self.targetInput.text())
        except ValueError:
            QtWidgets.QMessageBox.warning(None, '目标', '幅值必须是数字。')
            return 0.0

    def sendTarget(self, value):
        if not self.device.isConnected:
            return
        trace('[TUNE] send target=%r loop=%r', value, self.activeLoop)
        # 原子模式保证（2026-09-21 失控案防再犯）：每次非零目标前幂等重发本环
        # MT+MC——断连重连/板卡复位/命令丢失后模式永远就位，目标不再落进力矩
        # 模式被解释成恒压（Vq 直通无钳位的失控根源）。
        if value != 0 and self.activeLoop:
            cfg = self.LOOPS[self.activeLoop]
            self.device.sendTorqueType(cfg['torque'])
            self.device.sendControlType(cfg['motion'])
        self.device.sendTargetValue(value)
        # 非零目标 = 阶跃：通知波形开启判据测量窗。
        # Cq/Cd 曲线单位是 mA（库 monitor ×1000），目标按通道尺度换算
        if value != 0 and self.activeLoop:
            cfg = self.LOOPS[self.activeLoop]
            yRef = value * 1000.0 if cfg['response'] in (3, 4) else value
            self.graphicWidget.beginStepMeasure(yRef, cfg['response'])

    def _updateTargetGuard(self):
        """电流环 0.5A 持续红线提示（SPEC-T）；其余环无红线。"""
        cfg = self.LOOPS.get(self.activeLoop)
        if cfg is None:
            self.unitLabel.setStyleSheet('')
            return
        try:
            value = float(self.targetInput.text())
        except ValueError:
            value = None
        over = cfg['redline'] is not None and value is not None \
            and abs(value) > cfg['redline']
        self.unitLabel.setStyleSheet(
            'color: red; font-weight: bold;' if over else '')
        self.unitLabel.setToolTip(
            '超过持续电流红线 0.5A（SPEC-T）！' if over else '')

    def onEnableToggle(self, checked):
        if not self.device.isConnected:
            self.enableButton.setChecked(False)
            return
        self.device.sendDeviceStatus(1 if checked else 0)
        self.enableButton.setText('使能' if checked else '失能')

    # ── 读数 / 连接状态 ────────────────────────────────────
    def refreshReadouts(self):
        d_ = self.device
        cfg = self.LOOPS.get(self.activeLoop)
        unit = cfg['unit'] if cfg else '—'
        target = float(d_.targetNow or 0)
        self.readoutLabels['target'].setText('%.3f' % target)
        self.readoutLabels['velocity'].setText('%.3f' % float(d_.velocityNow or 0))
        self.readoutLabels['angle'].setText('%.3f' % float(d_.angleNow or 0))
        prefix = '当前Q轴目标：' if self.activeLoop == 'current' else '当前目标：'
        self.currentTargetLabel.setText('%s%.3f %s' % (prefix, target, unit))
        self._renderFwMode()
        self._renderMetrics()

    def _renderFwMode(self):
        """固件模式指示：显示 controller/torque 实际状态，与本环预期不符标红。"""
        motionNames = {0: '力矩', 1: '速度', 2: '位置', 3: '速度开环', 4: '位置开环'}
        torqueNames = {0: '电压', 1: '直流电流', 2: 'FOC电流'}
        m = self.device.controlType
        t = self.device.torqueType
        self.fwModeLabel.setText('%s/%s' % (motionNames.get(m, '?'),
                                            torqueNames.get(t, '?')))
        if self.activeLoop:
            cfg = self.LOOPS[self.activeLoop]
            ok = (m == cfg['motion'] and t == cfg['torque'])
            self.fwModeLabel.setStyleSheet(
                'color:#2e7d32;' if ok else 'color:#d32f2f;')
            self.fwModeCaption.setToolTip(
                '固件 controller/torque 实际状态（MC/MT 应答解析）。\n'
                '与本环不符时标红——此时发目标会被错误解释，'
                '点一次 ▶±幅值 即自动纠正（原子模式重发）。')
        else:
            self.fwModeLabel.setStyleSheet('')

    def _renderMetrics(self):
        m = self.graphicWidget.lastMetrics
        if m is None:
            return
        self.metricsNow.setText(self._fmtMetrics(m))
        p = self.graphicWidget.prevMetrics
        if p is not None and p.get('valid'):
            self.metricsPrev.setText('｜上次 ' + self._fmtMetrics(p))

    def _fmtMetrics(self, m):
        """tr/σ/ts 按秒自适应显示；ess 换算回当前环单位（Cq 曲线 mA → A）；
        电流环附加带宽法速读 ω≈4/ts（一阶近似）。"""
        if not m.get('valid'):
            return '无效窗（点数不足/步幅为0）'

        def sec(v):
            if v is None:
                return '—'
            return '%.0fms' % (v * 1000) if v < 1.0 else '%.2fs' % v

        ess = m['ess']
        if ess is None:
            essTxt = '—'
        elif self.activeLoop == 'current':
            essTxt = '%.4f A' % (ess / 1000.0)
        else:
            essTxt = '%.4f' % ess
        sigma = '—' if m['sigma'] is None else '%.1f%%' % m['sigma']
        text = 'tr %s · σ %s · ts %s · ess %s' % (
            sec(m['tr']), sigma, sec(m['ts']), essTxt)
        if self.activeLoop == 'current' and m['ts'] and m['ts'] > 0:
            text += ' · ω≈%.0f rad/s' % (4.0 / m['ts'])
        return text

    def connectionStateChanged(self, isConnected):
        trace('[TUNE] connectionStateChanged connected=%r', isConnected)
        for btn in self.loopButtons.values():
            btn.setEnabled(isConnected)
        if not isConnected:
            self.enableButton.setChecked(False)
            self.enableButton.setText('使能')

    # ── Tab 接口 ───────────────────────────────────────────
    def getTabIcon(self):
        return GUIToolKit.getIconByName('loop')

    def getTabName(self):
        return '整定台'  # 2026-09-22 起含力控（三环 + 力控一页）
