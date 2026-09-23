#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""力控台（2026-09-23 三级落地，REF-14 §5；固件 = 02 号 FocKit_force_ctrl）。

为什么独立成页 + 为什么波形不吃 Monitor 流（2026-09-23 用户决策与实测结论）：
- 02 号固件力律走电压力矩模式（setTorqueTarget → V=R·τ/KT），SimpleFOC 在
  voltage 模式下不刷新 currents —— Monitor 流 Cq 列恒 0，target 列是伏特非
  安培；力控判据量 θ/θ̇/Iq/τ 只存在于 [FC DBG] 探针行 → 本页波形直接解析探针行
  （自带固件 ms 时基，正是判据量化的时基锚点，EXP-06 纪律）。
- 与三环整定台分开：命令面（小写 shell）与曲线源（探针行）都与三环不同，
  混排一页容易误切环/误读曲线。

布局：连接条 + 大波形（上=机械量 θ/θ̇，下=力量 τ/Iq 双纵轴）+ ForceCard
（模式三钮 / 六参数写入读回 / 探针周期 / 实时读数）。
"""
import csv
import io
import os
import time
from pathlib import Path

import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

from src.gui.configtool.connectionControl import ConnectionControlGroupBox
from src.gui.sharedcomnponets.sharedcomponets import (WorkAreaTabWidget,
                                                      GUIToolKit)
from src.simpleFOCConnector import SimpleFOCDevice
from src.debugTrace import trace


class ForceCard(QtWidgets.QGroupBox):
    """力控参数卡（2026-09-22 二级方案落地，2026-09-23 迁入力控台；固件 = 02 号）。

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
        super().__init__('力控参数（02 号 FocKit_force_ctrl）', parent)
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
        # 与命令行页/树视图/本页波形并行接收互不干扰）
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


class ForceScope(QtWidgets.QGroupBox):
    """力控波形区：直接解析 [FC DBG] 探针行（判据量的原生通道）。

    - 上图「机械量」：θ (rad) 绿 / θ̇ (rad/s) 橙，共纵轴；
    - 下图「力量」：τ_cmd (N·m) 红（左轴）+ Iq (A) 蓝（右轴，双 ViewBox）——
      电压模式下 Iq=τ/KT 为账面值，与 τ 同源共线，双轴只为刻度各自可读；
    - 时基 = 固件探针行的 ms 字段（换算秒，首行归零）；ms 回退视为固件复位，
      自动清屏重开时基（判据窗不跨复位拼接）；
    - 采样/重绘分离（同 graphicWidget 纪律）：行回调只入队，QTimer 批量搬入
      显示数组统一重绘；暂停照常记录（暂停后仍可导整段），只停重绘；
    - 缓冲上限 MAX_POINTS，探针 10ms 时约 10 分钟窗口、50ms 约 5 分钟。
    """

    MAX_POINTS = 6000
    # (字段名, 显示名, 颜色) —— 与整定台读数配色习惯一致：绿/橙/红/蓝
    CURVES = [
        ('th', 'θ (rad)', '#43a047'),
        ('sv', 'θ̇ (rad/s)', '#fb8c00'),
        ('tc', 'τ_cmd (N·m)', '#e53935'),
        ('iq', 'Iq (A)', '#1e88e5'),
    ]

    def __init__(self, parent=None):
        super().__init__('力控波形（[FC DBG] 探针流 · 固件 ms 时基）', parent)
        self.device = SimpleFOCDevice.getInstance()

        # 显示缓冲：每通道一对 (t 列表, y 列表)；行原始记录供 CSV（含 mode 列）
        self.bufT = []
        self.bufY = {key: [] for key, _n, _c in self.CURVES}
        self.bufMode = []
        self.pendingSamples = []
        self.droppedRows = 0
        self.t0Ms = None       # 时基准点（固件 ms）
        self.lastMs = None
        self.paused = False
        self.arrivalStamps = []  # 行到达时刻（GUI 线程单调钟），供行率统计

        self.grid = QtWidgets.QGridLayout(self)

        # ── 上图：机械量 θ/θ̇ ──
        self.plotMech = pg.PlotWidget()
        self.plotMech.plotItem.setLabel('left', '机械量')
        self.plotMech.plotItem.setLabel('bottom', 't (s)')
        self.plotMech.showGrid(x=True, y=True, alpha=0.3)
        self.grid.addWidget(self.plotMech, 0, 0, 1, 2)

        # ── 下图：力量 τ（左）/ Iq（右，双 ViewBox）──
        self.plotForce = pg.PlotWidget()
        p1 = self.plotForce.plotItem
        p1.setLabel('left', 'τ_cmd (N·m)')
        p1.setLabel('bottom', 't (s)')
        self.plotForce.showGrid(x=True, y=True, alpha=0.3)
        self.grid.addWidget(self.plotForce, 1, 0, 1, 2)

        p1.showAxis('right')
        self.vbRight = pg.ViewBox()
        p1.scene().addItem(self.vbRight)
        p1.getAxis('right').linkToView(self.vbRight)
        self.vbRight.setXLink(p1)
        p1.getAxis('right').setLabel('Iq (A)')

        def syncRightView():
            self.vbRight.setGeometry(p1.vb.sceneBoundingRect())

        p1.vb.sigResized.connect(syncRightView)
        self.vbRight.enableAutoRange(x=False, y=True)

        # 上下图 X 轴联动（缩放/平移同步看同一段）
        self.plotMech.setXLink(self.plotForce)

        # 曲线对象：th/sv 归上图，tc 归下图左轴，iq 归下图右轴
        self.curves = {}
        for key, name, color in self.CURVES:
            pen = pg.mkPen(color, width=2)
            if key in ('th', 'sv'):
                curve = self.plotMech.plot(pen=pen, name=name)
            elif key == 'tc':
                curve = p1.plot(pen=pen, name=name)
            else:  # iq
                curve = pg.PlotDataItem(pen=pen)
                self.vbRight.addItem(curve)
            self.curves[key] = curve

        # ── 控制行：曲线勾选 + 暂停/清空/导出 + 状态 ──
        ctrl = QtWidgets.QFrame()
        ctrlLayout = QtWidgets.QHBoxLayout(ctrl)
        self.curveChecks = {}
        for key, name, color in self.CURVES:
            box = QtWidgets.QCheckBox(name)
            box.setChecked(True)
            box.setStyleSheet('color:%s; font-weight:bold;' % color)
            box.stateChanged.connect(
                lambda state, k=key: self.curves[k].setVisible(state != 0))
            self.curveChecks[key] = box
            ctrlLayout.addWidget(box)
        self.pauseButton = QtWidgets.QPushButton('暂停')
        self.pauseButton.setIcon(GUIToolKit.getIconByName('pause'))
        self.pauseButton.clicked.connect(self.onPauseToggle)
        ctrlLayout.addWidget(self.pauseButton)
        self.clearButton = QtWidgets.QPushButton('清空')
        self.clearButton.setIcon(GUIToolKit.getIconByName('restart'))
        self.clearButton.clicked.connect(self.clearPlot)
        ctrlLayout.addWidget(self.clearButton)
        self.exportButton = QtWidgets.QPushButton('导出CSV')
        self.exportButton.setIcon(GUIToolKit.getIconByName('save'))
        self.exportButton.clicked.connect(self.exportCsv)
        ctrlLayout.addWidget(self.exportButton)
        ctrlLayout.addStretch(1)
        self.statusLabel = QtWidgets.QLabel('等待 [FC DBG] 探针行…')
        self.statusLabel.setStyleSheet('color:#888;')
        ctrlLayout.addWidget(self.statusLabel)
        self.grid.addWidget(ctrl, 2, 0, 1, 2)

        # 串口行监听（多播，与 ForceCard 并行）
        self.device.commProvider.commandDataReceived.connect(self.onLine)

        # 采样/重绘分离：20Hz 定时器批量搬入 + 统一重绘
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.setInterval(50)
        self.redrawTimer.timeout.connect(self.drainAndRedraw)
        self.redrawTimer.start()

    # ── 数据面 ─────────────────────────────────────────────
    def onLine(self, line):
        """串口行回调：只校验解析 + 入队，不做任何绘图。"""
        if not line.startswith('[FC DBG]'):
            return
        values = ForceCard._parseKv(line[len('[FC DBG]'):].strip())
        try:
            ms = float(values['ms'])
            row = {'ms': ms,
                   'mode': values.get('mode', ''),
                   'th': float(values['th']), 'sv': float(values['sv']),
                   'iq': float(values['iq']), 'tc': float(values['tc'])}
        except (KeyError, ValueError, TypeError):
            self.droppedRows += 1
            return
        self.pendingSamples.append(row)

    def drainAndRedraw(self):
        """帧回调：积压批量入显示数组（含复位检测/上限裁剪），统一重绘一次。"""
        if not self.pendingSamples:
            self._refreshStatus()
            return
        rows = self.pendingSamples
        self.pendingSamples = []
        now = time.monotonic()
        self.arrivalStamps.append(now)
        # 复位检测：固件 ms 回退（复位后 millis 归零）→ 清屏重开时基，
        # 判据窗不跨复位拼接
        if self.t0Ms is not None and rows[0]['ms'] < self.lastMs - 1000.0:
            trace('[FORCE SCOPE] firmware ms rewind: %s -> %s, reset timebase',
                  self.lastMs, rows[0]['ms'])
            self._clearBuffers()
        for row in rows:
            if self.t0Ms is None:
                self.t0Ms = row['ms']
            self.lastMs = row['ms']
            self.bufT.append((row['ms'] - self.t0Ms) / 1000.0)
            self.bufMode.append(row['mode'])
            for key in self.bufY:
                self.bufY[key].append(row[key])
        overflow = len(self.bufT) - self.MAX_POINTS
        if overflow > 0:
            del self.bufT[:overflow]
            del self.bufMode[:overflow]
            for key in self.bufY:
                del self.bufY[key][:overflow]
        if not self.paused:
            self.updatePlot()
        self._refreshStatus()

    def updatePlot(self):
        for key in self.bufY:
            self.curves[key].setData(self.bufT, self.bufY[key])
        # 右轴 ViewBox 自适应（addItem 后 autorange 不总被触发，手动兜底）
        self.vbRight.autoRange()

    def _clearBuffers(self):
        self.bufT.clear()
        self.bufMode.clear()
        for key in self.bufY:
            self.bufY[key].clear()
        self.t0Ms = None
        self.lastMs = None
        for curve in self.curves.values():
            curve.setData([], [])

    # ── 视图/导出面 ────────────────────────────────────────
    def clearPlot(self):
        """清图：显示缓冲清零 + 队列清空 + 时基重开，立即重绘。

        纯本地视图操作：不发串口命令、探针流不断。
        """
        trace('[FORCE SCOPE] clear plot')
        self._clearBuffers()
        self.pendingSamples.clear()
        self.arrivalStamps.clear()

    def onPauseToggle(self):
        self.paused = not self.paused
        self.pauseButton.setText('继续' if self.paused else '暂停')

    def exportCsv(self):
        """导出显示缓冲为 CSV（显示与记录同源；暂停态可导整段）。

        列：sample, t_s, ms(固件), mode, th, sv, iq, tc —— mode 列标记力律切换
        时刻，滞回/释放判据离线处理时按它分段。
        """
        if not self.bufT:
            QtWidgets.QMessageBox.information(
                None, '导出CSV', '缓冲为空，先采一段再导出。')
            return
        defaultName = time.strftime('fockit_force_%Y%m%d_%H%M%S.csv')
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            None, '导出力控探针 CSV', defaultName, 'CSV files (*.csv)')
        if not path:
            return
        # 路径安全收口（Mimosa 要求，同 graphicWidget.exportCsv）：
        # 只取 basename，固定落 Studio 根 exports/ 目录
        root = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
        exportsDir = os.path.join(root, 'exports')
        os.makedirs(exportsDir, exist_ok=True)
        fileName = os.path.basename(path.replace('\x00', '').replace('\\', '/'))
        if not fileName:
            return
        path = os.path.join(exportsDir, fileName)
        msList = [t * 1000.0 + (self.t0Ms or 0.0) for t in self.bufT]
        sio = io.StringIO()
        writer = csv.writer(sio, lineterminator='\n')
        writer.writerow(['sample', 't_s', 'ms', 'mode', 'th', 'sv', 'iq', 'tc'])
        for i in range(len(self.bufT)):
            writer.writerow([i, self.bufT[i], msList[i], self.bufMode[i],
                             self.bufY['th'][i], self.bufY['sv'][i],
                             self.bufY['iq'][i], self.bufY['tc'][i]])
        Path(path).write_text(sio.getvalue(), encoding='utf-8-sig')
        trace('[FORCE SCOPE] CSV exported path=%r points=%d dropped=%d',
              path, len(self.bufT), self.droppedRows)
        QtWidgets.QMessageBox.information(
            None, '导出CSV',
            '已导出 %d 点 × 4 变量（丢弃行 %d）：\n%s' %
            (len(self.bufT), self.droppedRows, path))

    def _refreshStatus(self):
        # 行率：2s 滑窗内的到达行数 / 窗长
        now = time.monotonic()
        self.arrivalStamps = [t for t in self.arrivalStamps if now - t < 2.0]
        rate = len(self.arrivalStamps) / 2.0
        if not self.bufT:
            self.statusLabel.setText('等待 [FC DBG] 探针行…')
        else:
            probeTxt = ('探针≈%.0fms' % ((self.lastMs - self.t0Ms)
                        / max(len(self.bufT) - 1, 1))) if len(self.bufT) > 1 else ''
            self.statusLabel.setText(
                '%.1f 行/s · %d 点 · 丢 %d %s' %
                (rate, len(self.bufT), self.droppedRows, probeTxt))


class ForceBenchWidget(WorkAreaTabWidget):
    """力控台：连接条 + [FC DBG] 波形 + 力控参数卡（与三环整定台分页，2026-09-23）。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        trace('[FORCE] ForceBenchWidget.__init__ enter')
        self.device = SimpleFOCDevice.getInstance()
        self.setObjectName('forceBench')

        self.verticalLayout = QtWidgets.QVBoxLayout(self)

        # ── 连接条（复用零配置组件：端口下拉 + 获取参数 + 连接）──
        self.connectionControl = ConnectionControlGroupBox()
        self.verticalLayout.addWidget(self.connectionControl)

        # ── 主区：大波形（左） + 力控卡（右）──
        self.mainSplit = QtWidgets.QHBoxLayout()
        self.scope = ForceScope()
        self.mainSplit.addWidget(self.scope, 5)
        self.sideColumn = QtWidgets.QWidget()
        self.sideLayout = QtWidgets.QVBoxLayout(self.sideColumn)
        self.card = ForceCard(self.device)
        self.sideLayout.addWidget(self.card)
        self.sideLayout.addStretch(1)
        self.mainSplit.addWidget(self.sideColumn, 2)
        self.verticalLayout.addLayout(self.mainSplit, 1)

        trace('[FORCE] ForceBenchWidget.__init__ done')

    # ── Tab 接口 ───────────────────────────────────────────
    def getTabIcon(self):
        return GUIToolKit.getIconByName('statistics')

    def getTabName(self):
        return '力控台'
