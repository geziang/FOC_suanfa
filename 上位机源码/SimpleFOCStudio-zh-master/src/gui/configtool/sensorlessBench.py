#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""无感台（2026-09-29，无感观测器矩阵战役第一轮；固件 = 03 号 FocKit_sensorless）。

对账架构（战役三条设计约束之一）：AS5600 降役为真值裁判——控制链吃观测器角度
（真无感），本页波形吃 [SL DBG] 探针行的三列角度（the 编码器真值 / thc 控制链角 /
tho 观测器估计）+ dth 误差，同增益有感/无感曲线对照即验收判据载体。

布局：连接条 + 双波形（上=角度对账 the/thc/tho + dth 右轴；下=速度 we/wo + iq 右轴）
+ SensorlessCard（v2 一键式，2026-09-29 简化：观测器二选一/速度/一键启动/参数/实时
读数——原角度源·VF·切无感·三环会话·step·探针的手动流程收进 UI 状态机自动串联，
固件命令面一字不改）。结构沿力控台 forceBench 先例（2026-09-23 三级落地）。
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


class SensorlessCard(QtWidgets.QGroupBox):
    """无感控制卡（v2 一键式·到速判据版）：选观测器→一键启动→拧参数看波形。

    - 无感交付域=中高速：低速盲区约 5~10 rad/s（行业通病）不补丁，ENGAGE_SPEED_FLOOR 把门；
    - 一键启动把原手动流程收进 UI 状态机（vf 命令固件侧每次启动清观测器积分器）：
      ① `obs <选型>` + `vf <速度>` 开环起转，观测器并行收敛；
      ② 到速才切——三条件连续 LOCK_STREAK 拍（50ms 探针≈1s）全达标才动手：
         vfW ≥ max(下限, 0.9×目标)（VF 真到中高速）＋ lk=1（BEMF 过阈）
         ＋ wo ≥ 0.5×vfW（观测器真跟上速度——低速盲区里 lk 会说谎，速度作证）
         → `src obs` 切真无感 → `loop v` 速度闭环（三环冻结档）→ 裸数字速度目标；
      ③ 运行中「速度设定」只发裸数字目标（不重走流程，可下探测盲区边界，失锁告警照实报）；
    - 目标命令=裸数字：StudioBridge 的 'T' 是力矩类型命令非目标（实测踩坑——
      T 10 回显 Torque: volt 且 tt 恒 0，速度目标从未设上）；
    - 超时未达标 → 保持 VF 开开环，改参数→写入→再点启动即重试（固件已清零同权重来）；
    - [SL CFG] 回读驱动观测器按钮同步（EXP-04 写入必读回）；[SL DBG] 喂实时读数与状态机。
    """

    LOCK_STREAK = 20           # 三条件连续达标拍数（50ms 探针 ≈ 1s 稳判）
    ENGAGE_SPEED_FLOOR = 10.0  # [rad/s] 无感交付域下限（低速盲区 5~10，行业通病不补丁）
    LOCK_TIMEOUT_MS = 10000    # 含 VF 爬坡（40 rad/s 需 5s）+ 收敛 + 余量

    def __init__(self, device, parent=None):
        super().__init__('无感控制（03 号 FocKit_sensorless）', parent)
        self.device = device

        # 状态机：idle → vf_ramp →（lk 连续锁定）→ run；超时 → vf_open；停止 → stopped
        self.state = 'idle'
        self.speedValue = None
        self.lkStreak = 0
        self.lockTimer = QtCore.QTimer(self)
        self.lockTimer.setSingleShot(True)
        self.lockTimer.timeout.connect(self.onLockTimeout)

        self.grid = QtWidgets.QGridLayout(self)

        # 观测器二选一（矩阵战役核心：SMO+PLL × 磁链；运行中可热切看表现）
        self.obsButtons = {}
        for col, (key, text) in enumerate([('smo', 'SMO+PLL'), ('flux', '磁链')]):
            btn = QtWidgets.QPushButton(text)
            btn.setCheckable(True)
            btn.setToolTip('obs %s' % key)
            btn.clicked.connect(lambda checked, k=key: self.onObsButton(k))
            self.obsButtons[key] = btn
            self.grid.addWidget(btn, 0, col)
        self.obsButtons['smo'].setChecked(True)
        self.obsEcho = QtWidgets.QLabel('观测器：smo')
        self.grid.addWidget(self.obsEcho, 0, 2, 1, 3)

        # 速度 + 三键（唯一操作面：启动/调速/停止）
        self.speedInput = QtWidgets.QLineEdit('10')
        self.speedInput.setMinimumWidth(48)
        self.grid.addWidget(QtWidgets.QLabel('速度(rad/s)'), 1, 0)
        self.grid.addWidget(self.speedInput, 1, 1)
        self.startButton = QtWidgets.QPushButton('▶ 一键启动')
        self.startButton.clicked.connect(self.onStart)
        self.grid.addWidget(self.startButton, 1, 2)
        self.speedSetButton = QtWidgets.QPushButton('速度设定')
        self.speedSetButton.clicked.connect(self.onSpeedSet)
        self.grid.addWidget(self.speedSetButton, 1, 3)
        self.stopButton = QtWidgets.QPushButton('■ 停止')
        self.stopButton.clicked.connect(self.onStop)
        self.grid.addWidget(self.stopButton, 1, 4)

        # 状态行（状态机走到哪步 + 失锁告警，随 [SL DBG] 自愈刷新）
        self.stateLabel = QtWidgets.QLabel('状态：未启动')
        stateFont = self.stateLabel.font()
        stateFont.setBold(True)
        self.stateLabel.setFont(stateFont)
        self.grid.addWidget(self.stateLabel, 2, 0, 1, 5)

        # 观测器参数（运行中写入即时生效——VF 并行收敛段正是拧参数看表现的场景）
        self.fields = {}
        params = [('sk', '滑模k(V)', '1.5'), ('st', 'LPF(ms)', '2'),
                  ('sdkp', 'PLL kp', '100'), ('sdki', 'PLL ki', '2000'),
                  ('sd', '磁链Td(s)', '0.5')]
        for idx, (key, label, default) in enumerate(params):
            col = (idx % 5) * 2
            self.grid.addWidget(QtWidgets.QLabel(label), 3, col)
            edit = QtWidgets.QLineEdit(default)
            edit.setMinimumWidth(48)
            edit.setToolTip(key)
            self.fields[key] = edit
            self.grid.addWidget(edit, 3, col + 1)
        self.writeButton = QtWidgets.QPushButton('参数写入')
        self.writeButton.setIcon(GUIToolKit.getIconByName('push'))
        self.writeButton.clicked.connect(self.writeParams)
        self.grid.addWidget(self.writeButton, 4, 0, 1, 2)

        # 回读 + 实时读数（thc 控制链角看波形，卡上只留健康三量）
        self.echoLabel = QtWidgets.QLabel('固件回读：—（写入后刷新）')
        self.echoLabel.setStyleSheet('color:#888;')
        self.grid.addWidget(self.echoLabel, 4, 2, 1, 4)
        self.liveLabel = QtWidgets.QLabel('the — · tho — · dth — · 锁 —')
        liveFont = self.liveLabel.font()
        liveFont.setBold(True)
        self.liveLabel.setFont(liveFont)
        self.grid.addWidget(self.liveLabel, 5, 0, 1, 10)

        self.device.commProvider.commandDataReceived.connect(self.onLine)

    def send(self, text):
        if self.device.isConnected:
            self.device.sendCommand(text)

    @staticmethod
    def _num(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return 0.0

    def readSpeed(self, lo=10.0):
        try:
            v = float(self.speedInput.text().strip())
        except ValueError:
            QtWidgets.QMessageBox.warning(None, '速度', '目标速度必须是数字(rad/s)。')
            return None
        if not (lo <= v <= 40.0):
            QtWidgets.QMessageBox.warning(
                None, '速度', '速度范围 %g~40 rad/s。%s' % (
                    lo, '一键启动最低=无感交付域下限（低速盲区约 5~10 rad/s，行业通病不补丁）。'
                    if lo >= 10.0 else
                    '低于约 5~10 进入无感盲区：状态行会失锁告警，即边界实验现场。'))
            return None
        return v

    def onObsButton(self, key):
        for k, btn in self.obsButtons.items():
            btn.setChecked(k == key)
        self.send('obs ' + key)

    def onStart(self):
        v = self.readSpeed()
        if v is None:
            return
        self.speedValue = v
        obsKey = 'flux' if self.obsButtons['flux'].isChecked() else 'smo'
        self.send('obs %s' % obsKey)
        self.send('vf %.3g' % v)
        self.state = 'vf_ramp'
        self.lkStreak = 0
        self.lockTimer.start(self.LOCK_TIMEOUT_MS)
        self._setState('状态：① VF 起转中…（观测器并行收敛，锁定后自动切无感+闭环）', '#e65100')

    def onLockTimeout(self):
        if self.state != 'vf_ramp':
            return
        self.state = 'vf_open'
        self._setState('状态：⚠ 10s 未达标（到速/锁定/跟踪）——保持 VF 开环。'
                       '改参数→写入→再点「一键启动」重试', '#c62828')

    def onSpeedSet(self):
        v = self.readSpeed(lo=0.1)   # 运行中调速允许下探（盲区边界实验入口）
        if v is None:
            return
        self.speedValue = v
        self.send('%.3g' % v)        # 裸数字=速度目标（'T' 是力矩类型命令，非目标）

    def onStop(self):
        self.lockTimer.stop()
        if self.state == 'run':
            self.send('0')            # 裸数字=目标归零（'T'=力矩类型命令，非目标）
            self._setState('状态：已停止（目标=0 减速中）', '#888')
        elif self.state in ('vf_ramp', 'vf_open'):
            self.send('vf off')
            self._setState('状态：已停止（VF 减速中）', '#888')
        else:
            self.send('0')
            self.send('vf off')
            self._setState('状态：已停止', '#888')
        self.state = 'stopped'

    def writeParams(self):
        get = lambda k: self.fields[k].text().strip()
        self.send('sk %s' % get('sk'))
        self.send('st %s' % get('st'))
        self.send('sp %s %s' % (get('sdkp'), get('sdki')))
        self.send('sd %s' % get('sd'))

    def _setState(self, text, color):
        self.stateLabel.setText(text)
        self.stateLabel.setStyleSheet('color:%s;' % color)

    def _engageClosedLoop(self):
        self.lockTimer.stop()
        self.send('src obs')
        self.send('loop v')
        self.send('%.3g' % self.speedValue)   # 裸数字=速度目标（'T'=力矩类型命令）
        self.state = 'run'
        self._setState('状态：③ 无感闭环中 @%g rad/s · 锁定✓' % self.speedValue, '#2e7d32')

    def onLine(self, line):
        if line.startswith('[SL CFG]'):
            body = line[len('[SL CFG]'):].strip()
            self.echoLabel.setText('固件回读：%s' % body)
            self.echoLabel.setStyleSheet('color:#2e7d32;')
            for token in body.split():
                if '=' not in token:
                    continue
                k, v = token.split('=', 1)
                if k == 'obs':
                    for key, btn in self.obsButtons.items():
                        btn.setChecked(key == v)
                    self.obsEcho.setText('观测器：%s' % v)
        elif line.startswith('[SL DBG]'):
            values = {}
            for token in line[len('[SL DBG]'):].strip().split():
                if '=' in token:
                    k, v = token.split('=', 1)
                    values[k] = v

            def fmt(v, spec):
                try:
                    return '—' if v is None else spec % float(v)
                except ValueError:
                    return '—'

            self.liveLabel.setText('the %s · tho %s · dth %s · 锁 %s' % (
                fmt(values.get('the'), '%.3f'), fmt(values.get('tho'), '%.3f'),
                fmt(values.get('dth'), '%+.3f'), values.get('lk', '—')))

            # 状态机喂养：到速才切（三条件连续达标）——低速盲区里 lk 会说谎，速度作证
            if self.state == 'vf_ramp':
                lkOk = values.get('lk') == '1'
                vfW = self._num(values.get('vfW'))
                wo = self._num(values.get('wo'))
                need = max(self.ENGAGE_SPEED_FLOOR,
                           0.9 * (self.speedValue or 0.0))
                speedOk = vfW >= need
                trackOk = vfW > 0.0 and wo >= 0.5 * vfW
                self.lkStreak = (self.lkStreak + 1
                                 if (lkOk and speedOk and trackOk) else 0)
                self._setState(
                    '状态：① VF 爬速 vfW=%.1f/需≥%.1f · 锁%s · 跟踪%s（wo=%.1f）· 达标 %d/%d'
                    % (vfW, need, '✓' if lkOk else '×', '✓' if trackOk else '×',
                       wo, self.lkStreak, self.LOCK_STREAK), '#e65100')
                if self.lkStreak >= self.LOCK_STREAK:
                    self._engageClosedLoop()
            elif self.state == 'run':
                locked = values.get('lk') == '1'
                self._setState(
                    '状态：③ 无感闭环中 @%g rad/s · %s' % (
                        self.speedValue if self.speedValue is not None else 0.0,
                        '锁定✓' if locked else '⚠失锁(lk=0)'),
                    '#2e7d32' if locked else '#c62828')


class SensorlessScope(QtWidgets.QGroupBox):
    """对账波形区：吃 [SL DBG] 探针行。

    - 上图「角度对账」：the 真值绿 / thc 控制链蓝 / tho 估计橙（左轴，rad）+
      dth 误差红虚线（右轴，rad）——三线合一是判据本体（谁在跟谁、差多少）；
    - 下图「速度/电」：we 真值绿虚 / wo 估计橙（左轴 rad/s）+ iq 灰（右轴 A）；
    - 时基=固件 ms 字段（首行归零；ms 回退=固件复位清屏，沿 ForceScope 纪律）；
    - 采样/重绘分离、暂停照常记录、MAX_POINTS 上限（同 ForceScope）。
    """

    MAX_POINTS = 6000
    CURVES = [
        ('the', 'θ_enc 真值 (rad)', '#43a047'),
        ('thc', 'θ_ctrl 控制链 (rad)', '#1e88e5'),
        ('tho', 'θ_obs 估计 (rad)', '#fb8c00'),
        ('dth', 'Δθ 误差 (rad)', '#e53935'),
        ('we', 'ω_enc (rad/s)', '#43a047'),
        ('wo', 'ω_obs (rad/s)', '#fb8c00'),
        ('iq', 'Iq (A)', '#9e9e9e'),
    ]

    def __init__(self, parent=None):
        super().__init__('无感对账波形（[SL DBG] 探针流 · 固件 ms 时基）', parent)
        self.device = SimpleFOCDevice.getInstance()
        self.bufT = []
        self.bufY = {key: [] for key, _n, _c in self.CURVES}
        self.bufMeta = []
        self.pendingSamples = []
        self.droppedRows = 0
        self.t0Ms = None
        self.lastMs = None
        self.paused = False
        self.arrivalStamps = []

        self.grid = QtWidgets.QGridLayout(self)

        # 上图：角度对账（the/thc/tho 左轴 + dth 右轴）
        self.plotAngle = pg.PlotWidget()
        pA = self.plotAngle.plotItem
        pA.setLabel('left', '角度 (rad)')
        pA.setLabel('bottom', 't (s)')
        self.plotAngle.showGrid(x=True, y=True, alpha=0.3)
        self.grid.addWidget(self.plotAngle, 0, 0, 1, 2)

        pA.showAxis('right')
        self.vbErr = pg.ViewBox()
        pA.scene().addItem(self.vbErr)
        pA.getAxis('right').linkToView(self.vbErr)
        self.vbErr.setXLink(pA)
        pA.getAxis('right').setLabel('Δθ 误差 (rad)')

        def syncErrView():
            self.vbErr.setGeometry(pA.vb.sceneBoundingRect())

        pA.vb.sigResized.connect(syncErrView)
        self.vbErr.enableAutoRange(x=False, y=True)

        # 下图：速度 + 电流
        self.plotSpeed = pg.PlotWidget()
        pS = self.plotSpeed.plotItem
        pS.setLabel('left', 'ω (rad/s)')
        pS.setLabel('bottom', 't (s)')
        self.plotSpeed.showGrid(x=True, y=True, alpha=0.3)
        self.grid.addWidget(self.plotSpeed, 1, 0, 1, 2)

        pS.showAxis('right')
        self.vbIq = pg.ViewBox()
        pS.scene().addItem(self.vbIq)
        pS.getAxis('right').linkToView(self.vbIq)
        self.vbIq.setXLink(pS)
        pS.getAxis('right').setLabel('Iq (A)')

        def syncIqView():
            self.vbIq.setGeometry(pS.vb.sceneBoundingRect())

        pS.vb.sigResized.connect(syncIqView)
        self.vbIq.enableAutoRange(x=False, y=True)
        self.plotAngle.setXLink(self.plotSpeed)

        self.curves = {}
        for key, name, color in self.CURVES:
            pen = pg.mkPen(color, width=2)
            if key == 'dth':
                pen = pg.mkPen(color, width=2, style=QtCore.Qt.DashLine)
            elif key == 'we':
                pen = pg.mkPen(color, width=2, style=QtCore.Qt.DashLine)
            if key in ('the', 'thc', 'tho'):
                curve = self.plotAngle.plot(pen=pen, name=name)
            elif key == 'dth':
                curve = pg.PlotDataItem(pen=pen)
                self.vbErr.addItem(curve)
            elif key in ('we', 'wo'):
                curve = self.plotSpeed.plot(pen=pen, name=name)
            else:
                curve = pg.PlotDataItem(pen=pen)
                self.vbIq.addItem(curve)
            self.curves[key] = curve

        # 控制行
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
        self.statusLabel = QtWidgets.QLabel('等待 [SL DBG] 探针行…')
        self.statusLabel.setStyleSheet('color:#888;')
        ctrlLayout.addWidget(self.statusLabel)
        self.grid.addWidget(ctrl, 2, 0, 1, 2)

        self.device.commProvider.commandDataReceived.connect(self.onLine)

        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.setInterval(50)
        self.redrawTimer.timeout.connect(self.drainAndRedraw)
        self.redrawTimer.start()

    def onLine(self, line):
        if not line.startswith('[SL DBG]'):
            return
        values = {}
        for token in line[len('[SL DBG]'):].strip().split():
            if '=' in token:
                k, v = token.split('=', 1)
                values[k] = v
        try:
            ms = float(values['ms'])
            row = {'ms': ms, 'src': values.get('src', ''), 'obs': values.get('obs', ''),
                   'vf': values.get('vf', ''), 'lk': values.get('lk', '')}
            for key in self.bufY:
                row[key] = float(values[key])
        except (KeyError, ValueError, TypeError):
            self.droppedRows += 1
            return
        self.pendingSamples.append(row)

    def drainAndRedraw(self):
        if not self.pendingSamples:
            self._refreshStatus()
            return
        rows = self.pendingSamples
        self.pendingSamples = []
        self.arrivalStamps.append(time.monotonic())
        if self.t0Ms is not None and rows[0]['ms'] < self.lastMs - 1000.0:
            trace('[SL SCOPE] firmware ms rewind: reset timebase')
            self._clearBuffers()
        for row in rows:
            if self.t0Ms is None:
                self.t0Ms = row['ms']
            self.lastMs = row['ms']
            self.bufT.append((row['ms'] - self.t0Ms) / 1000.0)
            self.bufMeta.append('%s/%s/vf=%s/lk=%s' % (row['src'], row['obs'], row['vf'], row['lk']))
            for key in self.bufY:
                self.bufY[key].append(row[key])
        overflow = len(self.bufT) - self.MAX_POINTS
        if overflow > 0:
            del self.bufT[:overflow]
            del self.bufMeta[:overflow]
            for key in self.bufY:
                del self.bufY[key][:overflow]
        if not self.paused:
            self.updatePlot()
        self._refreshStatus()

    def updatePlot(self):
        for key in self.bufY:
            self.curves[key].setData(self.bufT, self.bufY[key])
        self.vbErr.autoRange()
        self.vbIq.autoRange()

    def _clearBuffers(self):
        self.bufT.clear()
        self.bufMeta.clear()
        for key in self.bufY:
            self.bufY[key].clear()
        self.t0Ms = None
        self.lastMs = None
        for curve in self.curves.values():
            curve.setData([], [])

    def clearPlot(self):
        trace('[SL SCOPE] clear plot')
        self._clearBuffers()
        self.pendingSamples.clear()
        self.arrivalStamps.clear()

    def onPauseToggle(self):
        self.paused = not self.paused
        self.pauseButton.setText('继续' if self.paused else '暂停')

    def exportCsv(self):
        """导出对账缓冲（列含 src/obs/lk 元信息——离线判据按它们分段）。"""
        if not self.bufT:
            QtWidgets.QMessageBox.information(None, '导出CSV', '缓冲为空，先采一段再导出。')
            return
        defaultName = time.strftime('fockit_sensorless_%Y%m%d_%H%M%S.csv')
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            None, '导出无感对账 CSV', defaultName, 'CSV files (*.csv)')
        if not path:
            return
        # 路径安全收口（同 forceBench.exportCsv / Mimosa）：basename 落 exports/
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
        writer.writerow(['sample', 't_s', 'ms', 'meta'] +
                        [key for key, _n, _c in self.CURVES])
        for i in range(len(self.bufT)):
            writer.writerow([i, self.bufT[i], msList[i], self.bufMeta[i]] +
                            [self.bufY[key][i] for key, _n, _c in self.CURVES])
        Path(path).write_text(sio.getvalue(), encoding='utf-8-sig')
        trace('[SL SCOPE] CSV exported path=%r points=%d dropped=%d',
              path, len(self.bufT), self.droppedRows)
        QtWidgets.QMessageBox.information(
            None, '导出CSV', '已导出 %d 点 × 7 变量（丢弃行 %d）：\n%s' %
            (len(self.bufT), self.droppedRows, path))

    def _refreshStatus(self):
        now = time.monotonic()
        self.arrivalStamps = [t for t in self.arrivalStamps if now - t < 2.0]
        if not self.bufT:
            self.statusLabel.setText('等待 [SL DBG] 探针行…')
        else:
            self.statusLabel.setText('%.1f 行/s · %d 点 · 丢 %d' %
                                     (len(self.arrivalStamps) / 2.0,
                                      len(self.bufT), self.droppedRows))


class SensorlessBenchWidget(WorkAreaTabWidget):
    """无感台：连接条 + [SL DBG] 对账波形 + 无感控制卡（沿 forceBench 布局先例）。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        trace('[SL] SensorlessBenchWidget.__init__ enter')
        self.device = SimpleFOCDevice.getInstance()
        self.setObjectName('sensorlessBench')

        self.verticalLayout = QtWidgets.QVBoxLayout(self)
        self.connectionControl = ConnectionControlGroupBox()
        self.verticalLayout.addWidget(self.connectionControl)

        self.mainSplit = QtWidgets.QHBoxLayout()
        self.scope = SensorlessScope()
        self.mainSplit.addWidget(self.scope, 5)
        self.sideColumn = QtWidgets.QWidget()
        self.sideLayout = QtWidgets.QVBoxLayout(self.sideColumn)
        self.card = SensorlessCard(self.device)
        self.sideLayout.addWidget(self.card)
        self.sideLayout.addStretch(1)
        self.mainSplit.addWidget(self.sideColumn, 2)
        self.verticalLayout.addLayout(self.mainSplit, 1)

        trace('[SL] SensorlessBenchWidget.__init__ done')

    def getTabIcon(self):
        return GUIToolKit.getIconByName('purpledot')

    def getTabName(self):
        return '无感台'
