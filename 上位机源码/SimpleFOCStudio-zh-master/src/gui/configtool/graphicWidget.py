#!/usr/bin/env python
# -*- coding: utf-8 -*-
import collections
import csv
import logging
import time

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets

from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit
from src.simpleFOCConnector import SimpleFOCDevice
from src.debugTrace import trace, trace_exception, trace_verbose


class SimpleFOCGraphicWidget(QtWidgets.QGroupBox):
    """实时曲线：限频 + 合并重绘（2026-09-20 改造，收口见 REF-12）。

    收数与重绘解耦：串口行回调只校验 + 入队（有界 deque），QTimer 每帧
    把积压批量搬入显示数组并统一重绘一次。成本从"行率×全量重绘"降为
    "重绘率×全量重绘"，界面事件队列不再被逐行小事件淹没（此前降采样
    100×7 变量即冻结的根因）。实时性取决于串口收数与缓冲，与屏幕刷新
    率无关——20Hz 对人眼已足够，60Hz 是上限，再高只是浪费 CPU。
    半行/坏行（断开冲刷的残行等）在入队前整行丢弃，绝不进缓冲。
    """

    disconnectedState = 0
    initialConnectedState = 1
    connectedPausedState = 2
    connectedPlottingStartedState = 3

    PLOT_FRAME_MS = 50     # 重绘帧周期：50ms=20Hz。勿再调小，见类 docstring
    PENDING_MAXLEN = 2048  # 串口行积压上限：GUI 偶发停顿时丢旧保新，内存有界

    MEASURE_SECONDS = 2.0  # 阶跃判据测量窗时长（按实测流率换算点数）
    MEASURE_BAND = 0.02    # 建立时间误差带：±2%（相对步幅）

    
    signals = ['Target', 'Vq','Vd','Cq','Cd','Vel','Angle']
    signal_tooltip = ['目标', '电压 Q [V]','电压 D [V]','电流 Q [mA]','电流 D [mA]','速度 [rad/sec]','角度 [rad]']
    signalColors = [GUIToolKit.RED_COLOR, GUIToolKit.BLUE_COLOR, GUIToolKit.PURPLE_COLOR,GUIToolKit.YELLOW_COLOR, GUIToolKit.MAROON_COLOR, GUIToolKit.ORANGE_COLOR, GUIToolKit.GREEN_COLOR]
    signalIcons = ['reddot', 'bluedot','purpledot', 'yellowdot', 'maroondot', 'orangedot', 'greendot']

    def __init__(self, parent=None):

        super().__init__(parent)
        trace('[UI] SimpleFOCGraphicWidget.__init__ enter')

        self.setObjectName('plotWidget')
        self.setTitle('实时电机数据: ')
        self.horizontalLayout = QtWidgets.QVBoxLayout(self)
        self.device = SimpleFOCDevice.getInstance()

        # 2026-09-20：300→1000 点。20Hz 重绘下全量 setData 成本可忽略，
        # 换来更长的时间窗（覆盖 2~4 秒的阶跃整拍，SPEC-T 判据需要）
        self.numberOfSamples = 1000

        # 抗锯齿是全量重绘成本的放大器，关掉换帧率
        pg.setConfigOptions(antialias=False)
        self.plotWidget = pg.PlotWidget()
        trace('[UI] plot widget created; pyqtgraph=%s', getattr(pg, '__version__', '?'))
        self.plotWidget.showGrid(x=True, y=True, alpha=0.5)
        self.plotWidget.addLegend()

        # self.legend = pg.LegendItem()
        # self.legend.setParentItem(self.plotWidget)

        self.timeArray = np.arange(-self.numberOfSamples, 0, 1)
        
        self.controlPlotWidget = ControlPlotPanel(controllerPlotWidget=self)

        self.signalDataArrays = []
        self.signalPlots = []
        self.signalPlotFlags = []
        for (sig, sigColor, checkBox, tooltip) in zip(self.signals, self.signalColors,self.controlPlotWidget.signalCheckBox, self.signal_tooltip):
            # define signal plot data array
            self.signalDataArrays.append(np.zeros(self.numberOfSamples))
            # configure signal plot parameters
            signalPen = pg.mkPen(color=sigColor, width=1.5)
            self.signalPlots.append(pg.PlotDataItem(self.timeArray,
                                            self.signalDataArrays[-1],
                                            pen=signalPen, name=tooltip))
            self.plotWidget.addItem(self.signalPlots[-1])

            # is plotted flag
            self.signalPlotFlags.append(True)
            # add callback
            checkBox.stateChanged.connect(self.signalPlotFlagUpdate)


        self.horizontalLayout.addWidget(self.plotWidget)
        self.horizontalLayout.addWidget(self.controlPlotWidget)
        
        self.device.commProvider.monitoringDataReceived.connect(
            self.upDateGraphic)

        # 收数/重绘解耦的缓冲与重绘定时器（见类 docstring）。
        # 定时器常开：非绘图态下行回调直接丢弃、队列为空，帧回调为空操作。
        self.pendingSamples = collections.deque(maxlen=self.PENDING_MAXLEN)
        self.droppedRows = 0
        self._enabledIndices = np.where(
            np.array(self.signalPlotFlags) == True)[0]
        self.plotTimer = QtCore.QTimer(self)
        self.plotTimer.setInterval(self.PLOT_FRAME_MS)
        self.plotTimer.timeout.connect(self.drainAndRedraw)
        self.plotTimer.start()

        # 视图缩放系数（2026-09-20）：≠1 的轴在重绘时按"跟随数据"的定幅视图套用，
        # =1 保持原自动范围行为；由面板 X×/Y× 输入框驱动，见 setViewScale
        self.viewXFactor = 1.0
        self.viewYFactor = 1.0

        # ── 秒轴（2026-09-21）：实测流率自动校准点间隔 ──
        # 每条曲线行到达时记录时间戳（滚动窗），点间隔 = 窗内跨度/点数；
        # 降采样/主循环变化会在 ~2 秒内自动重校准；流中断 >0.5s 清空重校。
        self.arrivalTimes = collections.deque(maxlen=256)
        self.sampleInterval = None   # [s/点]；None=未校准（X 轴暂以点序号显示）
        self._xLabelMode = None

        # ── 阶跃判据测量（tr/σ%/ts/ess，2026-09-21）：整定台阶跃按钮触发 ──
        self.measure = None          # 进行中的测量窗（None=空闲）
        self.lastMetrics = None
        self.prevMetrics = None
        self._measureSeq = 0
        # 标注线（Y 值锚定，不随数据滚动失效）：目标 + ±band 误差带
        bandPen = pg.mkPen((110, 110, 110, 190), style=QtCore.Qt.DashLine)
        targetPen = pg.mkPen((200, 40, 40, 200), style=QtCore.Qt.DashLine)
        self.targetLine = pg.InfiniteLine(angle=0, pen=targetPen)
        self.bandHiLine = pg.InfiniteLine(angle=0, pen=bandPen)
        self.bandLoLine = pg.InfiniteLine(angle=0, pen=bandPen)
        for line in (self.targetLine, self.bandHiLine, self.bandLoLine):
            line.hide()
            self.plotWidget.addItem(line)

        self.currentStatus = self.disconnectedState
        self.controlPlotWidget.pauseContinueButton.setDisabled(True)

        self.device.addConnectionStateListener(self)

        self.connectionStateChanged(self.device.isConnected)
        trace('[UI] SimpleFOCGraphicWidget.__init__ done')

    def connectionStateChanged(self, deviceConnected):
        trace('[UI] graphic connectionStateChanged connected=%r', deviceConnected)
        if deviceConnected is True:
            self.currentStatus = self.initialConnectedState
            self.enabeUI()
        else:
            self.controlPlotWidget.startStoPlotAction()
            self.controlPlotWidget.stopAndResetPlot()
            self.currentStatus = self.disconnectedState
            self.disableUI()

    def enabeUI(self):
        self.setEnabled(True)

    def disableUI(self):
        self.setEnabled(False)

    def signalPlotFlagUpdate(self):
        trace('[UI] signalPlotFlagUpdate enter flags=%r', [box.isChecked() for box in self.controlPlotWidget.signalCheckBox])
        self.controlPlotWidget.updateMonitorVariables()
        for i, (checkBox, plotFlag) in enumerate(zip(self.controlPlotWidget.signalCheckBox, self.signalPlotFlags)):
            if checkBox.isChecked() and (not plotFlag):
                self.signalPlotFlags[i] = True
                self.plotWidget.addItem( self.signalPlots[i] )
            elif (not checkBox.isChecked()) and plotFlag:
                self.signalPlotFlags[i]  = False
                self.plotWidget.removeItem( self.signalPlots[i] )
        # 勾选变化后刷新列数缓存：入队校验按它对列数
        # （勾选变化会经 updateMonitorVariables 同步重发 MMS 位图，两端保持一致）
        self._enabledIndices = np.where(np.array(self.signalPlotFlags) == True)[0]

    def connectioStatusUpdate(self, connectedFlag):
        if connectedFlag:
            self.currentStatus = self.initialConnectedState
        else:
            self.currentStatus = self.disconnectedState

    def upDateGraphic(self, signalList):
        """串口行回调（RX 线程投递到 GUI 线程）：只校验 + 入队，不做任何绘图。"""
        trace_verbose('[PLOT RX] signalList=%r status=%r', signalList, self.currentStatus)
        if self.currentStatus is not self.connectedPlottingStartedState and \
                self.currentStatus is not self.connectedPausedState:
            return
        try:
            signals = np.array(signalList, dtype=float)
        except (ValueError, TypeError):
            self.droppedRows += 1  # 非数值字段：整行丢弃
            return
        if signals.ndim != 1 or signals.size != len(self._enabledIndices):
            self.droppedRows += 1  # 半行/列数与勾选数不符：整行丢弃
            trace_verbose('[PLOT] row dropped (total=%d)', self.droppedRows)
            return
        row = tuple(float(v) for v in signals)
        self.pendingSamples.append(row)

        # 秒轴校准：记录到达时刻；流中断/重启（间隙>0.5s）清空重校
        now = time.monotonic()
        if self.arrivalTimes and now - self.arrivalTimes[-1] > 0.5:
            self.arrivalTimes.clear()
        self.arrivalTimes.append(now)

        # 判据测量窗：采集响应通道列，满窗自动结算
        if self.measure is not None:
            self.measure['buf'].append(row[self.measure['col']])
            if len(self.measure['buf']) >= self._measureWindow():
                self._finalizeMeasure()

    def drainAndRedraw(self):
        """QTimer 帧回调：积压批量搬入显示数组（每帧一次移位），统一重绘一次。

        暂停态照常填充数组（可暂停后导出整段），只是不重绘。
        """
        # 秒轴校准（EMA 平滑，滚动窗内自适应降采样/主循环变化）——
        # 放在积压早退之前：无新数据帧也要持续校准
        if len(self.arrivalTimes) >= 10:
            span = self.arrivalTimes[-1] - self.arrivalTimes[0]
            if span > 0:
                fresh = span / (len(self.arrivalTimes) - 1)
                self.sampleInterval = fresh if self.sampleInterval is None \
                    else 0.7 * self.sampleInterval + 0.3 * fresh
        k = len(self.pendingSamples)
        if k == 0:
            return
        rows = np.array(self.pendingSamples)
        self.pendingSamples.clear()
        n = self.numberOfSamples
        m = min(k, n)
        for i, ind in enumerate(self._enabledIndices):
            arr = self.signalDataArrays[ind]
            if m < n:
                arr[:n - m] = arr[m:]
            arr[n - m:] = rows[:, i][-m:]
        if self.currentStatus is self.connectedPlottingStartedState:
            self.updatePlot()

    def exportCsv(self):
        """导出当前显示缓冲为 CSV（显示与记录分离：导的是显示缓冲，非全量流）。"""
        enabled = [int(i) for i in self._enabledIndices]
        if not enabled:
            QtWidgets.QMessageBox.information(
                None, '导出CSV', '没有勾选任何变量，先勾选再导出。')
            return
        defaultName = time.strftime('fockit_plot_%Y%m%d_%H%M%S.csv')
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            None, '导出曲线缓冲 CSV', defaultName, 'CSV files (*.csv)')
        if not path:
            return
        headers = ['sample', 'time_s'] + [self.signals[i] for i in enabled]
        columns = [self.signalDataArrays[i] for i in enabled]
        with open(path, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            for r in range(self.numberOfSamples):
                ts = ((r - self.numberOfSamples) * self.sampleInterval
                      if self.sampleInterval else '')
                writer.writerow([r - self.numberOfSamples, ts] +
                                [float(c[r]) for c in columns])
        trace('[PLOT] CSV exported path=%r points=%d vars=%d dropped=%d',
              path, self.numberOfSamples, len(enabled), self.droppedRows)
        QtWidgets.QMessageBox.information(
            None, '导出CSV',
            '已导出 %d 点 × %d 变量（丢弃行 %d）：\n%s' %
            (self.numberOfSamples, len(enabled), self.droppedRows, path))


    def computeStatic(self, array):
        mean = np.mean(array)
        std = np.std(array)
        max = np.max(array)
        min = np.min(array)
        meadian = np.median(array)

    def setViewScale(self, xFactor, yFactor):
        """视图缩放系数：>1 放大看细节，<1 拉远，=1 恢复该轴自动范围。

        X：窗口 = 全窗/系数，右端始终锚定最新数据（持续跟新的细节窗）；
        Y：可见窗口内启用通道的数据包络 / 系数。系数在 updatePlot 里逐帧套用。
        """
        self.viewXFactor = float(xFactor)
        self.viewYFactor = float(yFactor)
        vb = self.plotWidget.getViewBox()
        vb.enableAutoRange(x=(self.viewXFactor == 1.0),
                           y=(self.viewYFactor == 1.0))
        if self.viewXFactor == 1.0 or self.viewYFactor == 1.0:
            # 回归自动的轴立即拟合一次：静态数据下 setData 不会触发重算
            vb.autoRange()

    # ── 阶跃判据测量（tr/σ%/ts/ess，2026-09-21）─────────────
    def beginStepMeasure(self, yRef, channel):
        """整定台阶跃按钮触发：开始一次阶跃响应判据测量窗。

        yRef 为**曲线坐标**的目标值（调用方注意通道单位，如 Cq/Cd 是 mA）。
        响应通道必须已勾选（未勾选则放弃测量并记日志）。
        """
        enabled = [int(i) for i in self._enabledIndices]
        if channel not in enabled:
            trace('[PLOT] step measure skipped: channel %d not plotted', channel)
            self.measure = None
            return
        if self.measure is not None:
            self._finalizeMeasure()  # 上一次未满窗，先用手头数据出结果
        self.measure = {
            'y_ref': float(yRef), 'ch': int(channel),
            'col': enabled.index(channel),
            'y0': float(self.signalDataArrays[channel][-1]),
            'buf': []}
        trace('[PLOT] step measure begin y_ref=%r ch=%d', yRef, channel)

    def _measureWindow(self):
        """测量窗点数：MEASURE_SECONDS 按实测点间隔换算，未校准用保守 300 点。"""
        if self.sampleInterval and self.sampleInterval > 0:
            return int(max(50, min(self.MEASURE_SECONDS / self.sampleInterval,
                                   600)))
        return 300

    def _finalizeMeasure(self):
        """结算测量窗：归一化 p=(y-y0)/步幅 → tr(10%→90%) / σ% / ts(±band) / ess。"""
        m, self.measure = self.measure, None
        self._measureSeq += 1
        result = {'id': self._measureSeq, 'valid': False, 'tr': None,
                  'sigma': None, 'ts': None, 'ess': None}
        self.prevMetrics, self.lastMetrics = self.lastMetrics, result
        if m is None:
            return
        y = np.array(m['buf'], dtype=float)
        y0, yr = m['y0'], m['y_ref']
        step = yr - y0
        if len(y) < 10 or abs(step) < 1e-9:
            trace('[PLOT] step metrics invalid (n=%d step=%.4g)', len(y), step)
            return
        dt = self.sampleInterval
        p = (y - y0) / step                     # 归一化：1.0 = 到达目标
        band = self.MEASURE_BAND
        hit10 = np.where(p >= 0.1)[0]
        hit90 = np.where(p >= 0.9)[0]
        outside = np.where(np.abs(p - 1.0) > band)[0]
        tail = y[int(len(y) * 0.8):]
        result.update(
            valid=True,
            tr=(float(hit90[0] - hit10[0]) * dt)
                if (len(hit10) and len(hit90) and dt) else None,
            sigma=max(float(p.max()) - 1.0, 0.0) * 100.0,
            ts=(float(outside[-1] + 1) * dt) if dt else None,
            ess=float(np.mean(tail)) - yr)
        # 标注线（Y 值锚定，不随数据滚动失效）：目标 + ±band 误差带
        stepAbs = abs(step)
        self.targetLine.setPos(yr)
        self.bandHiLine.setPos(yr + band * stepAbs)
        self.bandLoLine.setPos(yr - band * stepAbs)
        for line in (self.targetLine, self.bandHiLine, self.bandLoLine):
            line.show()
        trace('[PLOT] step metrics %r', result)

    def updatePlot(self):
        trace_verbose('[PLOT] updatePlot enter enabled=%r', self.signalPlotFlags)
        # X 坐标：已校准 → 秒（timeArray×间隔），未校准 → 点序号
        interval = self.sampleInterval
        xs = self.timeArray if interval is None else self.timeArray * interval
        mode = 's' if interval is not None else 'pts'
        if mode != self._xLabelMode:
            self._xLabelMode = mode
            if interval is not None:
                self.plotWidget.setLabel('bottom', '时间', units='s')
            else:
                self.plotWidget.setLabel('bottom', '样本点')
        for i, plotFlag in enumerate(self.signalPlotFlags):
            if plotFlag:
                self.signalPlots[i].setData(xs, self.signalDataArrays[i])
        # 视图缩放：仅对系数≠1 的轴套用（=1 轴维持自动范围，原行为不变）
        if self.viewXFactor != 1.0 or self.viewYFactor != 1.0:
            dt = interval if interval is not None else 1.0
            visible = max(10, int(self.numberOfSamples / self.viewXFactor))
            if self.viewXFactor != 1.0:
                self.plotWidget.setXRange(-visible * dt, 0, padding=0)
            if self.viewYFactor != 1.0:
                enabled = [int(i) for i in self._enabledIndices]
                if enabled:
                    window = [self.signalDataArrays[i][-visible:]
                              for i in enabled]
                    lo = min(col.min() for col in window)
                    hi = max(col.max() for col in window)
                    center = (lo + hi) / 2.0
                    half = max((hi - lo) / 2.0, abs(center) * 0.05 + 1e-6) \
                        / self.viewYFactor
                    self.plotWidget.setYRange(center - half, center + half,
                                              padding=0)


class ControlPlotPanel(QtWidgets.QWidget):

    def __init__(self, parent=None, controllerPlotWidget=None):
        '''Constructor for ToolsWidget'''
        super().__init__(parent)
        trace('[UI] ControlPlotPanel.__init__ enter')

        self.device = SimpleFOCDevice.getInstance()
        self.controlledPlot = controllerPlotWidget
        
        self.verticalLayout = QtWidgets.QVBoxLayout(self)
        self.setLayout(self.verticalLayout)

        self.horizontalLayout1 = QtWidgets.QHBoxLayout()
        self.horizontalLayout1.setObjectName('horizontalLayout')

        self.startStopButton = QtWidgets.QPushButton(self)
        self.startStopButton.setText('开始')
        self.startStopButton.setObjectName('Start')
        self.startStopButton.clicked.connect(self.startStoPlotAction)
        self.startStopButton.setIcon(GUIToolKit.getIconByName('start'))
        self.horizontalLayout1.addWidget(self.startStopButton)

        self.pauseContinueButton = QtWidgets.QPushButton(self)
        self.pauseContinueButton.setObjectName('pauseButton')
        self.pauseContinueButton.setText('暂停')
        self.pauseContinueButton.setIcon(GUIToolKit.getIconByName('pause'))
        self.pauseContinueButton.clicked.connect(self.pauseContinuePlotAction)
        self.horizontalLayout1.addWidget(self.pauseContinueButton)

        self.zoomAllButton = QtWidgets.QPushButton(self)
        self.zoomAllButton.setObjectName('zoomAllButton')
        self.zoomAllButton.setText('显示所有')
        self.zoomAllButton.setIcon(GUIToolKit.getIconByName('zoomall'))
        self.zoomAllButton.clicked.connect(self.zoomAllPlot)
        self.horizontalLayout1.addWidget(self.zoomAllButton)

        self.exportCsvButton = QtWidgets.QPushButton(self)
        self.exportCsvButton.setObjectName('exportCsvButton')
        self.exportCsvButton.setText('导出CSV')
        self.exportCsvButton.setIcon(GUIToolKit.getIconByName('save'))
        self.exportCsvButton.setToolTip(
            '把当前显示缓冲（1000 点）存为 CSV。\n'
            '暂停态下缓冲仍在填充：可先暂停冻结画面，再导出整段。')
        self.exportCsvButton.clicked.connect(self.exportCsvAction)
        self.horizontalLayout1.addWidget(self.exportCsvButton)

        self.signalCheckBox = []
        for i in range(len(self.controlledPlot.signals)):
            checkBox = QtWidgets.QCheckBox(self)
            checkBox.setObjectName('signalCheckBox'+str(i))
            checkBox.setToolTip(self.controlledPlot.signal_tooltip[i])
            checkBox.setText(self.controlledPlot.signals[i])
            checkBox.setIcon(GUIToolKit.getIconByName(self.controlledPlot.signalIcons[i]))
            checkBox.setChecked(True)
            self.signalCheckBox.append(checkBox)
            self.horizontalLayout1.addWidget(checkBox)


        spacerItem = QtWidgets.QSpacerItem(100, 20,
                                           QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Maximum)

        self.horizontalLayout1.addItem(spacerItem)
        self.horizontalLayout1.addItem(spacerItem)

        self.downsampleLabel = QtWidgets.QLabel(self)
        self.downsampleLabel.setText('降采样')
        self.downampleValue = QtWidgets.QLineEdit(self.downsampleLabel)
        self.downampleValue.setText("100")
        self.downampleValue.editingFinished.connect(self.changeDownsampling)
        self.horizontalLayout1.addWidget(self.downsampleLabel)
        self.horizontalLayout1.addWidget(self.downampleValue)

        # 视图缩放（2026-09-20）：X/Y 倍数，回车生效；>1 放大看细节，<1 拉远，
        # 1 自动（原行为）。「显示所有」复位两系数。
        self.xScaleEdit = QtWidgets.QLineEdit(self)
        self.xScaleEdit.setText('1')
        self.xScaleEdit.setMaximumWidth(42)
        self.xScaleEdit.setToolTip(
            'X 倍数：>1 放大（窗口=全窗/倍数，右端锁定最新点），<1 拉远，1 自动。\n'
            '例：填 4 只看最近 250 点，配合 Y 倍数观察波形细节。')
        self.xScaleEdit.editingFinished.connect(self.changeViewScale)
        self.yScaleEdit = QtWidgets.QLineEdit(self)
        self.yScaleEdit.setText('1')
        self.yScaleEdit.setMaximumWidth(42)
        self.yScaleEdit.setToolTip(
            'Y 倍数：>1 放大可见窗口的数据包络，<1 拉远，1 自动。')
        self.yScaleEdit.editingFinished.connect(self.changeViewScale)
        self.horizontalLayout1.addWidget(QtWidgets.QLabel('视图缩放'))
        self.horizontalLayout1.addWidget(QtWidgets.QLabel('X×'))
        self.horizontalLayout1.addWidget(self.xScaleEdit)
        self.horizontalLayout1.addWidget(QtWidgets.QLabel('Y×'))
        self.horizontalLayout1.addWidget(self.yScaleEdit)

        self.verticalLayout.addLayout(self.horizontalLayout1)
        trace('[UI] ControlPlotPanel.__init__ done')

    def startStoPlotAction(self):
        trace('[UI] plot start/stop clicked currentStatus=%r downsample=%r', self.controlledPlot.currentStatus, self.downampleValue.text())
        if self.controlledPlot.currentStatus is self.controlledPlot.initialConnectedState:
            # Start pressed
            self.startStopButton.setText('停止')
            self.startStopButton.setIcon(GUIToolKit.getIconByName('stop'))
            self.controlledPlot.currentStatus = \
                self.controlledPlot.connectedPlottingStartedState
            self.pauseContinueButton.setEnabled(True)
            self.device.sendMonitorDownsample(int(self.downampleValue.text()))
            self.updateMonitorVariables()
        else:
            # Stop pressed
            self.startStopButton.setText('开始')
            self.startStopButton.setIcon(GUIToolKit.getIconByName('start'))
            self.pauseContinueButton.setText('暂停')
            self.pauseContinueButton.setIcon(GUIToolKit.getIconByName('pause'))
            self.pauseContinueButton.setEnabled(False)
            self.stopAndResetPlot()
            self.device.sendMonitorDownsample(0)
            self.device.sendMonitorClearVariables()

    def pauseContinuePlotAction(self):
        trace('[UI] plot pause/continue clicked currentStatus=%r', self.controlledPlot.currentStatus)
        if self.controlledPlot.currentStatus is self.controlledPlot.connectedPausedState:
            # Continue pressed
            self.pauseContinueButton.setText('停止')
            self.pauseContinueButton.setIcon(GUIToolKit.getIconByName('pause'))
            self.controlledPlot.currentStatus = self.controlledPlot.connectedPlottingStartedState
        else:
            # Pause pressed
            self.pauseContinueButton.setText('继续')
            self.pauseContinueButton.setIcon(
                GUIToolKit.getIconByName('continue'))
            self.controlledPlot.currentStatus = self.controlledPlot.connectedPausedState

    def stopAndResetPlot(self):
        trace('[UI] stopAndResetPlot enter')
        self.controlledPlot.currentStatus = self.controlledPlot.initialConnectedState
        for dataArray in self.controlledPlot.signalDataArrays:
            dataArray = np.zeros(self.controlledPlot.numberOfSamples)

    def zoomAllPlot(self):
        trace('[UI] zoomAllPlot clicked')
        # 显示所有 = 复位视图缩放系数并恢复双轴自动范围
        self.controlledPlot.setViewScale(1.0, 1.0)
        self.xScaleEdit.setText('1')
        self.yScaleEdit.setText('1')
        self.controlledPlot.plotWidget.enableAutoRange()

    def changeViewScale(self):
        try:
            xFactor = float(self.xScaleEdit.text())
            yFactor = float(self.yScaleEdit.text())
            if xFactor <= 0 or yFactor <= 0:
                raise ValueError
        except ValueError:
            self.xScaleEdit.setText('1')
            self.yScaleEdit.setText('1')
            xFactor = yFactor = 1.0
        trace('[UI] view scale changed x=%r y=%r', xFactor, yFactor)
        self.controlledPlot.setViewScale(xFactor, yFactor)

    def exportCsvAction(self):
        trace('[UI] export CSV clicked')
        self.controlledPlot.exportCsv()

    def changeDownsampling(self):
        trace('[UI] changeDownsampling value=%r status=%r', self.downampleValue.text(), self.controlledPlot.currentStatus)
        if  self.controlledPlot.currentStatus == self.controlledPlot.connectedPlottingStartedState:
            self.device.sendMonitorDownsample(int(self.downampleValue.text()))

    def updateMonitorVariables(self):
        trace('[UI] updateMonitorVariables flags=%r', [box.isChecked() for box in self.signalCheckBox])
        self.device.sendMonitorVariables([self.signalCheckBox[0].isChecked(), 
                                                self.signalCheckBox[1].isChecked(),
                                                 self.signalCheckBox[2].isChecked(), 
                                                 self.signalCheckBox[3].isChecked(), 
                                                 self.signalCheckBox[4].isChecked(), 
                                                 self.signalCheckBox[5].isChecked(), 
                                                 self.signalCheckBox[6].isChecked()])
