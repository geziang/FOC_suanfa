#!/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtWidgets

from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit, SerialPortComboBox
from src.simpleFOCConnector import SimpleFOCDevice
from src.debugTrace import trace, trace_exception


class ConnectionControlGroupBox(QtWidgets.QGroupBox):
    """FocKit 零配置连接区（2026-09-20 改造）。

    人机操作只剩两步：选端口 → 点连接。链路参数全部写死、不可改：
      - 命令前缀 'M'（simpleFOCConnector 构造默认，运行期无写点）
      - 115200-8N1（构造默认）
      - 连接模式 = 拉取（PULL_CONFIG_ON_CONNECT）
    历史包袱一并移除：命令ID 输入框（曾致前缀丢失、unknown cmd err 刷屏）、
    「设置」串口弹窗（校验位/停止位对本项目无意义）。
    上次使用的端口记在 QSettings（本机记忆，不进 device.json），启动自动回选。
    """

    SETTINGS_ORG = 'FocKit'
    SETTINGS_APP = 'SimpleFOCStudio'
    LAST_PORT_KEY = 'lastPort'

    def __init__(self, parent=None):
        super().__init__(parent)
        trace('[UI] ConnectionControlGroupBox.__init__ enter')

        self.device = SimpleFOCDevice.getInstance()

        self.setObjectName('connectionControl')
        self.setTitle('连接（FocKit 绑定：前缀 M / 115200-8N1 / 拉取模式）')

        self.horizontalLayout = QtWidgets.QHBoxLayout(self)
        self.horizontalLayout.setObjectName('generalControlHL')

        self.portNameLabel = QtWidgets.QLabel('端口:')
        self.portNameLabel.setToolTip(
            '点击下拉刷新列表；板卡通电、驱动就绪后端口会自动出现。\n'
            '上次用过的端口会自动回选。')
        self.horizontalLayout.addWidget(self.portNameLabel)

        self.portNameComboBox = SerialPortComboBox(self)
        self.portNameComboBox.setObjectName('portNameComboBox')
        self.portNameComboBox.setMinimumWidth(140)
        lastPort = self._readLastPort()
        if lastPort:
            self.portNameComboBox.setCurrentText(lastPort)
        self.horizontalLayout.addWidget(self.portNameComboBox)

        self.pullConfig = QtWidgets.QPushButton()
        self.pullConfig.setObjectName('pullConfig')
        self.pullConfig.setIcon(GUIToolKit.getIconByName('pull'))
        self.pullConfig.setText('获取参数')
        self.pullConfig.clicked.connect(self.device.pullConfiguration)

        self.horizontalLayout.addWidget(self.pullConfig)

        self.connectDisconnectButton = QtWidgets.QPushButton(self)
        self.connectDisconnectButton.setIcon(GUIToolKit.getIconByName('connect'))
        self.connectDisconnectButton.setObjectName('connectDeviceButton')
        self.connectDisconnectButton.setText('连接')
        self.connectDisconnectButton.clicked.connect(self.connectDisconnectDeviceAction)

        self.horizontalLayout.addWidget(self.connectDisconnectButton)

        self.device.addConnectionStateListener(self)
        self.connectionStateChanged(self.device.isConnected)
        trace('[UI] ConnectionControlGroupBox.__init__ done')

    @staticmethod
    def _readLastPort():
        settings = QtCore.QSettings(ConnectionControlGroupBox.SETTINGS_ORG,
                                    ConnectionControlGroupBox.SETTINGS_APP)
        return str(settings.value(ConnectionControlGroupBox.LAST_PORT_KEY, ''))

    @staticmethod
    def _writeLastPort(port):
        settings = QtCore.QSettings(ConnectionControlGroupBox.SETTINGS_ORG,
                                    ConnectionControlGroupBox.SETTINGS_APP)
        settings.setValue(ConnectionControlGroupBox.LAST_PORT_KEY, port)

    def connectDisconnectDeviceAction(self):
        trace('[UI] connect button clicked connected=%r', self.device.isConnected)
        if self.device.isConnected:
            self.device.disConnect()
            return
        port = self.portNameComboBox.currentText().strip()
        if not port:
            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Warning)
            msgBox.setText('请先在下拉框中选择串口（列表为空时：确认板卡已通电、'
                           '驱动已装好，再点开下拉刷新）。')
            msgBox.setWindowTitle('SimpleFOC ConfigTool')
            msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
            msgBox.exec_()
            return
        self._writeLastPort(port)
        self.device.serialPortName = port
        self.device.connect(SimpleFOCDevice.PULL_CONFIG_ON_CONNECT)

    def connectionStateChanged(self, isConnected):
        trace('[UI] connectionStateChanged connected=%r', isConnected)
        if isConnected:
            self.connectDisconnectButton.setIcon(
                GUIToolKit.getIconByName('disconnect'))
            self.connectDisconnectButton.setText('断开')
        else:
            self.connectDisconnectButton.setIcon(
                GUIToolKit.getIconByName('connect'))
            self.connectDisconnectButton.setText('连接')
