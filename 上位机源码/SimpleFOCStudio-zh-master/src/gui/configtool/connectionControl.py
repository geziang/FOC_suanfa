#!/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets

from src.gui.configtool.configureConnectionDialog import \
    ConfigureSerailConnectionDialog
from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit
from src.simpleFOCConnector import SimpleFOCDevice
from src.debugTrace import trace, trace_exception


class ConnectionControlGroupBox(QtWidgets.QGroupBox):

    def __init__(self, parent=None):
        super().__init__(parent)
        trace('[UI] ConnectionControlGroupBox.__init__ enter')

        self.device = SimpleFOCDevice.getInstance()

        self.setObjectName('connectionControl')
        self.setTitle('连接')

        self.horizontalLayout = QtWidgets.QHBoxLayout(self)
        self.horizontalLayout.setObjectName('generalControlHL')

        self.devCommandIDLabel = QtWidgets.QLabel("命令ID:")
        self.devCommandIDLabel.setToolTip(
            '真正生效的设备命令ID —— 下行命令会拼成 <命令ID><命令体>。\n'
            '本固件注册为 M，故须填 M。\n'
            '注意：「设置」弹窗里的那个“连接ID（不下发）”只在 device.json 里留档，与本框无关。')
        self.horizontalLayout.addWidget(self.devCommandIDLabel)

        self.devCommandIDLetter = QtWidgets.QLineEdit()
        self.devCommandIDLetter.setObjectName('devCommandIDLetter')
        self.devCommandIDLetter.setMaxLength(1)          # 设备 ID 是单个字符，防止误填 "MM"
        self.devCommandIDLetter.setPlaceholderText('M')
        self.devCommandIDLetter.setToolTip(
            '须填 M。\n'
            '改完立即生效（textChanged 提交，无需回车）。\n'
            '本框不落盘：重启 Studio 后用「文件 → 打开设备」载入 device.json 才会自动恢复。')
        # 双重提交：textChanged 覆盖"打了字但没回车"；editingFinished 保留兼容
        self.devCommandIDLetter.textChanged.connect(self.changeDevicedevCommandID)
        self.devCommandIDLetter.editingFinished.connect(self.changeDevicedevCommandID)
        self.horizontalLayout.addWidget(self.devCommandIDLetter)
        self.devCommandIDLetter.setText(self.device.devCommandID)

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

        self.configureDeviceButton = QtWidgets.QPushButton(self)
        self.configureDeviceButton.setIcon(GUIToolKit.getIconByName('configure'))
        self.configureDeviceButton.setObjectName('configureDeviceButton')
        self.configureDeviceButton.setText('设置')
        self.configureDeviceButton.clicked.connect(self.configureDeviceAction)
        self.horizontalLayout.addWidget(self.configureDeviceButton)

        self.device.addConnectionStateListener(self)
        self.connectionStateChanged(self.device.isConnected)
        trace('[UI] ConnectionControlGroupBox.__init__ done')
    
    def changeDevicedevCommandID(self):
        # 去掉误输入的空白（单字符设备 ID，带空格会让下行命令前缀错位）
        value = self.devCommandIDLetter.text().strip()
        if value != self.devCommandIDLetter.text():
            # setText 会再触发一次 textChanged，但那时已相等 → 不会再进这里，无递归
            self.devCommandIDLetter.setText(value)
        self.device.devCommandID = value
        trace('[UI] command ID committed value=%r', value)

    def connectDisconnectDeviceAction(self):
        trace('[UI] connect button clicked connected=%r', self.device.isConnected)
        if self.device.isConnected:
            self.device.disConnect()
        else:
            connectionMode  = SimpleFOCDevice.PULL_CONFIG_ON_CONNECT
            self.device.connect(connectionMode)

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

    def configureDeviceAction(self):
        trace('[UI] serial settings button clicked')
        dialog = ConfigureSerailConnectionDialog()
        trace('[UI] serial settings dialog created; exec begin')
        result = dialog.exec_()
        trace('[UI] serial settings dialog exec returned result=%r', result)
        if result:
            deviceConfig = dialog.getConfigValues()
            trace('[UI] serial settings accepted config=%r', deviceConfig)
            self.device.configureConnection(deviceConfig)
