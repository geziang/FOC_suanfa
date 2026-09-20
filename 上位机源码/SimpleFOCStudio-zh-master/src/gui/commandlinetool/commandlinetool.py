#!/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets

from src.gui.commandlinetool.configureConnectionWidget import \
    ConfigureConnection
from src.gui.sharedcomnponets.commandLineInterface import CommandLineWidget
from src.gui.sharedcomnponets.sharedcomponets import (WorkAreaTabWidget,
                                                      GUIToolKit)
from src.simpleFOCConnector import SimpleFOCDevice


class CommandLineConsoleTool(WorkAreaTabWidget):

    def __init__(self, parent=None):
        super().__init__(parent)

        self.device = SimpleFOCDevice.getInstance()

        self.verticalLayout = QtWidgets.QVBoxLayout(self)
        self.verticalLayout.setObjectName('verticalLayout')

        self.configureConnection = ConfigureConnection()
        self.verticalLayout.addWidget(self.configureConnection)

        self.commandLineInterface = CommandLineWidget()
        self.verticalLayout.addWidget(self.commandLineInterface)

        self.device.commProvider.rawDataReceived.connect(self.publishFilteredRawData)

    def publishFilteredRawData(self, data):
        # 绘图数据行（数字/负号开头的制表符分隔行）不回显：QTextEdit 逐行
        # append+滚屏在高频曲线流下会拖累 GUI（2026-09-20 绘图冻结根因之一）
        if data and (data[0].isdigit() or data[0] == '-'):
            return
        self.commandLineInterface.publishCommandResponseData(data)

    def getTabIcon(self):
        return GUIToolKit.getIconByName('consoletool')

    def getTabName(self):
        return self.device.connectionID
