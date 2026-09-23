#!/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtWidgets

from src.gui.toolbar import SimpleFOCConfigToolBar
from src.gui.workAreaTabbedWidget import WorkAreaTabbedWidget


class UserInteractionMainWindow(object):

    def setupUi(self, main_window):

        main_window.setObjectName('MainWindow')
        main_window.resize(1300, 900)
        main_window.setWindowTitle('SimpleFOC Configuration Tool ')

        self.centralwidget = QtWidgets.QWidget(main_window)
        self.centralwidget.setObjectName('centralwidget')

        # Add layout de to the main window
        self.horizontalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName('verticalLayout')

        # Add tabebd tools widget to the main  window
        self.tabbedToolsWidget = WorkAreaTabbedWidget(self.centralwidget)
        self.horizontalLayout.addWidget(self.tabbedToolsWidget)

        # Add toolbar to the main window
        self.toolBar = SimpleFOCConfigToolBar(main_window,self.tabbedToolsWidget, main_window)
        main_window.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)

        # Add status bar to the main window
        self.statusbar = QtWidgets.QStatusBar(main_window)
        self.statusbar.setObjectName('statusbar')
        main_window.setStatusBar(self.statusbar)

        # Add central Widget to the main window
        main_window.setCentralWidget(self.centralwidget)

        # FocKit 绑定（2026-09-20）：启动自动开页，免「文件 → 打开设备」。
        # 默认第一页 = 三环整定台（使用面浓缩的工作页，REF-13）；
        # 第二页 = 力控台（2026-09-23 独立成页：曲线源吃 [FC DBG] 探针行，
        # 与三环 Monitor 流分道，REF-14 §5）；
        # 设备页随后（全功能后盾：全景参数/树形视图）。链路参数已写死，
        # 整定台的主路径操作 = "选端口 + 点连接 + 点环"。
        self.tabbedToolsWidget.addTuningBench()
        self.tabbedToolsWidget.addForceBench()
        self.tabbedToolsWidget.addDeviceForm()
        self.tabbedToolsWidget.setCurrentIndex(0)

