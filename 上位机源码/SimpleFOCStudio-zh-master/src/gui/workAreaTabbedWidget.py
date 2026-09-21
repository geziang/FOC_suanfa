#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import os
from pathlib import Path

from PyQt5 import QtWidgets

from src.gui.commandlinetool.commandlinetool import CommandLineConsoleTool
from src.gui.configtool.deviceConfigurationTool import DeviceConfigurationTool
from src.gui.configtool.generatedCodeDisplay import GeneratedCodeDisplay
from src.gui.configtool.treeViewConfigTool import TreeViewConfigTool
from src.gui.configtool.tuningBench import TuningBenchWidget
from src.simpleFOCConnector import SimpleFOCDevice
from src.debugTrace import trace, trace_exception


class WorkAreaTabbedWidget(QtWidgets.QTabWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        trace('[UI] WorkAreaTabbedWidget.__init__ enter')
        self.setTabsClosable(True)
        self.setMovable(True)
        self.setObjectName('devicesTabWidget')

        self.device = SimpleFOCDevice.getInstance()

        self.cmdLineTool = None
        self.configDeviceTool = None
        self.tuningBenchTool = None
        self.generatedCodeTab = None
        self.activeToolsList = []

        self.tabCloseRequested.connect(self.removeTabHandler)

        self.setStyleSheet(
            'QTabBar::close - button { image: url(close.png) subcontrol - position: left; }')
        self.setStyleSheet('QTabBar::tab { height: 30px; width: 150px;}')
        trace('[UI] WorkAreaTabbedWidget.__init__ done')

    def removeTabHandler(self, index):
        trace('[UI] removeTabHandler enter index=%r', index)
        if type(self.currentWidget()) == CommandLineConsoleTool:
            self.cmdLineTool = None
        if type(self.currentWidget()) == DeviceConfigurationTool or type(
                self.currentWidget()) == TreeViewConfigTool:
            self.configDeviceTool = None
        if type(self.currentWidget()) == TuningBenchWidget:
            self.tuningBenchTool = None
        if type(self.currentWidget()) == GeneratedCodeDisplay:
            self.generatedCodeTab = None
        if self.configDeviceTool == None and self.cmdLineTool == None:
            if self.device.isConnected:
                self.device.disConnect()

        self.activeToolsList.pop(index)
        self.removeTab(index)

    def addTuningBench(self):
        """三环整定台（2026-09-20）：使用面浓缩的默认工作页，见 REF-13/REF-12 §4.2。"""
        trace('[UI] addTuningBench clicked; existing=%r', self.tuningBenchTool is not None)
        if self.tuningBenchTool is None:
            try:
                trace('[UI] addTuningBench constructing TuningBenchWidget')
                self.tuningBenchTool = TuningBenchWidget()
                trace('[UI] addTuningBench TuningBenchWidget constructed')
            except Exception as exception:
                trace_exception('addTuningBench', exception)
                raise
            self.activeToolsList.append(self.tuningBenchTool)
            self.addTab(self.tuningBenchTool,
                        self.tuningBenchTool.getTabIcon(), '三环整定')
            self.setCurrentIndex(self.currentIndex() + 1)

    def addDeviceForm(self):
        trace('[UI] addDeviceForm clicked; existing=%r', self.configDeviceTool is not None)
        if self.configDeviceTool is None:
            try:
                trace('[UI] addDeviceForm constructing DeviceConfigurationTool')
                self.configDeviceTool = DeviceConfigurationTool()
                trace('[UI] addDeviceForm DeviceConfigurationTool constructed')
            except Exception as exception:
                trace_exception('addDeviceForm', exception)
                raise
            self.activeToolsList.append(self.configDeviceTool)
            self.addTab(self.configDeviceTool,
                        self.configDeviceTool.getTabIcon(), 'Device')
            self.setCurrentIndex(self.currentIndex() + 1)
            
    def addDeviceTree(self):
        trace('[UI] addDeviceTree clicked; existing=%r', self.configDeviceTool is not None)
        if self.configDeviceTool is None:
            try:
                trace('[UI] addDeviceTree constructing TreeViewConfigTool')
                self.configDeviceTool = TreeViewConfigTool()
                trace('[UI] addDeviceTree TreeViewConfigTool constructed')
            except Exception as exception:
                trace_exception('addDeviceTree', exception)
                raise
            self.activeToolsList.append(self.configDeviceTool)
            self.addTab(self.configDeviceTool,
                        self.configDeviceTool.getTabIcon(), 'Device')
            self.setCurrentIndex(self.currentIndex() + 1)

    def openDevice(self):
        trace('[UI] openDevice clicked')
        if self.configDeviceTool is not None:
            # 原来这里是静默 no-op：整个函数体都套在 `if configDeviceTool is None` 里，
            # 已有 Device 页时连文件对话框都不弹，用户会以为"打开设备"了 —— 其实
            # configureDevice 从未执行，devCommandID 仍是空串（下行命令缺前缀）。
            trace('[UI] openDevice skipped: a device tab is already open')
            msgBox = QtWidgets.QMessageBox()
            msgBox.setIcon(QtWidgets.QMessageBox.Warning)
            msgBox.setText('已有「设备」页打开，无法再加载配置文件。\n'
                           '请先关闭该页（标签上的 ×），再执行「打开设备」。')
            msgBox.setWindowTitle('SimpleFOC configDeviceTool')
            msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
            msgBox.exec_()
            return
        # 下面这段沿用原缩进结构：执行到这里 configDeviceTool 必为 None
        if self.configDeviceTool is None:
            dlg = QtWidgets.QFileDialog()
            dlg.setFileMode(QtWidgets.QFileDialog.AnyFile)
            filenames = None
            if dlg.exec_():
                filenames = dlg.selectedFiles()
                try:
                    with open(filenames[0]) as json_file:
                        configurationInfo = json.load(json_file)
                        sfd = SimpleFOCDevice.getInstance()
                        sfd.configureDevice(configurationInfo)
                        self.configDeviceTool = TreeViewConfigTool()
                        sfd.openedFile = filenames
                        self.activeToolsList.append(self.configDeviceTool)
                        tabName = self.configDeviceTool.getTabName()
                        if tabName == '':
                            tabName = 'Device'
                        self.addTab(self.configDeviceTool,
                                    self.configDeviceTool.getTabIcon(), tabName)
                        self.setCurrentIndex(self.currentIndex() + 1)

                except Exception as exception:
                    msgBox = QtWidgets.QMessageBox()
                    msgBox.setIcon(QtWidgets.QMessageBox.Warning)
                    msgBox.setText('打开文件时出错')
                    msgBox.setWindowTitle('SimpleFOC configDeviceTool')
                    msgBox.setStandardButtons(QtWidgets.QMessageBox.Ok)
                    msgBox.exec()

    def saveDevice(self):
        trace('[UI] saveDevice clicked currentIndex=%r tabs=%d', self.currentIndex(), len(self.activeToolsList))
        if len(self.activeToolsList) > 0:
            currentconfigDeviceTool = self.activeToolsList[self.currentIndex()]
            if currentconfigDeviceTool.device.openedFile is None:
                options = QtWidgets.QFileDialog.Options()
                options |= QtWidgets.QFileDialog.DontUseNativeDialog
                fileName, _ = QtWidgets.QFileDialog.getSaveFileName(self,
                                                                    '保存电机配置参数',
                                                                    '',
                                                                    'JSON configuration file (*.json)',
                                                                    options=options)
                if fileName:
                    self.saveToFile(currentconfigDeviceTool.device, fileName)
            else:
                self.saveToFile(currentconfigDeviceTool.device,
                                currentconfigDeviceTool.device.openedFile)
                                
    def generateCode(self):
        trace('[UI] generateCode clicked currentIndex=%r tabs=%d', self.currentIndex(), len(self.activeToolsList))
        if len(self.activeToolsList) > 0:
            currentconfigDeviceTool = self.activeToolsList[self.currentIndex()]
            self.generatedCodeTab = GeneratedCodeDisplay()
            self.activeToolsList.append(self.generatedCodeTab)
            self.addTab(self.generatedCodeTab,
                        self.generatedCodeTab.getTabIcon(), self.generatedCodeTab.getTabName())
            self.setCurrentIndex(self.currentIndex() + 1)


    def saveToFile(self, deviceToSave, file):
        # 路径安全收口（Mimosa 要求）：只取 basename，固定落 exports/ 目录
        #（本文件位于 <root>/src/gui/，dirname 链取根；Path.write_text 落盘）
        if isinstance(file, list):
            if not file:
                return
            file = file[0]
        fileName = os.path.basename(str(file).replace('\x00', '')
                                    .replace('\\', '/'))
        if not fileName:
            return
        root = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        exportsDir = os.path.join(root, 'exports')
        os.makedirs(exportsDir, exist_ok=True)
        target = Path(exportsDir) / fileName
        target.write_text(
            json.dumps(deviceToSave.toJSON(), indent=4, sort_keys=True),
            encoding='utf-8')

    def openConsoleTool(self):
        trace('[UI] openConsoleTool clicked existing=%r', self.cmdLineTool is not None)
        if self.cmdLineTool is None:
            self.cmdLineTool = CommandLineConsoleTool()
            self.activeToolsList.append(self.cmdLineTool)
            self.addTab(self.cmdLineTool,
                        self.cmdLineTool.getTabIcon(), '命令行交互')
            self.setCurrentIndex(self.currentIndex() + 1)
