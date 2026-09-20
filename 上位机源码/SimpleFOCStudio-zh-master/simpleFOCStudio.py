#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" This module contains ans script to start the SimpleFOC ConfigTool, a GIU
    application ta monitor, tune and configure BLDC motor controllers based on
    SimpleFOC library.
"""
from PyQt5 import QtWidgets
from src.gui.mainWindow import UserInteractionMainWindow
import sys
import logging
from src.debugTrace import trace, trace_exception, attach_verbose_handler

if __name__ == '__main__':
    try:
        fileHandler = logging.FileHandler('.SimpleFOCConfigTool.log', mode='w', encoding='utf-8')
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s %(levelname)s %(name)s - %(message)s',
            handlers=[
                fileHandler,
                logging.StreamHandler(sys.stdout),
            ],
            force=True,
        )
        # 循环/高频打印（逐条报文、轮询、波形）默认只落日志文件，不刷控制台。
        # 需要现场排查时设环境变量 FOC_TRACE_VERBOSE=1 让它们也上控制台。
        attach_verbose_handler(fileHandler)
        trace('[BOOT] simpleFOCStudio.py start; argv=%r', sys.argv)
        app = QtWidgets.QApplication(sys.argv)
        trace('[BOOT] QApplication created')
        mainWindow = QtWidgets.QMainWindow()
        trace('[BOOT] QMainWindow created')
        userInteractionMainWindow = UserInteractionMainWindow()
        trace('[BOOT] UserInteractionMainWindow created')
        userInteractionMainWindow.setupUi(mainWindow)
        trace('[BOOT] setupUi completed')
        mainWindow.show()
        trace('[BOOT] main window shown; entering Qt event loop')
        sys.exit(app.exec_())
    except Exception as exception:\
        trace_exception('simpleFOCStudio main', exception)
