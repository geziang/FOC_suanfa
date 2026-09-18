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
from src.debugTrace import trace, trace_exception

if __name__ == '__main__':
    try:
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s %(levelname)s %(name)s - %(message)s',
            handlers=[
                logging.FileHandler('.SimpleFOCConfigTool.log', mode='w', encoding='utf-8'),
                logging.StreamHandler(sys.stdout),
            ],
            force=True,
        )
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
