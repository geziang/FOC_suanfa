#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Lightweight diagnostics for locating GUI/serial stalls.

The helper deliberately has no control-flow side effects: it prints and logs
only, so adding diagnostics cannot change the SimpleFOC command sequence.

打印分两档（2026-09-20 裁剪）：
  trace()         触发型 —— 启动、连接/断开、拉取、界面动作、异常。始终输出。
  trace_verbose() 循环型 —— 逐条报文、每圈轮询、每帧波形。默认不占控制台，
                  始终落日志文件；需要现场排查时用环境变量把控制台也打开：
                      cmd:      set FOC_TRACE_VERBOSE=1 && python simpleFOCStudio.py
                      bash:     FOC_TRACE_VERBOSE=1 python simpleFOCStudio.py
                  或在代码里调用 enable_verbose_console(True) 运行期打开。
"""
import logging
import os
import traceback

_VERBOSE_TRUE = ('1', 'true', 'yes', 'on')

# 冗长打印是否同时打到控制台（默认否；日志文件始终保留全量）
VERBOSE_TO_CONSOLE = os.environ.get('FOC_TRACE_VERBOSE', '').strip().lower() in _VERBOSE_TRUE

# 独立 logger：不向 root 传播，免得被 root 的 StreamHandler 打到控制台
_verbose_logger = logging.getLogger('SimpleFOCVerbose')
_verbose_logger.propagate = False
_verbose_logger.setLevel(logging.INFO)


def trace(message, *args):
    """触发型打印：控制台 + 日志文件，始终输出。"""
    text = message % args if args else str(message)
    logging.getLogger('SimpleFOCTrace').info(text)
    print(text, flush=True)


def trace_verbose(message, *args):
    """循环/高频打印：默认只进日志文件，不刷控制台。"""
    text = message % args if args else str(message)
    _verbose_logger.info(text)
    if VERBOSE_TO_CONSOLE:
        print(text, flush=True)


def enable_verbose_console(on=True):
    """运行期切换冗长打印是否上控制台，返回切换后的状态。"""
    global VERBOSE_TO_CONSOLE
    VERBOSE_TO_CONSOLE = bool(on)
    return VERBOSE_TO_CONSOLE


def attach_verbose_handler(handler):
    """由启动脚本注入日志文件 handler，使冗长打印始终有落盘通道。"""
    if handler is not None and handler not in _verbose_logger.handlers:
        _verbose_logger.addHandler(handler)


def trace_exception(where, exception):
    text = '[EXCEPTION] %s: %s' % (where, exception)
    logging.getLogger('SimpleFOCTrace').exception(text)
    print(text, flush=True)
    traceback.print_exc()
