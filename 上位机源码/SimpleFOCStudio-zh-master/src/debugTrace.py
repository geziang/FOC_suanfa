#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Lightweight diagnostics for locating GUI/serial stalls.

The helper deliberately has no control-flow side effects: it prints and logs
only, so adding diagnostics cannot change the SimpleFOC command sequence.
"""
import logging
import traceback


def trace(message, *args):
    text = message % args if args else str(message)
    logging.getLogger('SimpleFOCTrace').info(text)
    print(text, flush=True)


def trace_exception(where, exception):
    text = '[EXCEPTION] %s: %s' % (where, exception)
    logging.getLogger('SimpleFOCTrace').exception(text)
    print(text, flush=True)
    traceback.print_exc()
