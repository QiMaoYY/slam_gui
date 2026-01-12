#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GUI 日志工具：
- 将 python logging 输出到Qt控件（QPlainTextEdit）
- 可选重定向 stdout/stderr（捕获print与未处理异常）
"""

import logging
import sys
from typing import Optional, Tuple, Callable

from PyQt5.QtCore import QObject, pyqtSignal


class LogSignal(QObject):
    log_signal = pyqtSignal(str)


class QtLogHandler(logging.Handler):
    """将logging记录通过Qt信号转发到UI线程"""

    def __init__(self, signal: LogSignal):
        super().__init__()
        self._signal = signal

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
        except Exception:
            msg = record.getMessage()
        self._signal.log_signal.emit(msg)


class StreamToLogger:
    """将stdout/stderr写入logging（用于捕获print与异常堆栈）"""

    def __init__(self, logger: logging.Logger, level: int):
        self._logger = logger
        self._level = level
        self._buffer = ""

    def write(self, s: str) -> None:
        if not s:
            return
        self._buffer += s
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            line = line.rstrip()
            if line:
                self._logger.log(self._level, line)

    def flush(self) -> None:
        if self._buffer.strip():
            self._logger.log(self._level, self._buffer.strip())
        self._buffer = ""


class StdRedirectGuard:
    """上下文式保存/恢复stdout/stderr，避免退出时污染其它环境"""

    def __init__(self):
        self._old_out = None
        self._old_err = None

    def redirect(self, stdout_obj, stderr_obj) -> None:
        self._old_out = sys.stdout
        self._old_err = sys.stderr
        sys.stdout = stdout_obj
        sys.stderr = stderr_obj

    def restore(self) -> None:
        if self._old_out is not None:
            sys.stdout = self._old_out
        if self._old_err is not None:
            sys.stderr = self._old_err


def setup_gui_logger(
    on_log_line: Callable[[str], None],
    *,
    logger_name: str = "slam_gui",
    level: int = logging.INFO,
    redirect_std: bool = True,
) -> Tuple[logging.Logger, LogSignal, StdRedirectGuard]:
    """
    初始化GUI日志系统。

    Args:
        on_log_line: 回调函数(str)->None，用于把日志行追加到UI控件
        logger_name: logger名称
        level: logger等级
        redirect_std: 是否将stdout/stderr重定向到logger

    Returns:
        (logger, signal, redirect_guard)
    """
    signal = LogSignal()
    signal.log_signal.connect(on_log_line)

    logger = logging.getLogger(logger_name)
    logger.setLevel(level)
    logger.propagate = False

    # 避免重复添加handler（多次创建窗口/热重载）
    for h in list(logger.handlers):
        if isinstance(h, QtLogHandler):
            logger.removeHandler(h)

    handler = QtLogHandler(signal)
    handler.setLevel(level)
    handler.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(message)s"))
    logger.addHandler(handler)

    guard = StdRedirectGuard()
    if redirect_std:
        guard.redirect(StreamToLogger(logger, logging.INFO), StreamToLogger(logger, logging.ERROR))

    return logger, signal, guard


