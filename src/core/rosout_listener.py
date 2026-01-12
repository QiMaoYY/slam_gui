#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
订阅/rosout_agg并筛选指定节点日志，用于GUI右侧日志面板。
"""

from __future__ import annotations

from datetime import datetime
from typing import Callable, Optional

import rospy
from PyQt5.QtCore import QObject, pyqtSignal
from rosgraph_msgs.msg import Log


class RosoutSignal(QObject):
    line = pyqtSignal(str)


class SlamManagerRosoutListener:
    """
    监听/rosout_agg并筛选slam_manager日志。
    回调发生在rospy线程中，通过Qt信号转发到UI线程。
    """

    def __init__(
        self,
        on_line: Callable[[str], None],
        *,
        node_name_keyword: str = "slam_manager",
        topic: str = "/rosout_agg",
    ):
        self._keyword = node_name_keyword
        self._topic = topic
        self._signal = RosoutSignal()
        self._signal.line.connect(on_line)

        self._sub = rospy.Subscriber(self._topic, Log, self._cb, queue_size=200)

    def shutdown(self):
        try:
            self._sub.unregister()
        except Exception:
            pass

    def _cb(self, msg: Log):
        try:
            # msg.name: logger name (通常是节点名，如 /slam_manager)
            name = (msg.name or "").strip()
            if self._keyword not in name:
                return

            ts = self._format_stamp(msg.header.stamp)
            level = self._level_to_str(msg.level)
            text = (msg.msg or "").rstrip()
            if not text:
                return

            line = f"{ts} | {level:<5} | {name} | {text}"
            self._signal.line.emit(line)
        except Exception:
            # 忽略日志格式化错误，避免影响回调线程
            return

    @staticmethod
    def _format_stamp(stamp: rospy.Time) -> str:
        try:
            t = stamp.to_sec()
            dt = datetime.fromtimestamp(t)
            return dt.strftime("%H:%M:%S.%f")[:-3]
        except Exception:
            return "--:--:--.---"

    @staticmethod
    def _level_to_str(level: int) -> str:
        if level >= Log.FATAL:
            return "FATAL"
        if level >= Log.ERROR:
            return "ERROR"
        if level >= Log.WARN:
            return "WARN"
        if level >= Log.INFO:
            return "INFO"
        return "DEBUG"


