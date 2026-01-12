#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底部状态栏（自定义一行）"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel

from ...config.settings import config


class AppStatusBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("statusBar")
        self.setFixedHeight(config.STATUS_BAR_HEIGHT)

        self._lbl_manager = QLabel("Manager: --")
        self._lbl_manager.setObjectName("statusItem")
        self._lbl_slam = QLabel("SLAM: --")
        self._lbl_slam.setObjectName("statusItem")
        self._lbl_uptime = QLabel("Uptime: --")
        self._lbl_uptime.setObjectName("statusItem")
        self._lbl_perf = QLabel("CPU: -- | MEM: -- | GPU: --")
        self._lbl_perf.setObjectName("statusItem")

        layout = QHBoxLayout()
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(8)
        self.setLayout(layout)

        layout.addWidget(self._lbl_manager)
        layout.addWidget(self._lbl_slam)
        layout.addWidget(self._lbl_uptime)
        layout.addStretch()
        layout.addWidget(self._lbl_perf)

    def set_manager_running(self, running: bool):
        self._lbl_manager.setText(f"Manager: {'RUN' if running else 'STOP'}")

    def set_slam_status(self, status: str):
        self._lbl_slam.setText(f"SLAM: {status or '--'}")

    def set_uptime(self, uptime_sec: int):
        if uptime_sec <= 0:
            self._lbl_uptime.setText("Uptime: --")
            return
        h = uptime_sec // 3600
        m = (uptime_sec % 3600) // 60
        s = uptime_sec % 60
        if h > 0:
            self._lbl_uptime.setText(f"Uptime: {h:02d}:{m:02d}:{s:02d}")
        else:
            self._lbl_uptime.setText(f"Uptime: {m:02d}:{s:02d}")

    def set_perf(self, cpu, mem, gpu):
        def f(x):
            return "--" if x is None else f"{x:.1f}%"
        self._lbl_perf.setText(f"CPU: {f(cpu)} | MEM: {f(mem)} | GPU: {f(gpu)}")


