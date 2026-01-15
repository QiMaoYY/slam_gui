#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务管理页面
- 地图选择区（下拉+刷新+基本信息+2D预览）
"""

from __future__ import annotations

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QFrame, QLabel

from ...core.ros_manager import ROSServiceManager
from ..widgets.map_selector import MapSelectorPanel


class TaskManagementPage(QWidget):
    def __init__(self, ros_manager: ROSServiceManager, parent=None):
        super().__init__(parent)
        self._ros = ros_manager
        self._map_selector = MapSelectorPanel(self._ros, title="任务地图选择")

        self._build_ui()

    def _build_ui(self):
        root = QVBoxLayout()
        root.setSpacing(12)
        root.setContentsMargins(16, 16, 16, 16)
        self.setLayout(root)

        root.addWidget(self._map_selector)

        placeholder = QFrame()
        placeholder.setObjectName("card")
        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        placeholder.setLayout(layout)

        t = QLabel("任务控制")
        t.setObjectName("sectionTitle")
        layout.addWidget(t)
        tip = QLabel("该页面功能待接入（已预留框架与接口）。")
        tip.setObjectName("muted")
        layout.addWidget(tip)

        root.addWidget(placeholder)
        root.addStretch()
