#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航管理页面
- 地图选择区（下拉+刷新+基本信息+2D预览）
"""

from __future__ import annotations

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QFrame, QLabel, QHBoxLayout, QPushButton, QCheckBox

from ...core.ros_manager import ROSServiceManager
from ..widgets.map_selector import MapSelectorPanel


class NavigationManagementPage(QWidget):
    def __init__(self, ros_manager: ROSServiceManager, parent=None):
        super().__init__(parent)
        self._ros = ros_manager
        self._map_selector = MapSelectorPanel(self._ros, title="导航地图选择")

        self._slam_status = "disconnected"

        self._chk_rviz = QCheckBox("开启RViz")
        self._chk_rviz.setChecked(True)
        self._chk_calib = QCheckBox("雷达校准")
        self._chk_calib.setChecked(False)

        self._btn_start_nav = QPushButton("开始导航")
        self._btn_stop_nav = QPushButton("结束导航")

        self._build_ui()
        self._wire()
        self._refresh_nav_controls()

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

        t = QLabel("导航控制")
        t.setObjectName("sectionTitle")
        layout.addWidget(t)

        opt_row = QHBoxLayout()
        opt_row.setSpacing(18)
        opt_row.addWidget(self._chk_rviz)
        opt_row.addWidget(self._chk_calib)
        opt_row.addStretch()
        layout.addLayout(opt_row)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        btn_row.addWidget(self._btn_start_nav)
        btn_row.addWidget(self._btn_stop_nav)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        tip = QLabel("提示：仅“可导航地图”才能开始导航（需存在 map2d.yaml/map2d.pgm/pointcloud.pcd）。")
        tip.setObjectName("muted")
        layout.addWidget(tip)

        root.addWidget(placeholder)
        root.addStretch()

    def _wire(self):
        self._map_selector.map_changed.connect(lambda _: self._refresh_nav_controls())
        self._btn_start_nav.clicked.connect(self._on_start_navigation)
        self._btn_stop_nav.clicked.connect(self._on_stop_navigation)

    def set_slam_state(self, slam_status: str):
        self._slam_status = slam_status or "disconnected"
        self._refresh_nav_controls()

    def _refresh_nav_controls(self):
        # 仅可导航地图允许开始导航
        nav_ready = self._map_selector.current_nav_ready()
        navigating = (self._slam_status == "navigating")

        self._btn_start_nav.setEnabled(bool(nav_ready) and not navigating)
        self._btn_stop_nav.setEnabled(bool(navigating))

    def _on_start_navigation(self):
        e = self._map_selector.current_entry()
        if e is None:
            return
        map_name = str(getattr(e, "name", "") or "").strip()
        if not map_name:
            return
        if not bool(getattr(e, "nav_ready", False)):
            return

        self._ros.start_navigation(
            map_name=map_name,
            enable_rviz=self._chk_rviz.isChecked(),
            need_calibration=self._chk_calib.isChecked(),
        )

    def _on_stop_navigation(self):
        self._ros.stop_navigation()
