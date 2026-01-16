#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图选择面板（通用）
- 下拉选择 + 刷新
- 创建时间/地图大小
- 2D预览图
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QFrame,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QComboBox,
)

from ...core.ros_manager import ROSServiceManager


def _format_created_at(s: str) -> str:
    # YYYYMMDD_HHMMSS -> YYYY-MM-DD HH:MM:SS
    if not s:
        return "--"
    try:
        if "_" in s:
            d, t = s.split("_", 1)
        else:
            d, t = s[:8], s[8:]
        if len(d) == 8 and len(t) >= 6:
            return f"{d[0:4]}-{d[4:6]}-{d[6:8]} {t[0:2]}:{t[2:4]}:{t[4:6]}"
        return s
    except Exception:
        return s


def _bytes_to_mb_str(b: int) -> str:
    try:
        mb = float(b) / (1024.0 * 1024.0)
        return f"{mb:.2f} MB"
    except Exception:
        return "0.00 MB"


class MapSelectorPanel(QFrame):
    map_changed = pyqtSignal(object)  # MapEntry | None

    def __init__(self, ros_manager: ROSServiceManager, title: str = "地图选择", parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        self._ros = ros_manager
        self._maps: Dict[str, object] = {}

        self._combo_maps = QComboBox()
        self._combo_maps.setEditable(False)
        self._btn_refresh = QPushButton("刷新地图列表")

        self._lbl_created = QLabel("--")
        self._lbl_created.setObjectName("valueBox")
        self._lbl_ori_size = QLabel("--")
        self._lbl_ori_size.setObjectName("valueBox")
        self._lbl_nav_size = QLabel("--")
        self._lbl_nav_size.setObjectName("valueBox")

        self._preview = QLabel("无2d预览图")
        self._preview.setObjectName("muted")
        self._preview.setAlignment(Qt.AlignCenter)
        self._preview.setFixedSize(320, 180)
        self._preview.setStyleSheet(
            "background-color:#0b0e14;border:1px solid rgba(120,140,170,0.18);border-radius:10px;"
        )

        self._build_ui(title)
        self._wire()

    def current_map_name(self) -> str:
        return self._combo_maps.currentText().strip()

    def current_entry(self):
        name = self.current_map_name()
        if not name or name not in self._maps:
            return None
        return self._maps[name]

    def current_nav_ready(self) -> bool:
        e = self.current_entry()
        if e is None:
            return False
        return bool(getattr(e, "nav_ready", False))

    def _build_ui(self, title: str):
        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        self.setLayout(layout)

        t = QLabel(title)
        t.setObjectName("sectionTitle")
        layout.addWidget(t)

        select_row = QHBoxLayout()
        select_row.setSpacing(10)
        select_row.addWidget(QLabel("选择地图："))
        select_row.addWidget(self._combo_maps, 1)
        select_row.addWidget(self._btn_refresh)
        layout.addLayout(select_row)

        detail_row = QHBoxLayout()
        detail_row.setSpacing(18)

        left = QVBoxLayout()
        left.setSpacing(10)
        left.addLayout(self._kv("创建时间", self._lbl_created))
        left.addLayout(self._kv("原始点云大小", self._lbl_ori_size))
        left.addLayout(self._kv("导航点云大小", self._lbl_nav_size))
        left.addStretch()
        detail_row.addLayout(left, 1)

        right = QVBoxLayout()
        right.setSpacing(6)
        t2 = QLabel("2D预览")
        t2.setObjectName("muted")
        right.addWidget(t2)
        right.addWidget(self._preview, 0, Qt.AlignLeft)
        right.addStretch()
        detail_row.addLayout(right, 0)

        layout.addLayout(detail_row)

    def _wire(self):
        self._btn_refresh.clicked.connect(self.refresh_map_list)
        self._combo_maps.currentIndexChanged.connect(self._on_map_selected)

    @staticmethod
    def _kv(k: str, v_label: QLabel):
        k_label = QLabel(k)
        k_label.setObjectName("muted")
        col = QVBoxLayout()
        col.setSpacing(4)
        col.addWidget(k_label)
        col.addWidget(v_label)
        return col

    def refresh_map_list(self):
        prev_selected = self._combo_maps.currentText().strip()

        entries = self._ros.list_maps()
        if entries is None:
            return

        self._maps.clear()
        self._combo_maps.blockSignals(True)
        self._combo_maps.clear()

        selected_index = -1
        for e in entries:
            self._maps[e.name] = e
            self._combo_maps.addItem(e.name)
            if prev_selected and e.name == prev_selected:
                selected_index = self._combo_maps.count() - 1

        self._combo_maps.blockSignals(False)

        if self._combo_maps.count() > 0:
            if selected_index < 0:
                selected_index = 0
            self._combo_maps.setCurrentIndex(selected_index)
            self._on_map_selected(selected_index)
        else:
            self._set_map_detail_none()

    def _set_map_detail_none(self):
        self._lbl_created.setText("--")
        self._lbl_ori_size.setText("--")
        self._lbl_nav_size.setText("--")
        self._preview.setText("无2d预览图")
        self._preview.setPixmap(QPixmap())
        self.map_changed.emit(None)

    def _on_map_selected(self, idx: int):
        name = self._combo_maps.currentText().strip()
        if not name or name not in self._maps:
            self._set_map_detail_none()
            return

        e = self._maps[name]
        created = _format_created_at(getattr(e, "created_at", "") or "")
        ori_mb = _bytes_to_mb_str(int(getattr(e, "ori_pointcloud_bytes", 0) or 0))
        nav_ready = bool(getattr(e, "nav_ready", False))
        nav_bytes = int(getattr(e, "nav_pointcloud_bytes", 0) or 0) if nav_ready else 0
        nav_mb = _bytes_to_mb_str(nav_bytes)

        self._lbl_created.setText(created)
        self._lbl_ori_size.setText(ori_mb)
        self._lbl_nav_size.setText(nav_mb)

        map_path = str(getattr(e, "path", "") or "")
        sketch = Path(map_path) / "sketch.png"
        if nav_ready and sketch.exists():
            pix = QPixmap(str(sketch))
            if not pix.isNull():
                scaled = pix.scaled(self._preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self._preview.setPixmap(scaled)
                self._preview.setText("")
            else:
                self._preview.setText("无2d预览图")
                self._preview.setPixmap(QPixmap())
        else:
            self._preview.setText("无2d预览图")
            self._preview.setPixmap(QPixmap())

        self.map_changed.emit(e)
