#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图管理页面
- 上：建图控制（开始/保存/终止/校准）
- 下：地图编辑区（下拉+按钮，功能先预留接口）
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QFrame,
    QCheckBox,
    QInputDialog,
    QMessageBox,
    QComboBox,
)

from ...core.ros_manager import ROSServiceManager
from ..styles import DarkTheme


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


class MapManagementPage(QWidget):
    def __init__(self, ros_manager: ROSServiceManager, parent=None):
        super().__init__(parent)
        self._ros = ros_manager
        self._maps: Dict[str, object] = {}

        self._calib = QCheckBox("启用雷达校准（首次建图推荐）")
        self._calib.setChecked(True)

        self._btn_start = QPushButton("开始建图")
        self._btn_start.setObjectName("startMappingBtn")
        self._btn_save = QPushButton("保存地图")
        self._btn_save.setObjectName("saveMapBtn")
        self._btn_abort = QPushButton("终止建图")
        self._btn_abort.setObjectName("abortMappingBtn")

        self._combo_maps = QComboBox()
        self._combo_maps.setEditable(False)

        self._btn_refresh = QPushButton("刷新地图列表")
        self._btn_gen2d = QPushButton("生成2D地图")
        self._btn_view2d = QPushButton("查看2D地图")
        self._btn_edit2d = QPushButton("编辑2D地图")
        self._btn_delete = QPushButton("删除该地图")

        # 地图详情（选择后显示）
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

        self._build_ui()
        self._wire()
        self.set_mapping_state("disconnected")

    def _build_ui(self):
        root = QVBoxLayout()
        root.setSpacing(12)
        root.setContentsMargins(16, 16, 16, 16)
        self.setLayout(root)

        # 建图控制区
        mapping_card = QFrame()
        mapping_card.setObjectName("card")
        mapping_layout = QVBoxLayout()
        mapping_layout.setContentsMargins(16, 16, 16, 16)
        mapping_layout.setSpacing(10)
        mapping_card.setLayout(mapping_layout)

        title = QLabel("建图管理")
        title.setObjectName("sectionTitle")
        mapping_layout.addWidget(title)

        mapping_layout.addWidget(self._calib)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        btn_row.addWidget(self._btn_start)
        btn_row.addWidget(self._btn_save)
        btn_row.addWidget(self._btn_abort)
        btn_row.addStretch()
        mapping_layout.addLayout(btn_row)

        root.addWidget(mapping_card)

        # 地图编辑区（接口预留）
        edit_card = QFrame()
        edit_card.setObjectName("card")
        edit_layout = QVBoxLayout()
        edit_layout.setContentsMargins(16, 16, 16, 16)
        edit_layout.setSpacing(10)
        edit_card.setLayout(edit_layout)

        title2 = QLabel("地图编辑")
        title2.setObjectName("sectionTitle")
        edit_layout.addWidget(title2)

        select_row = QHBoxLayout()
        select_row.setSpacing(10)
        select_row.addWidget(QLabel("选择地图："))
        select_row.addWidget(self._combo_maps, 1)
        select_row.addWidget(self._btn_refresh)
        edit_layout.addLayout(select_row)

        action_row = QHBoxLayout()
        action_row.setSpacing(10)
        action_row.addWidget(self._btn_gen2d)
        action_row.addWidget(self._btn_view2d)
        action_row.addWidget(self._btn_edit2d)
        action_row.addWidget(self._btn_delete)
        action_row.addStretch()
        edit_layout.addLayout(action_row)

        # 详情区：时间/大小 + 2D缩略图
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
        t = QLabel("2D预览")
        t.setObjectName("muted")
        right.addWidget(t)
        right.addWidget(self._preview, 0, Qt.AlignLeft)
        right.addStretch()

        detail_row.addLayout(right, 0)

        edit_layout.addLayout(detail_row)

        root.addWidget(edit_card)
        root.addStretch()

        # 先禁用未实现功能按钮（后续你安排再接）
        self._btn_refresh.setEnabled(True)
        self._btn_gen2d.setEnabled(False)   # 预留
        self._btn_delete.setEnabled(False)  # 预留
        self._btn_view2d.setEnabled(False)  # 由nav_ready控制
        self._btn_edit2d.setEnabled(False)  # 由nav_ready控制

    def _wire(self):
        self._btn_start.clicked.connect(self._on_start_mapping)
        self._btn_save.clicked.connect(self._on_save_map)
        self._btn_abort.clicked.connect(self._on_abort_mapping)
        self._btn_refresh.clicked.connect(self.refresh_map_list)
        self._combo_maps.currentIndexChanged.connect(self._on_map_selected)

    def _on_start_mapping(self):
        need_calibration = self._calib.isChecked()
        self._ros.start_mapping(need_calibration)

    def _on_save_map(self):
        dialog = QInputDialog(self)
        dialog.setWindowTitle("保存地图")
        dialog.setLabelText(
            "请输入地图名称：\n\n"
            "规则：\n"
            "- 只能包含字母、数字、下划线(_)、横线(-)\n"
            "- 长度不超过50个字符\n"
            "- 不能为空"
        )
        dialog.setTextValue("")
        dialog.setStyleSheet(DarkTheme.get_inputdialog_style())

        ok = dialog.exec_()
        map_name = dialog.textValue()
        if not ok:
            return

        map_name = (map_name or "").strip()
        if not map_name:
            QMessageBox.warning(self, "错误", "地图名称不能为空")
            return
        if len(map_name) > 50:
            QMessageBox.warning(self, "错误", "地图名称长度不能超过50个字符")
            return
        if not re.match(r"^[a-zA-Z0-9_-]+$", map_name):
            QMessageBox.warning(self, "错误", f"地图名称包含非法字符: {map_name}")
            return

        reply = QMessageBox.question(
            self,
            "确认保存",
            f'确定要停止建图并保存为 "{map_name}" 吗？',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )
        if reply == QMessageBox.Yes:
            self._ros.stop_mapping(save_map=True, map_name=map_name)

    def _on_abort_mapping(self):
        reply = QMessageBox.question(
            self,
            "确认终止",
            "确定要终止建图吗？\n\n⚠️ 警告：地图将不会被保存！",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self._ros.stop_mapping(save_map=False, map_name="")

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
        entries = self._ros.list_maps()
        if entries is None:
            return

        self._maps.clear()
        self._combo_maps.blockSignals(True)
        self._combo_maps.clear()

        for e in entries:
            self._maps[e.name] = e
            self._combo_maps.addItem(e.name)

        self._combo_maps.blockSignals(False)

        if self._combo_maps.count() > 0:
            self._combo_maps.setCurrentIndex(0)
            self._on_map_selected(0)
        else:
            self._set_map_detail_none()

    def _set_map_detail_none(self):
        self._lbl_created.setText("--")
        self._lbl_ori_size.setText("--")
        self._lbl_nav_size.setText("--")
        self._preview.setText("无2d预览图")
        self._preview.setPixmap(QPixmap())
        self._btn_view2d.setEnabled(False)
        self._btn_edit2d.setEnabled(False)

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

        # 2D缩略图：map_path/sketch.png
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

        # 可导航地图才允许查看/编辑2D
        self._btn_view2d.setEnabled(nav_ready)
        self._btn_edit2d.setEnabled(nav_ready)

    def set_mapping_state(self, slam_status: str):
        """
        根据slam状态更新建图按钮状态。
        slam_status: idle/mapping/localizing/navigating/error/disconnected/...
        """
        status = slam_status or "disconnected"
        if status == "idle":
            self._btn_start.setEnabled(True)
            self._btn_save.setEnabled(False)
            self._btn_abort.setEnabled(False)
            self._calib.setEnabled(True)
        elif status == "mapping":
            self._btn_start.setEnabled(False)
            self._btn_save.setEnabled(True)
            self._btn_abort.setEnabled(True)
            self._calib.setEnabled(False)
        else:
            # 其它状态先统一禁用建图控制，避免误操作
            self._btn_start.setEnabled(False)
            self._btn_save.setEnabled(False)
            self._btn_abort.setEnabled(False)
            self._calib.setEnabled(False)


