#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务管理页面
- 顶部：地图选择（下拉+刷新）
- 中部：任务信息（获取/重置/保存 + 任务组/任务点/属性）
- 下部：编辑区（任务组/任务点，仅框架）
- 底部：地图显示区（占位）
"""

from __future__ import annotations

from typing import Dict, List, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QFrame,
    QLabel,
    QHBoxLayout,
    QPushButton,
    QComboBox,
    QListWidget,
    QMessageBox,
    QPlainTextEdit,
    QSizePolicy,
    QGridLayout,
)

from ...core.ros_manager import ROSServiceManager


class TaskManagementPage(QWidget):
    def __init__(self, ros_manager: ROSServiceManager, parent=None):
        super().__init__(parent)
        self._ros = ros_manager
        self._maps: Dict[str, object] = {}

        # 任务数据（临时内存字符串 + 解析结果）
        self._tasks_yaml: str = ""
        self._tasks_data: dict = {}
        self._task_groups: List[dict] = []
        self._current_points: List[dict] = []

        # 地图选择
        self._combo_maps = QComboBox()
        self._combo_maps.setEditable(False)
        self._btn_refresh_maps = QPushButton("刷新地图列表")

        # 总控按钮
        self._btn_fetch = QPushButton("获取任务信息")
        self._btn_reset = QPushButton("重置任务信息")
        self._btn_save = QPushButton("保存任务信息")

        # 列表
        self._list_groups = QListWidget()
        self._list_points = QListWidget()

        # 任务属性（紧凑文本框）
        self._group_info = self._make_info_box()
        self._point_info = self._make_info_box()
        self._group_info.setFixedHeight(84)
        self._point_info.setFixedHeight(104)

        self._build_ui()
        self._wire()
        self._set_buttons_enabled(False, False, False)
        self._set_group_attrs(None)
        self._set_point_attrs(None)

    def _build_ui(self):
        root = QVBoxLayout()
        root.setSpacing(12)
        root.setContentsMargins(16, 16, 16, 16)
        self.setLayout(root)

        # 地图选择区（简化）
        map_card = QFrame()
        map_card.setObjectName("card")
        map_layout = QVBoxLayout()
        map_layout.setContentsMargins(16, 16, 16, 16)
        map_layout.setSpacing(10)
        map_card.setLayout(map_layout)

        title = QLabel("任务地图选择")
        title.setObjectName("sectionTitle")
        map_layout.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(QLabel("选择地图："))
        row.addWidget(self._combo_maps, 1)
        row.addWidget(self._btn_refresh_maps)
        row.addStretch()
        map_layout.addLayout(row)

        root.addWidget(map_card)

        # 任务信息区
        task_card = QFrame()
        task_card.setObjectName("card")
        task_layout = QVBoxLayout()
        task_layout.setContentsMargins(16, 16, 16, 16)
        task_layout.setSpacing(10)
        task_card.setLayout(task_layout)

        title2 = QLabel("任务信息")
        title2.setObjectName("sectionTitle")
        task_layout.addWidget(title2)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        btn_row.addWidget(self._btn_fetch)
        btn_row.addWidget(self._btn_reset)
        btn_row.addWidget(self._btn_save)
        btn_row.addStretch()
        task_layout.addLayout(btn_row)

        list_row = QHBoxLayout()
        list_row.setSpacing(14)

        # 任务组列表
        group_col = QVBoxLayout()
        group_col.setSpacing(6)
        group_col.setContentsMargins(0, 0, 0, 0)
        group_title = self._make_list_title("任务组列表")
        group_col.addWidget(group_title)
        self._list_groups.setFixedHeight(200)
        group_col.addWidget(self._list_groups, 1)
        list_row.addLayout(group_col, 1)

        # 任务点列表
        point_col = QVBoxLayout()
        point_col.setSpacing(6)
        point_col.setContentsMargins(0, 0, 0, 0)
        point_title = self._make_list_title("任务点列表")
        point_col.addWidget(point_title)
        self._list_points.setFixedHeight(200)
        point_col.addWidget(self._list_points, 1)
        list_row.addLayout(point_col, 1)

        # 属性区（上：任务组属性，下：任务点属性）
        attr_col = QVBoxLayout()
        attr_col.setSpacing(10)

        group_attr_title = QLabel("任务组属性")
        group_attr_title.setObjectName("muted")
        attr_col.addWidget(group_attr_title)

        attr_col.addWidget(self._group_info)

        attr_col.addSpacing(8)

        point_attr_title = QLabel("任务点属性")
        point_attr_title.setObjectName("muted")
        attr_col.addWidget(point_attr_title)

        attr_col.addWidget(self._point_info)
        attr_col.addStretch()

        list_row.addLayout(attr_col, 1)
        task_layout.addLayout(list_row)

        root.addWidget(task_card)

        # 编辑区（框架）
        edit_row = QHBoxLayout()
        edit_row.setSpacing(12)

        group_edit_card = self._build_group_edit_card()
        point_edit_card = self._build_point_edit_card()
        edit_row.addWidget(group_edit_card, 1)
        edit_row.addWidget(point_edit_card, 1)

        root.addLayout(edit_row)

        # 地图显示区（占位）
        map_view = QFrame()
        map_view.setObjectName("card")
        map_view_layout = QVBoxLayout()
        map_view_layout.setContentsMargins(16, 16, 16, 16)
        map_view_layout.setSpacing(10)
        map_view.setLayout(map_view_layout)

        title3 = QLabel("地图显示")
        title3.setObjectName("sectionTitle")
        map_view_layout.addWidget(title3)

        hint = QLabel("地图显示区域（占位，后续支持缩放与拖动）")
        hint.setObjectName("muted")
        hint.setAlignment(Qt.AlignCenter)
        hint.setMinimumHeight(160)
        map_view_layout.addWidget(hint)

        root.addWidget(map_view)
        root.addStretch()

    def _build_group_edit_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(8)
        card.setLayout(layout)

        title = QLabel("任务组编辑")
        title.setObjectName("sectionTitle")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)

        items = [
            ("新建任务组", 0, 0),
            ("删除任务组", 0, 1),
            ("排序上移", 0, 2),
            ("任务组重命名", 1, 0),
            ("编辑描述", 1, 1),
            ("排序下移", 1, 2),
        ]
        for text, r, c in items:
            b = self._make_compact_btn(text)
            grid.addWidget(b, r, c)
        for i in range(3):
            grid.setColumnStretch(i, 1)
        layout.addLayout(grid)

        tip = QLabel("功能待接入")
        tip.setObjectName("muted")
        layout.addWidget(tip)
        layout.addStretch()
        return card

    def _build_point_edit_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(8)
        card.setLayout(layout)

        title = QLabel("任务点编辑")
        title.setObjectName("sectionTitle")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)

        items = [
            ("新建任务点", 0, 0),
            ("删除任务点", 0, 1),
            ("排序上移", 0, 2),
            ("任务点重命名", 1, 0),
            ("位姿编辑", 1, 1),
            ("排序下移", 1, 2),
        ]
        for text, r, c in items:
            b = self._make_compact_btn(text)
            grid.addWidget(b, r, c)
        for i in range(3):
            grid.setColumnStretch(i, 1)
        layout.addLayout(grid)

        tip = QLabel("功能待接入")
        tip.setObjectName("muted")
        layout.addWidget(tip)
        layout.addStretch()
        return card

    def _wire(self):
        self._btn_refresh_maps.clicked.connect(self.refresh_map_list)
        self._combo_maps.currentIndexChanged.connect(self._on_map_changed)

        self._btn_fetch.clicked.connect(self._on_fetch_tasks)
        self._btn_reset.clicked.connect(self._on_reset_tasks)
        self._btn_save.clicked.connect(self._on_save_tasks)

        self._list_groups.currentRowChanged.connect(self._on_group_selected)
        self._list_points.currentRowChanged.connect(self._on_point_selected)

    def _set_buttons_enabled(self, fetch: bool, reset: bool, save: bool):
        self._btn_fetch.setEnabled(fetch)
        self._btn_reset.setEnabled(reset)
        self._btn_save.setEnabled(save)

    def _current_map_name(self) -> str:
        return self._combo_maps.currentText().strip()

    def refresh_map_list(self):
        prev_selected = self._current_map_name()
        entries = self._ros.list_maps()
        if entries is None:
            return

        self._maps.clear()
        self._combo_maps.blockSignals(True)
        self._combo_maps.clear()

        selected_index = -1
        for e in entries:
            name = str(getattr(e, "name", "") or "").strip()
            if not name:
                continue
            self._maps[name] = e
            self._combo_maps.addItem(name)
            if prev_selected and name == prev_selected:
                selected_index = self._combo_maps.count() - 1

        self._combo_maps.blockSignals(False)

        if self._combo_maps.count() > 0:
            if selected_index < 0:
                selected_index = 0
            self._combo_maps.setCurrentIndex(selected_index)
            self._on_map_changed(selected_index)
        else:
            self._set_buttons_enabled(False, False, False)
            self._clear_task_views()

    def _on_map_changed(self, idx: int):
        has_map = bool(self._current_map_name())
        self._set_buttons_enabled(has_map, False, False)
        self._tasks_yaml = ""
        self._tasks_data = {}
        self._clear_task_views()

    def _on_fetch_tasks(self):
        map_name = self._current_map_name()
        if not map_name:
            QMessageBox.warning(self, "提示", "请先选择地图")
            return

        tasks_yaml = self._ros.get_map_tasks(map_name)
        if tasks_yaml is None:
            return

        self._tasks_yaml = tasks_yaml
        data = self._parse_tasks_yaml(tasks_yaml)
        if data is None:
            return

        self._tasks_data = data
        self._load_group_list(data)
        self._set_buttons_enabled(True, True, True)

    def _on_reset_tasks(self):
        # 重新从服务端读取
        self._on_fetch_tasks()

    def _on_save_tasks(self):
        map_name = self._current_map_name()
        if not map_name:
            QMessageBox.warning(self, "提示", "请先选择地图")
            return
        if not self._tasks_yaml:
            QMessageBox.warning(self, "提示", "尚未获取任务信息")
            return

        self._ros.set_map_tasks(map_name, self._tasks_yaml)

    def _parse_tasks_yaml(self, tasks_yaml: str) -> Optional[dict]:
        try:
            import yaml
        except Exception as e:
            QMessageBox.critical(self, "错误", f"缺少YAML解析库，无法读取任务信息：\n{str(e)}")
            return None

        try:
            data = yaml.safe_load(tasks_yaml) or {}
            if not isinstance(data, dict):
                raise ValueError("任务内容格式异常")
            return data
        except Exception as e:
            QMessageBox.critical(self, "错误", f"解析任务信息失败：\n{str(e)}")
            return None

    def _load_group_list(self, data: dict):
        groups = data.get("task_groups", [])
        if not isinstance(groups, list):
            groups = []

        self._task_groups = groups
        self._list_groups.blockSignals(True)
        self._list_groups.clear()
        for g in groups:
            if isinstance(g, dict):
                gid = g.get("id", "--")
                name = g.get("name", "未命名")
            else:
                gid = "--"
                name = "未命名"
            self._list_groups.addItem(f"[{gid}] {name}")
        self._list_groups.blockSignals(False)

        self._list_groups.setCurrentRow(-1)
        self._set_group_attrs(None)
        self._load_point_list([])

    def _load_point_list(self, points: List[dict]):
        self._current_points = points if isinstance(points, list) else []
        self._list_points.blockSignals(True)
        self._list_points.clear()
        for p in self._current_points:
            if isinstance(p, dict):
                pid = p.get("id", "--")
                name = p.get("name", "未命名")
            else:
                pid = "--"
                name = "未命名"
            self._list_points.addItem(f"[{pid}] {name}")
        self._list_points.blockSignals(False)

        self._list_points.setCurrentRow(-1)
        self._set_point_attrs(None)

    def _on_group_selected(self, row: int):
        if row < 0 or row >= len(self._task_groups):
            self._set_group_attrs(None)
            self._load_point_list([])
            return

        group = self._task_groups[row]
        if not isinstance(group, dict):
            self._set_group_attrs(None)
            self._load_point_list([])
            return

        self._set_group_attrs(group)
        points = group.get("points", [])
        if not isinstance(points, list):
            points = []
        self._load_point_list(points)

    def _on_point_selected(self, row: int):
        if row < 0 or row >= len(self._current_points):
            self._set_point_attrs(None)
            return

        point = self._current_points[row]
        if not isinstance(point, dict):
            self._set_point_attrs(None)
            return

        self._set_point_attrs(point)

    def _clear_task_views(self):
        self._list_groups.blockSignals(True)
        self._list_groups.clear()
        self._list_groups.blockSignals(False)

        self._list_points.blockSignals(True)
        self._list_points.clear()
        self._list_points.blockSignals(False)

        self._set_group_attrs(None)
        self._set_point_attrs(None)

    def _set_group_attrs(self, group: Optional[dict]):
        if not group:
            self._group_info.setPlainText("未选择")
            return

        gid = group.get("id", "--")
        name = group.get("name", "--")
        desc = group.get("description", "--")
        point_count = group.get("point_count", "--")
        if point_count == "--":
            points = group.get("points", [])
            if isinstance(points, list):
                point_count = len(points)

        text = (
            f"ID: {gid}\n"
            f"名称: {name}\n"
            f"描述: {desc}\n"
            f"任务点数量: {point_count}"
        )
        self._group_info.setPlainText(text)

    def _set_point_attrs(self, point: Optional[dict]):
        if not point:
            self._point_info.setPlainText("未选择")
            return

        pid = point.get("id", "--")
        name = point.get("name", "--")
        x = point.get("x", "--")
        y = point.get("y", "--")
        theta = point.get("theta", "--")

        text = (
            f"ID: {pid}\n"
            f"名称: {name}\n"
            f"X: {x}\n"
            f"Y: {y}\n"
            f"Theta: {theta}"
        )
        self._point_info.setPlainText(text)

    @staticmethod
    def _make_info_box() -> QPlainTextEdit:
        box = QPlainTextEdit()
        box.setObjectName("compactInfoBox")
        box.setReadOnly(True)
        box.setPlainText("未选择")
        box.setLineWrapMode(QPlainTextEdit.WidgetWidth)
        return box

    @staticmethod
    def _make_compact_btn(text: str) -> QPushButton:
        btn = QPushButton(text)
        btn.setObjectName("compactActionBtn")
        btn.setEnabled(False)
        btn.setFixedHeight(32)
        btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        return btn

    @staticmethod
    def _make_list_title(text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("listTitle")
        label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        label.setFixedHeight(18)
        return label
