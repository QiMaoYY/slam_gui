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
    QInputDialog,
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
        self._tasks_loaded = False
        self._dirty = False
        self._ignore_map_change = False
        self._last_map_index = -1

        # 地图选择
        self._combo_maps = QComboBox()
        self._combo_maps.setEditable(False)
        self._combo_maps.setFixedWidth(220)
        self._btn_refresh_maps = QPushButton("刷新地图列表")

        # 总控按钮
        self._btn_fetch_reset = QPushButton("获取/重置任务信息")
        self._btn_save = QPushButton("保存任务信息")

        # 任务组操作按钮（在构建编辑区时初始化）
        self._btn_group_new = None
        self._btn_group_delete = None
        self._btn_group_up = None
        self._btn_group_down = None
        self._btn_group_rename = None
        self._btn_group_desc = None

        # 任务点操作按钮（在构建编辑区时初始化）
        self._btn_point_new = None
        self._btn_point_delete = None
        self._btn_point_up = None
        self._btn_point_down = None
        self._btn_point_rename = None
        self._btn_point_pose = None

        # 列表
        self._list_groups = QListWidget()
        self._list_points = QListWidget()

        # 任务属性（紧凑文本框）
        self._group_info = self._make_info_box()
        self._point_info = self._make_info_box()
        self._group_info.setFixedHeight(100)
        self._point_info.setFixedHeight(100)

        self._build_ui()
        self._wire()
        self._set_buttons_enabled(False, False)
        self._set_group_attrs(None)
        self._set_point_attrs(None)

    def _build_ui(self):
        root = QVBoxLayout()
        root.setSpacing(12)
        root.setContentsMargins(16, 16, 16, 16)
        self.setLayout(root)

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

        top_row = QHBoxLayout()
        top_row.setSpacing(10)
        top_row.addWidget(QLabel("选择地图："))
        top_row.addWidget(self._combo_maps)
        top_row.addWidget(self._btn_refresh_maps)
        top_row.addWidget(self._btn_fetch_reset)
        top_row.addWidget(self._btn_save)
        top_row.addStretch()
        task_layout.addLayout(top_row)
        task_layout.addSpacing(15)

        list_row = QHBoxLayout()
        list_row.setSpacing(8)

        # 任务组列表
        group_col = QVBoxLayout()
        group_col.setSpacing(10)
        group_col.setContentsMargins(0, 0, 0, 0)
        group_title = self._make_list_title("任务组列表")
        group_col.addWidget(group_title)
        self._list_groups.setFixedHeight(100)
        group_col.addWidget(self._list_groups, 1)
        list_row.addLayout(group_col, 5)

        # 任务点列表
        point_col = QVBoxLayout()
        point_col.setSpacing(10)
        point_col.setContentsMargins(0, 0, 0, 0)
        point_title = self._make_list_title("任务点列表")
        point_col.addWidget(point_title)
        self._list_points.setFixedHeight(100)
        point_col.addWidget(self._list_points, 1)
        list_row.addLayout(point_col, 5)

        # 任务组属性
        group_attr_col = QVBoxLayout()
        group_attr_col.setSpacing(10)
        group_attr_title = self._make_list_title("任务组属性")
        group_attr_title.setObjectName("muted")
        group_attr_col.addWidget(group_attr_title)
        group_attr_col.addWidget(self._group_info)
        list_row.addLayout(group_attr_col, 4)
        
        # 任务点属性
        point_attr_col = QVBoxLayout()
        point_attr_col.setSpacing(10)
        point_attr_title = self._make_list_title("任务点属性")
        point_attr_title.setObjectName("muted")
        point_attr_col.addWidget(point_attr_title)
        point_attr_col.addWidget(self._point_info)
        list_row.addLayout(point_attr_col, 4)

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

        self._btn_group_new = self._make_compact_btn("新建任务组")
        self._btn_group_delete = self._make_compact_btn("删除任务组")
        self._btn_group_up = self._make_compact_btn("排序上移")
        self._btn_group_rename = self._make_compact_btn("任务组重命名")
        self._btn_group_desc = self._make_compact_btn("编辑描述")
        self._btn_group_down = self._make_compact_btn("排序下移")

        grid.addWidget(self._btn_group_new, 0, 0)
        grid.addWidget(self._btn_group_delete, 0, 1)
        grid.addWidget(self._btn_group_up, 0, 2)
        grid.addWidget(self._btn_group_rename, 1, 0)
        grid.addWidget(self._btn_group_desc, 1, 1)
        grid.addWidget(self._btn_group_down, 1, 2)
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

        self._btn_point_new = self._make_compact_btn("新建任务点")
        self._btn_point_delete = self._make_compact_btn("删除任务点")
        self._btn_point_up = self._make_compact_btn("排序上移")
        self._btn_point_rename = self._make_compact_btn("任务点重命名")
        self._btn_point_pose = self._make_compact_btn("位姿编辑")
        self._btn_point_down = self._make_compact_btn("排序下移")

        grid.addWidget(self._btn_point_new, 0, 0)
        grid.addWidget(self._btn_point_delete, 0, 1)
        grid.addWidget(self._btn_point_up, 0, 2)
        grid.addWidget(self._btn_point_rename, 1, 0)
        grid.addWidget(self._btn_point_pose, 1, 1)
        grid.addWidget(self._btn_point_down, 1, 2)
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

        self._btn_fetch_reset.clicked.connect(self._on_fetch_tasks)
        self._btn_save.clicked.connect(self._on_save_tasks)

        if self._btn_group_new:
            self._btn_group_new.clicked.connect(self._on_group_add)
        if self._btn_group_delete:
            self._btn_group_delete.clicked.connect(self._on_group_delete)
        if self._btn_group_up:
            self._btn_group_up.clicked.connect(self._on_group_move_up)
        if self._btn_group_down:
            self._btn_group_down.clicked.connect(self._on_group_move_down)
        if self._btn_group_rename:
            self._btn_group_rename.clicked.connect(self._on_group_rename)
        if self._btn_group_desc:
            self._btn_group_desc.clicked.connect(self._on_group_edit_desc)

        if self._btn_point_new:
            self._btn_point_new.clicked.connect(self._on_point_add)
        if self._btn_point_delete:
            self._btn_point_delete.clicked.connect(self._on_point_delete)
        if self._btn_point_up:
            self._btn_point_up.clicked.connect(self._on_point_move_up)
        if self._btn_point_down:
            self._btn_point_down.clicked.connect(self._on_point_move_down)
        if self._btn_point_rename:
            self._btn_point_rename.clicked.connect(self._on_point_rename)
        if self._btn_point_pose:
            self._btn_point_pose.clicked.connect(self._on_point_edit_pose)

        self._list_groups.currentRowChanged.connect(self._on_group_selected)
        self._list_points.currentRowChanged.connect(self._on_point_selected)

    def _set_buttons_enabled(self, fetch: bool, save: bool):
        self._btn_fetch_reset.setEnabled(fetch)
        self._btn_save.setEnabled(save)
        self._update_group_action_state()
        self._update_point_action_state()

    def _update_group_action_state(self):
        has_tasks = bool(self._tasks_loaded)
        row = self._list_groups.currentRow()
        has_sel = has_tasks and 0 <= row < len(self._task_groups)

        if self._btn_group_new:
            self._btn_group_new.setEnabled(has_tasks)
        if self._btn_group_delete:
            self._btn_group_delete.setEnabled(has_sel)
        if self._btn_group_rename:
            self._btn_group_rename.setEnabled(has_sel)
        if self._btn_group_desc:
            self._btn_group_desc.setEnabled(has_sel)
        if self._btn_group_up:
            self._btn_group_up.setEnabled(has_sel and row > 0)
        if self._btn_group_down:
            self._btn_group_down.setEnabled(has_sel and row < len(self._task_groups) - 1)

    def _update_point_action_state(self):
        has_tasks = bool(self._tasks_loaded)
        group_row = self._list_groups.currentRow()
        has_group = has_tasks and 0 <= group_row < len(self._task_groups)
        point_row = self._list_points.currentRow()
        has_point = has_group and 0 <= point_row < len(self._current_points)

        if self._btn_point_new:
            self._btn_point_new.setEnabled(has_group)
        if self._btn_point_delete:
            self._btn_point_delete.setEnabled(has_point)
        if self._btn_point_rename:
            self._btn_point_rename.setEnabled(has_point)
        if self._btn_point_pose:
            self._btn_point_pose.setEnabled(has_point)
        if self._btn_point_up:
            self._btn_point_up.setEnabled(has_point and point_row > 0)
        if self._btn_point_down:
            self._btn_point_down.setEnabled(has_point and point_row < len(self._current_points) - 1)

    def _mark_dirty(self):
        self._dirty = True

    def _confirm_discard_changes(self, action: str) -> bool:
        if not self._dirty:
            return True
        reply = QMessageBox.question(
            self,
            "确认操作",
            f"任务信息已修改，{action}会丢失未保存的改动，是否继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        return reply == QMessageBox.Yes

    def _confirm_save_changes(self) -> bool:
        if not self._dirty:
            return True
        reply = QMessageBox.question(
            self,
            "确认保存",
            "任务信息已修改，确定要保存吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )
        return reply == QMessageBox.Yes

    def _normalize_task_groups(self):
        groups: List[dict] = []
        for idx, g in enumerate(self._task_groups):
            if not isinstance(g, dict):
                g = {}
            g["id"] = idx
            if "name" not in g:
                g["name"] = "未命名"
            if "description" not in g:
                g["description"] = ""
            points = g.get("points", [])
            if not isinstance(points, list):
                points = []
            norm_points: List[dict] = []
            for p_idx, p in enumerate(points):
                if not isinstance(p, dict):
                    p = {}
                p["id"] = p_idx
                if "name" not in p:
                    p["name"] = ""
                norm_points.append(p)
            g["points"] = norm_points
            g["point_count"] = len(norm_points)
            groups.append(g)
        self._task_groups = groups

    def _sync_tasks_yaml(self) -> bool:
        try:
            import yaml
        except Exception as e:
            QMessageBox.critical(self, "错误", f"缺少YAML解析库，无法同步任务信息：\n{str(e)}")
            return False

        if not isinstance(self._tasks_data, dict):
            self._tasks_data = {}
        self._tasks_data["task_groups"] = self._task_groups
        try:
            self._tasks_yaml = yaml.safe_dump(self._tasks_data, allow_unicode=True, sort_keys=False)
            return True
        except Exception as e:
            QMessageBox.critical(self, "错误", f"生成任务信息失败：\n{str(e)}")
            return False

    def _validate_group_name(self, name: str, exclude_index: Optional[int] = None) -> Optional[str]:
        name = (name or "").strip()
        if not name:
            return "任务组名称不能为空"
        for i, g in enumerate(self._task_groups):
            if exclude_index is not None and i == exclude_index:
                continue
            if str(g.get("name", "")).strip() == name:
                return f"任务组名称重复：{name}"
        return None

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
            self._set_buttons_enabled(False, False)
            self._clear_task_views()

    def _on_map_changed(self, idx: int):
        if self._ignore_map_change:
            return
        if not self._confirm_discard_changes("切换地图"):
            if self._last_map_index >= 0:
                self._ignore_map_change = True
                self._combo_maps.setCurrentIndex(self._last_map_index)
                self._ignore_map_change = False
            return

        self._last_map_index = idx
        has_map = bool(self._current_map_name())
        self._set_buttons_enabled(has_map, False)
        self._tasks_yaml = ""
        self._tasks_data = {}
        self._tasks_loaded = False
        self._dirty = False
        self._clear_task_views()

    def _on_fetch_tasks(self):
        map_name = self._current_map_name()
        if not map_name:
            QMessageBox.warning(self, "提示", "请先选择地图")
            return
        if self._tasks_loaded and self._dirty:
            if not self._confirm_discard_changes("获取/重置任务信息"):
                return

        tasks_yaml = self._ros.get_map_tasks(map_name)
        if tasks_yaml is None:
            return

        self._tasks_yaml = tasks_yaml
        data = self._parse_tasks_yaml(tasks_yaml)
        if data is None:
            return

        self._tasks_data = data
        self._task_groups = list(data.get("task_groups", []) or [])
        self._normalize_task_groups()
        self._refresh_group_list(select_index=-1)
        self._tasks_loaded = True
        self._dirty = False
        self._set_buttons_enabled(True, True)

    def _on_save_tasks(self):
        map_name = self._current_map_name()
        if not map_name:
            QMessageBox.warning(self, "提示", "请先选择地图")
            return
        if not self._tasks_yaml:
            QMessageBox.warning(self, "提示", "尚未获取任务信息")
            return
        if not self._confirm_save_changes():
            return

        ok = self._ros.set_map_tasks(map_name, self._tasks_yaml)
        if ok:
            self._dirty = False

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
        self._normalize_task_groups()
        self._refresh_group_list(select_index=-1)

    def _refresh_group_list(self, select_index: int = -1):
        self._list_groups.blockSignals(True)
        self._list_groups.clear()
        for g in self._task_groups:
            if isinstance(g, dict):
                gid = g.get("id", "--")
                name = g.get("name", "未命名")
            else:
                gid = "--"
                name = "未命名"
            self._list_groups.addItem(f"[{gid}] {name}")
        self._list_groups.blockSignals(False)

        if select_index is None or select_index < 0 or select_index >= len(self._task_groups):
            self._list_groups.setCurrentRow(-1)
            self._set_group_attrs(None)
            self._load_point_list([])
        else:
            self._list_groups.setCurrentRow(select_index)
            self._on_group_selected(select_index)
        self._update_group_action_state()

    def _load_point_list(self, points: List[dict]):
        self._refresh_point_list(points, select_index=-1)

    def _refresh_point_list(self, points: List[dict], select_index: int = -1):
        self._current_points = points if isinstance(points, list) else []
        self._list_points.blockSignals(True)
        self._list_points.clear()
        for p in self._current_points:
            if isinstance(p, dict):
                pid = p.get("id", "--")
                name = p.get("name", "")
                name = "未命名" if not str(name).strip() else str(name)
            else:
                pid = "--"
                name = "未命名"
            self._list_points.addItem(f"[{pid}] {name}")
        self._list_points.blockSignals(False)

        if select_index is None or select_index < 0 or select_index >= len(self._current_points):
            self._list_points.setCurrentRow(-1)
            self._set_point_attrs(None)
        else:
            self._list_points.setCurrentRow(select_index)
            self._on_point_selected(select_index)
        self._update_point_action_state()

    def _on_group_selected(self, row: int):
        if row < 0 or row >= len(self._task_groups):
            self._set_group_attrs(None)
            self._load_point_list([])
            self._update_group_action_state()
            self._update_point_action_state()
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
        self._update_group_action_state()
        self._update_point_action_state()

    def _on_point_selected(self, row: int):
        if row < 0 or row >= len(self._current_points):
            self._set_point_attrs(None)
            self._update_point_action_state()
            return

        point = self._current_points[row]
        if not isinstance(point, dict):
            self._set_point_attrs(None)
            return

        self._set_point_attrs(point)
        self._update_point_action_state()

    def _on_point_add(self):
        group_idx = self._list_groups.currentRow()
        if group_idx < 0 or group_idx >= len(self._task_groups):
            QMessageBox.warning(self, "提示", "请先选择任务组")
            return

        mode = self._choose_pose_input_method("新建任务点")
        if mode is None:
            return
        if mode == "map":
            QMessageBox.information(self, "提示", "地图选点功能待接入")
            return

        pose = self._input_pose_manual(defaults=(0.0, 0.0, 0.0))
        if pose is None:
            return
        x, y, theta = pose
        if not self._validate_pose_range(x, y, theta):
            QMessageBox.warning(self, "错误", "位姿超出地图范围（接口预留）")
            return

        group = self._task_groups[group_idx]
        points = group.get("points", [])
        if not isinstance(points, list):
            points = []
        points.append(
            {
                "id": len(points),
                "x": float(x),
                "y": float(y),
                "theta": float(theta),
                "name": "",
            }
        )
        group["points"] = points
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()

        group = self._task_groups[group_idx]
        self._set_group_attrs(group)
        new_index = len(group.get("points", [])) - 1
        self._refresh_point_list(group.get("points", []), select_index=new_index)

    def _on_point_delete(self):
        group_idx = self._list_groups.currentRow()
        row = self._list_points.currentRow()
        if group_idx < 0 or group_idx >= len(self._task_groups):
            return
        points = self._task_groups[group_idx].get("points", [])
        if not isinstance(points, list) or row < 0 or row >= len(points):
            return

        reply = QMessageBox.question(
            self,
            "确认删除",
            "确定要删除该任务点吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        points.pop(row)
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()

        group = self._task_groups[group_idx]
        self._set_group_attrs(group)
        next_index = min(row, len(group.get("points", [])) - 1)
        self._refresh_point_list(group.get("points", []), select_index=next_index)

    def _on_point_move_up(self):
        group_idx = self._list_groups.currentRow()
        row = self._list_points.currentRow()
        if group_idx < 0 or group_idx >= len(self._task_groups):
            return
        points = self._task_groups[group_idx].get("points", [])
        if not isinstance(points, list) or row <= 0 or row >= len(points):
            return

        points[row - 1], points[row] = points[row], points[row - 1]
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()

        group = self._task_groups[group_idx]
        self._set_group_attrs(group)
        self._refresh_point_list(group.get("points", []), select_index=row - 1)

    def _on_point_move_down(self):
        group_idx = self._list_groups.currentRow()
        row = self._list_points.currentRow()
        if group_idx < 0 or group_idx >= len(self._task_groups):
            return
        points = self._task_groups[group_idx].get("points", [])
        if not isinstance(points, list) or row < 0 or row >= len(points) - 1:
            return

        points[row + 1], points[row] = points[row], points[row + 1]
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()

        group = self._task_groups[group_idx]
        self._set_group_attrs(group)
        self._refresh_point_list(group.get("points", []), select_index=row + 1)

    def _on_point_rename(self):
        group_idx = self._list_groups.currentRow()
        row = self._list_points.currentRow()
        if group_idx < 0 or group_idx >= len(self._task_groups):
            return
        points = self._task_groups[group_idx].get("points", [])
        if not isinstance(points, list) or row < 0 or row >= len(points):
            return

        current_name = str(points[row].get("name", ""))
        name, ok = QInputDialog.getText(self, "任务点重命名", "请输入任务点名称（可为空）：", text=current_name)
        if not ok:
            return
        name = str(name or "")
        if name == current_name:
            return

        points[row]["name"] = name
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()

        group = self._task_groups[group_idx]
        self._set_group_attrs(group)
        self._refresh_point_list(group.get("points", []), select_index=row)

    def _on_point_edit_pose(self):
        group_idx = self._list_groups.currentRow()
        row = self._list_points.currentRow()
        if group_idx < 0 or group_idx >= len(self._task_groups):
            return
        points = self._task_groups[group_idx].get("points", [])
        if not isinstance(points, list) or row < 0 or row >= len(points):
            return

        mode = self._choose_pose_input_method("编辑位姿")
        if mode is None:
            return
        if mode == "map":
            QMessageBox.information(self, "提示", "地图选点功能待接入")
            return

        p = points[row]
        defaults = (
            float(p.get("x", 0.0) or 0.0),
            float(p.get("y", 0.0) or 0.0),
            float(p.get("theta", 0.0) or 0.0),
        )
        pose = self._input_pose_manual(defaults=defaults)
        if pose is None:
            return
        x, y, theta = pose
        if not self._validate_pose_range(x, y, theta):
            QMessageBox.warning(self, "错误", "位姿超出地图范围（接口预留）")
            return

        p["x"] = float(x)
        p["y"] = float(y)
        p["theta"] = float(theta)
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()

        group = self._task_groups[group_idx]
        self._set_group_attrs(group)
        self._refresh_point_list(group.get("points", []), select_index=row)

    def _on_group_add(self):
        if not self._tasks_loaded:
            QMessageBox.warning(self, "提示", "请先获取任务信息")
            return

        name, ok = QInputDialog.getText(self, "新建任务组", "请输入任务组名称：")
        if not ok:
            return
        name = (name or "").strip()
        err = self._validate_group_name(name)
        if err:
            QMessageBox.warning(self, "错误", err)
            return

        new_group = {
            "id": len(self._task_groups),
            "name": name,
            "description": "",
            "point_count": 0,
            "points": [],
        }
        self._task_groups.append(new_group)
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()

        new_index = len(self._task_groups) - 1
        self._refresh_group_list(select_index=new_index)

    def _on_group_delete(self):
        row = self._list_groups.currentRow()
        if row < 0 or row >= len(self._task_groups):
            return

        name = str(self._task_groups[row].get("name", ""))
        reply = QMessageBox.question(
            self,
            "确认删除",
            f'确定要删除任务组 "{name}" 吗？',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        self._task_groups.pop(row)
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()

        next_index = min(row, len(self._task_groups) - 1)
        self._refresh_group_list(select_index=next_index)

    def _on_group_move_up(self):
        row = self._list_groups.currentRow()
        if row <= 0 or row >= len(self._task_groups):
            return

        self._task_groups[row - 1], self._task_groups[row] = (
            self._task_groups[row],
            self._task_groups[row - 1],
        )
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()
        self._refresh_group_list(select_index=row - 1)

    def _on_group_move_down(self):
        row = self._list_groups.currentRow()
        if row < 0 or row >= len(self._task_groups) - 1:
            return

        self._task_groups[row + 1], self._task_groups[row] = (
            self._task_groups[row],
            self._task_groups[row + 1],
        )
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()
        self._refresh_group_list(select_index=row + 1)

    def _on_group_rename(self):
        row = self._list_groups.currentRow()
        if row < 0 or row >= len(self._task_groups):
            return

        current_name = str(self._task_groups[row].get("name", ""))
        name, ok = QInputDialog.getText(self, "任务组重命名", "请输入新的任务组名称：", text=current_name)
        if not ok:
            return
        name = (name or "").strip()
        if name == current_name:
            return

        err = self._validate_group_name(name, exclude_index=row)
        if err:
            QMessageBox.warning(self, "错误", err)
            return

        self._task_groups[row]["name"] = name
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()
        self._refresh_group_list(select_index=row)

    def _on_group_edit_desc(self):
        row = self._list_groups.currentRow()
        if row < 0 or row >= len(self._task_groups):
            return

        current_desc = str(self._task_groups[row].get("description", ""))
        desc, ok = QInputDialog.getText(self, "编辑描述", "请输入任务组描述（最多100字）：", text=current_desc)
        if not ok:
            return
        desc = (desc or "").strip()
        if len(desc) > 100:
            QMessageBox.warning(self, "错误", "描述长度不能超过100字")
            return

        self._task_groups[row]["description"] = desc
        self._normalize_task_groups()
        if not self._sync_tasks_yaml():
            return
        self._mark_dirty()
        self._refresh_group_list(select_index=row)

    def _clear_task_views(self):
        self._list_groups.blockSignals(True)
        self._list_groups.clear()
        self._list_groups.blockSignals(False)

        self._list_points.blockSignals(True)
        self._list_points.clear()
        self._list_points.blockSignals(False)

        self._set_group_attrs(None)
        self._set_point_attrs(None)
        self._update_group_action_state()
        self._update_point_action_state()

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
            f"任务点数量: {point_count}\n"
            f"描述: {desc}"
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

    def _choose_pose_input_method(self, title: str) -> Optional[str]:
        msg = QMessageBox(self)
        msg.setWindowTitle(title)
        msg.setText("请选择位姿输入方式：")
        btn_map = msg.addButton("地图选点", QMessageBox.ActionRole)
        btn_manual = msg.addButton("手动输入", QMessageBox.ActionRole)
        msg.addButton("取消", QMessageBox.RejectRole)
        msg.exec_()

        clicked = msg.clickedButton()
        if clicked == btn_map:
            return "map"
        if clicked == btn_manual:
            return "manual"
        return None

    def _input_pose_manual(self, defaults=(0.0, 0.0, 0.0)) -> Optional[tuple]:
        x0, y0, t0 = defaults
        x, ok = QInputDialog.getDouble(self, "手动输入", "X:", float(x0), -1e9, 1e9, 4)
        if not ok:
            return None
        y, ok = QInputDialog.getDouble(self, "手动输入", "Y:", float(y0), -1e9, 1e9, 4)
        if not ok:
            return None
        theta, ok = QInputDialog.getDouble(self, "手动输入", "Theta:", float(t0), -1e9, 1e9, 4)
        if not ok:
            return None
        return x, y, theta

    def _validate_pose_range(self, x: float, y: float, theta: float) -> bool:
        # 预留地图范围校验接口（后续使用地图范围进行约束）
        return True

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
