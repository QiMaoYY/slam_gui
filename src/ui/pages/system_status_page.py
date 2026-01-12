#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
系统状态页面
- 上：性能（CPU/MEM/GPU）
- 中：SLAM服务端状态与按钮
- 下：系统信息
"""

from __future__ import annotations

from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QFrame,
)

from ...core.server_manager import ServerProcessManager
from ...utils.system_info import read_system_info


class SystemStatusPage(QWidget):
    def __init__(self, server_manager: ServerProcessManager, parent=None):
        super().__init__(parent)
        self._server_manager = server_manager

        self._cpu_value = QLabel("--%")
        self._mem_value = QLabel("--%")
        self._gpu_value = QLabel("--%")

        self._server_state = QLabel("未知")
        self._slam_state = QLabel("未知")

        self._btn_start_server = QPushButton("启动服务端")
        self._btn_start_server.setObjectName("startServerBtn")
        self._btn_stop_server = QPushButton("关闭服务端")
        self._btn_stop_server.setObjectName("stopServerBtn")

        self._sys_info = QLabel("")
        self._sys_info.setWordWrap(True)

        self._build_ui()
        self._wire()
        self.refresh_system_info()

    def _build_ui(self):
        root = QVBoxLayout()
        root.setSpacing(12)
        root.setContentsMargins(16, 16, 16, 16)
        self.setLayout(root)

        # 性能区
        perf_card = QFrame()
        perf_card.setObjectName("card")
        perf_layout = QVBoxLayout()
        perf_layout.setContentsMargins(16, 16, 16, 16)
        perf_layout.setSpacing(10)
        perf_card.setLayout(perf_layout)

        title = QLabel("系统性能")
        title.setObjectName("sectionTitle")
        perf_layout.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(18)
        row.addLayout(self._kv("CPU占用率", self._cpu_value))
        row.addLayout(self._kv("内存占用率", self._mem_value))
        row.addLayout(self._kv("GPU占用率", self._gpu_value))
        row.addStretch()
        perf_layout.addLayout(row)
        root.addWidget(perf_card)

        # 服务端状态
        server_card = QFrame()
        server_card.setObjectName("card")
        server_layout = QVBoxLayout()
        server_layout.setContentsMargins(16, 16, 16, 16)
        server_layout.setSpacing(10)
        server_card.setLayout(server_layout)

        title2 = QLabel("SLAM服务端")
        title2.setObjectName("sectionTitle")
        server_layout.addWidget(title2)

        state_row = QHBoxLayout()
        state_row.setSpacing(18)
        state_row.addLayout(self._kv("Manager状态", self._server_state))
        state_row.addLayout(self._kv("SLAM节点状态", self._slam_state))
        state_row.addStretch()
        server_layout.addLayout(state_row)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        btn_row.addWidget(self._btn_start_server)
        btn_row.addWidget(self._btn_stop_server)
        btn_row.addStretch()
        server_layout.addLayout(btn_row)

        root.addWidget(server_card)

        # 系统信息
        info_card = QFrame()
        info_card.setObjectName("card")
        info_layout = QVBoxLayout()
        info_layout.setContentsMargins(16, 16, 16, 16)
        info_layout.setSpacing(10)
        info_card.setLayout(info_layout)

        title3 = QLabel("系统信息")
        title3.setObjectName("sectionTitle")
        info_layout.addWidget(title3)

        self._sys_info.setObjectName("muted")
        info_layout.addWidget(self._sys_info)

        root.addWidget(info_card)
        root.addStretch()

    def _wire(self):
        self._btn_start_server.clicked.connect(self._server_manager.start)
        self._btn_stop_server.clicked.connect(self._server_manager.stop)

    @staticmethod
    def _kv(k: str, v_label: QLabel):
        v_label.setObjectName("statusItemStrong")
        k_label = QLabel(k)
        k_label.setObjectName("muted")
        col = QVBoxLayout()
        col.setSpacing(4)
        col.addWidget(k_label)
        col.addWidget(v_label)
        return col

    def refresh_system_info(self):
        info = read_system_info()
        lines = [
            f"系统版本：{info.os_pretty}",
            f"内核版本：{info.kernel}",
            f"架构：{info.arch}",
        ]
        if info.device_model:
            lines.append(f"设备型号：{info.device_model}")
        self._sys_info.setText("\n".join(lines))

    def set_metrics(self, cpu: Optional[float], mem: Optional[float], gpu: Optional[float]):
        self._cpu_value.setText("--%" if cpu is None else f"{cpu:.1f}%")
        self._mem_value.setText("--%" if mem is None else f"{mem:.1f}%")
        self._gpu_value.setText("--%" if gpu is None else f"{gpu:.1f}%")

    def set_states(self, manager_running: bool, slam_status: str):
        self._server_state.setText("已启动" if manager_running else "未启动")
        self._slam_state.setText(slam_status or "未知")

        # 按钮可用性（简单规则）
        self._btn_start_server.setEnabled(not manager_running)
        self._btn_stop_server.setEnabled(manager_running)


