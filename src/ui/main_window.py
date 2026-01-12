#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主窗口（三栏布局）
- 左：菜单（软件名+页面）
- 中：页面内容（QStackedWidget）
- 右：GUI自身日志输出
- 底：状态栏（manager运行、slam状态、性能等）
"""

from __future__ import annotations

import logging

import rospy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QMainWindow,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QLabel,
    QToolButton,
    QButtonGroup,
    QStackedWidget,
    QMessageBox,
    QSplitter,
)

from ..config.settings import config
from ..core.ros_manager import ROSServiceManager
from ..core.server_manager import ServerProcessManager
from ..core.status_thread import StatusUpdateThread
from ..utils.gui_logging import setup_gui_logger
from ..utils.system_metrics import CpuUsageSampler, read_mem_percent, read_jetson_gpu_percent
from .styles import DarkTheme
from .pages.system_status_page import SystemStatusPage
from .pages.map_management_page import MapManagementPage
from .pages.placeholder_page import PlaceholderPage
from .widgets.status_bar import AppStatusBar
from .widgets.log_panel import LogPanel


class SlamMainWindow(QMainWindow):
    """SLAM系统主控界面（三栏）"""

    def __init__(self):
        super().__init__()

        # 初始化ROS节点（保持与之前一致）
        try:
            rospy.init_node("slam_gui", anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException:
            pass

        self.setWindowTitle(config.WINDOW_TITLE)
        self.setGeometry(config.WINDOW_POS_X, config.WINDOW_POS_Y, config.WINDOW_WIDTH, config.WINDOW_HEIGHT)

        # 管理器
        self.ros_manager = ROSServiceManager(self)
        self.server_manager = ServerProcessManager(self)

        # 右侧日志面板（仅GUI自身日志）
        self.log_panel = LogPanel()
        self.logger, self._log_signal, self._std_guard = setup_gui_logger(
            self.log_panel.append_line, logger_name="slam_gui", level=logging.INFO, redirect_std=True
        )

        # 性能采样
        self._cpu_sampler = CpuUsageSampler()
        self._last_cpu = None
        self._last_mem = None
        self._last_gpu = None

        # 当前slam状态
        self.current_status = "disconnected"
        self.current_message = ""
        self.current_uptime = 0

        # UI
        self._build_ui()
        self.setStyleSheet(DarkTheme.get_stylesheet())

        # 状态线程
        self.status_thread = StatusUpdateThread()
        self.status_thread.status_updated.connect(self.on_status_updated)
        self.status_thread.start()

        # 性能定时刷新
        self._metrics_timer = QTimer(self)
        self._metrics_timer.timeout.connect(self._update_metrics)
        self._metrics_timer.start(1000)

        self.logger.info("GUI启动完成")

    def _build_ui(self):
        root = QWidget()
        root.setObjectName("appRoot")
        self.setCentralWidget(root)

        main = QVBoxLayout()
        main.setContentsMargins(0, 0, 0, 0)
        main.setSpacing(0)
        root.setLayout(main)

        # 三栏主体
        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        left = self._build_sidebar()
        center = self._build_center()
        right = self.log_panel

        splitter.addWidget(left)
        splitter.addWidget(center)
        splitter.addWidget(right)

        splitter.setStretchFactor(0, config.LAYOUT_RATIO_LEFT)
        splitter.setStretchFactor(1, config.LAYOUT_RATIO_CENTER)
        splitter.setStretchFactor(2, config.LAYOUT_RATIO_RIGHT)

        main.addWidget(splitter, 1)

        # 底部状态栏
        self.status_bar = AppStatusBar()
        main.addWidget(self.status_bar, 0)

    def _build_sidebar(self) -> QWidget:
        sidebar = QWidget()
        sidebar.setObjectName("sidebar")

        layout = QVBoxLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        sidebar.setLayout(layout)

        name = QLabel("KUAVO SLAM")
        name.setObjectName("appName")
        subtitle = QLabel("控制中心")
        subtitle.setObjectName("appSubtitle")
        layout.addWidget(name)
        layout.addWidget(subtitle)

        self._nav_group = QButtonGroup(self)
        self._nav_group.setExclusive(True)

        self._btn_status = self._make_nav_btn("系统状态")
        self._btn_nav = self._make_nav_btn("导航管理")
        self._btn_map = self._make_nav_btn("地图管理")
        self._btn_task = self._make_nav_btn("任务管理")

        for i, b in enumerate([self._btn_status, self._btn_nav, self._btn_map, self._btn_task]):
            self._nav_group.addButton(b, i)
            layout.addWidget(b)

        layout.addStretch()

        self._nav_group.buttonClicked[int].connect(self._on_nav_clicked)
        self._btn_status.setChecked(True)
        return sidebar

    def _make_nav_btn(self, text: str) -> QToolButton:
        b = QToolButton()
        b.setText(text)
        b.setCheckable(True)
        b.setObjectName("navBtn")
        b.setToolButtonStyle(Qt.ToolButtonTextOnly)
        return b

    def _build_center(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        container.setLayout(layout)

        self.pages = QStackedWidget()

        self.page_system = SystemStatusPage(self.server_manager)
        self.page_map = MapManagementPage(self.ros_manager)
        self.page_nav = PlaceholderPage("导航管理")
        self.page_task = PlaceholderPage("任务管理")

        self.pages.addWidget(self.page_system)  # 0
        self.pages.addWidget(self.page_nav)     # 1
        self.pages.addWidget(self.page_map)     # 2
        self.pages.addWidget(self.page_task)    # 3

        layout.addWidget(self.pages, 1)
        return container

    def _on_nav_clicked(self, idx: int):
        self.pages.setCurrentIndex(idx)
        # 给日志留个痕迹
        names = ["系统状态", "导航管理", "地图管理", "任务管理"]
        if 0 <= idx < len(names):
            self.logger.info("切换页面：%s", names[idx])

    def _update_metrics(self):
        cpu = self._cpu_sampler.sample_percent()
        mem = read_mem_percent()
        gpu = read_jetson_gpu_percent()

        self._last_cpu = cpu if cpu is not None else self._last_cpu
        self._last_mem = mem if mem is not None else self._last_mem
        self._last_gpu = gpu if gpu is not None else self._last_gpu

        self.page_system.set_metrics(cpu, mem, gpu)
        self.status_bar.set_perf(cpu, mem, gpu)

    def on_status_updated(self, status: str, message: str, uptime: int):
        self.current_status = status or "disconnected"
        self.current_message = message or ""
        self.current_uptime = int(uptime or 0)

        manager_running = bool(self.server_manager.is_running())
        self.status_bar.set_manager_running(manager_running)
        self.status_bar.set_slam_status(self.current_status)
        self.status_bar.set_uptime(self.current_uptime)

        self.page_system.set_states(manager_running, self.current_status)
        self.page_map.set_mapping_state(self.current_status)

    def show_message(self, title: str, message: str, icon=QMessageBox.Information):
        msg_box = QMessageBox(self)
        msg_box.setIcon(icon)
        msg_box.setWindowTitle(title)
        msg_box.setText(message)
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.setStyleSheet(DarkTheme.get_messagebox_style())
        msg_box.exec_()

    def closeEvent(self, event):
        # 停止状态线程
        if hasattr(self, "status_thread"):
            self.status_thread.stop()
            self.status_thread.wait(1000)

        # 清理服务端资源
        if hasattr(self, "server_manager"):
            self.server_manager.cleanup()

        # 恢复stdout/stderr
        if hasattr(self, "_std_guard") and self._std_guard:
            self._std_guard.restore()

        event.accept()


