#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SLAM GUI 配置文件
集中管理所有配置项
"""

import os


class Config:
    """全局配置类"""
    
    # ==================== 路径配置 ====================
    # SLAM 工作空间路径
    SLAM_WS = "/media/data/slam_ws"
    
    # 脚本路径
    SLAM_MANAGER_SCRIPT = os.path.join(
        SLAM_WS, 
        "src/slam_controller/scripts/run_slam_manager.sh"
    )
    
    # PID 文件路径
    SERVER_PID_FILE = "/tmp/slam_manager_terminal.pid"
    
    # ==================== ROS 服务配置 ====================
    # 服务名称
    SERVICE_START_MAPPING = "/slam_manager/start_mapping"
    SERVICE_STOP_MAPPING = "/slam_manager/stop_mapping"
    SERVICE_GET_STATUS = "/slam_manager/get_status"
    SERVICE_LIST_MAPS = "/slam_manager/list_maps"
    SERVICE_PROCESS_MAP = "/slam_manager/process_map"
    SERVICE_START_NAVIGATION = "/slam_manager/start_navigation"
    SERVICE_STOP_NAVIGATION = "/slam_manager/stop_navigation"
    SERVICE_GET_MAP_TASKS = "/slam_manager/get_map_tasks"
    SERVICE_SET_MAP_TASKS = "/slam_manager/set_map_tasks"
    
    # 服务超时时间（秒）
    SERVICE_TIMEOUT = 2.0
    
    # 状态更新频率（Hz）
    STATUS_UPDATE_RATE = 2
    
    # ==================== UI 配置 ====================
    # 窗口标题
    WINDOW_TITLE = "Kuavo SLAM 控制中心"
    
    # 窗口尺寸
    WINDOW_WIDTH = 1280
    WINDOW_HEIGHT = 720
    WINDOW_POS_X = 100
    WINDOW_POS_Y = 100

    # 横屏三栏布局比例（左/中/右）
    LAYOUT_RATIO_LEFT = 2     # 约 20%
    LAYOUT_RATIO_CENTER = 5   # 约 50%
    LAYOUT_RATIO_RIGHT = 3    # 约 30%

    # 底部状态栏高度
    STATUS_BAR_HEIGHT = 32
    
    # 按钮高度
    BUTTON_HEIGHT_LARGE = 60  # 建图按钮
    BUTTON_HEIGHT_MEDIUM = 50  # 服务端按钮
    
    # ==================== 状态配置 ====================
    # 状态颜色映射（科技风格）
    STATUS_COLORS = {
        'idle': '#00e676',          # 霓虹绿 - 空闲
        'mapping': '#00d4ff',       # 霓虹蓝 - 建图中
        'localizing': '#ffa726',    # 橙色 - 定位中
        'navigating': '#ab47bc',    # 紫色 - 导航中
        'error': '#ff5252',         # 红色 - 错误
        'disconnected': '#607d8b'   # 灰色 - 断开连接
    }
    
    # 状态文本映射
    STATUS_TEXT = {
        'idle': '空闲',
        'mapping': '建图中',
        'localizing': '定位中',
        'navigating': '导航中',
        'error': '错误',
        'disconnected': '未连接'
    }
    
    # ==================== 终端配置 ====================
    # 优先使用的终端列表
    TERMINAL_PRIORITY = [
        'gnome-terminal',
        'x-terminal-emulator',
        'xterm'
    ]
    
    # ==================== 进程管理配置 ====================
    # 停止服务端时等待进程退出的最大时间（秒）
    PROCESS_STOP_TIMEOUT = 5
    
    # 进程状态检查间隔（秒）
    PROCESS_CHECK_INTERVAL = 0.1


# 导出配置实例
config = Config()

