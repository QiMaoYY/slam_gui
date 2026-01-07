#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主窗口UI
SLAM系统控制界面
"""

import re
import rospy
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QCheckBox, QMessageBox, QGroupBox,
    QInputDialog, QLineEdit
)
from PyQt5.QtCore import Qt

from ..config.settings import config
from ..core.status_thread import StatusUpdateThread
from ..core.ros_manager import ROSServiceManager
from ..core.server_manager import ServerProcessManager
from .styles import DarkTheme


class SlamMainWindow(QMainWindow):
    """SLAM系统主控界面"""
    
    def __init__(self):
        super().__init__()
        
        # 初始化ROS节点
        try:
            rospy.init_node('slam_gui', anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException:
            pass  # 节点已初始化
        
        # 当前状态
        self.current_status = 'disconnected'
        self.current_message = ''
        self.current_uptime = 0
        
        # 初始化管理器
        self.ros_manager = ROSServiceManager(self)
        self.server_manager = ServerProcessManager(self)
        
        # 初始化UI
        self.init_ui()
        
        # 启动状态更新线程
        self.status_thread = StatusUpdateThread()
        self.status_thread.status_updated.connect(self.on_status_updated)
        self.status_thread.start()
    
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle(config.WINDOW_TITLE)
        self.setGeometry(
            config.WINDOW_POS_X,
            config.WINDOW_POS_Y,
            config.WINDOW_WIDTH,
            config.WINDOW_HEIGHT
        )
        
        # 主窗口容器
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(30, 30, 30, 30)
        central_widget.setLayout(main_layout)
        
        # 标题
        title_label = QLabel('SLAM 系统控制面板')
        title_label.setObjectName('titleLabel')
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # 状态显示区域
        self.status_group = self.create_status_group()
        main_layout.addWidget(self.status_group)
        
        # 建图控制区域
        mapping_group = self.create_mapping_group()
        main_layout.addWidget(mapping_group)
        
        # 底部弹性空间
        main_layout.addStretch()
        
        # 服务端控制区域
        server_group = self.create_server_group()
        main_layout.addWidget(server_group)
        
        # 应用暗色主题样式
        self.setStyleSheet(DarkTheme.get_stylesheet())
    
    def create_status_group(self) -> QGroupBox:
        """创建状态显示组"""
        group = QGroupBox('系统状态')
        group.setObjectName('statusGroup')
        
        layout = QVBoxLayout()
        layout.setSpacing(15)
        
        # 状态指示器
        status_layout = QHBoxLayout()
        
        status_label_text = QLabel('当前状态:')
        status_label_text.setObjectName('statusLabelText')
        status_layout.addWidget(status_label_text)
        
        self.status_indicator = QLabel('未连接')
        self.status_indicator.setObjectName('statusIndicator')
        self.status_indicator.setAlignment(Qt.AlignCenter)
        self.status_indicator.setMinimumWidth(120)
        status_layout.addWidget(self.status_indicator)
        
        status_layout.addStretch()
        
        # 运行时间
        self.uptime_label = QLabel('运行时间: --')
        self.uptime_label.setObjectName('uptimeLabel')
        status_layout.addWidget(self.uptime_label)
        
        layout.addLayout(status_layout)
        
        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setObjectName('separator')
        layout.addWidget(line)
        
        # 状态消息
        self.message_label = QLabel('等待连接到SLAM Manager...')
        self.message_label.setObjectName('messageLabel')
        self.message_label.setWordWrap(True)
        layout.addWidget(self.message_label)
        
        group.setLayout(layout)
        return group
    
    def create_mapping_group(self) -> QGroupBox:
        """创建建图控制组"""
        group = QGroupBox('建图管理')
        group.setObjectName('mappingGroup')
        
        layout = QVBoxLayout()
        layout.setSpacing(15)
        
        # 校准选项
        self.calib_checkbox = QCheckBox('启用雷达校准（首次建图推荐）')
        self.calib_checkbox.setObjectName('calibCheckbox')
        self.calib_checkbox.setChecked(True)
        layout.addWidget(self.calib_checkbox)
        
        # 按钮布局
        button_layout = QHBoxLayout()
        button_layout.setSpacing(15)
        
        # 开始建图按钮
        self.start_mapping_btn = QPushButton('🚀 开始建图')
        self.start_mapping_btn.setObjectName('startMappingBtn')
        self.start_mapping_btn.setMinimumHeight(config.BUTTON_HEIGHT_LARGE)
        self.start_mapping_btn.clicked.connect(self.on_start_mapping)
        button_layout.addWidget(self.start_mapping_btn)
        
        # 保存地图按钮
        self.save_map_btn = QPushButton('💾 保存地图')
        self.save_map_btn.setObjectName('saveMapBtn')
        self.save_map_btn.setMinimumHeight(config.BUTTON_HEIGHT_LARGE)
        self.save_map_btn.setEnabled(False)
        self.save_map_btn.clicked.connect(self.on_save_map)
        button_layout.addWidget(self.save_map_btn)
        
        # 终止建图按钮
        self.abort_mapping_btn = QPushButton('⏹ 终止建图')
        self.abort_mapping_btn.setObjectName('abortMappingBtn')
        self.abort_mapping_btn.setMinimumHeight(config.BUTTON_HEIGHT_LARGE)
        self.abort_mapping_btn.setEnabled(False)
        self.abort_mapping_btn.clicked.connect(self.on_abort_mapping)
        button_layout.addWidget(self.abort_mapping_btn)
        
        layout.addLayout(button_layout)
        
        group.setLayout(layout)
        return group
    
    def create_server_group(self) -> QGroupBox:
        """创建服务端控制组"""
        group = QGroupBox('服务端控制')
        group.setObjectName('serverGroup')
        
        layout = QHBoxLayout()
        layout.setSpacing(20)
        
        # 启动服务端按钮
        self.start_server_btn = QPushButton('▶ 启动服务端')
        self.start_server_btn.setObjectName('startServerBtn')
        self.start_server_btn.setMinimumHeight(config.BUTTON_HEIGHT_MEDIUM)
        self.start_server_btn.clicked.connect(self.on_start_server)
        layout.addWidget(self.start_server_btn)
        
        # 停止服务端按钮
        self.stop_server_btn = QPushButton('⏹ 停止服务端')
        self.stop_server_btn.setObjectName('stopServerBtn')
        self.stop_server_btn.setMinimumHeight(config.BUTTON_HEIGHT_MEDIUM)
        self.stop_server_btn.clicked.connect(self.on_stop_server)
        layout.addWidget(self.stop_server_btn)
        
        group.setLayout(layout)
        return group
    
    def on_status_updated(self, status: str, message: str, uptime: int):
        """状态更新回调"""
        self.current_status = status
        self.current_message = message
        self.current_uptime = uptime
        
        # 更新状态指示器
        status_text = config.STATUS_TEXT.get(status, status)
        self.status_indicator.setText(status_text)
        
        # 设置状态颜色
        color = config.STATUS_COLORS.get(status, '#607d8b')
        self.status_indicator.setStyleSheet(f"""
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba({self._hex_to_rgb(color)}, 0.6),
                stop:1 rgba({self._hex_to_rgb(color)}, 0.8)
            );
            color: #ffffff;
            border: 1px solid rgba({self._hex_to_rgb(color)}, 0.9);
            border-radius: 10px;
            padding: 10px 20px;
            font-weight: 600;
            font-size: 14px;
            letter-spacing: 1px;
        """)
        
        # 更新消息
        self.message_label.setText(message)
        
        # 更新运行时间
        if uptime > 0:
            hours = uptime // 3600
            minutes = (uptime % 3600) // 60
            seconds = uptime % 60
            if hours > 0:
                time_str = f'{hours:02d}:{minutes:02d}:{seconds:02d}'
            else:
                time_str = f'{minutes:02d}:{seconds:02d}'
            self.uptime_label.setText(f'运行时间: {time_str}')
        else:
            self.uptime_label.setText('运行时间: --')
        
        # 根据状态更新按钮状态
        self.update_button_states(status)
    
    def update_button_states(self, status: str):
        """根据系统状态更新按钮可用性"""
        if status == 'idle':
            self.start_mapping_btn.setEnabled(True)
            self.save_map_btn.setEnabled(False)
            self.abort_mapping_btn.setEnabled(False)
            self.calib_checkbox.setEnabled(True)
            self.start_server_btn.setEnabled(False)
            self.stop_server_btn.setEnabled(True)
        elif status == 'mapping':
            self.start_mapping_btn.setEnabled(False)
            self.save_map_btn.setEnabled(True)
            self.abort_mapping_btn.setEnabled(True)
            self.calib_checkbox.setEnabled(False)
            self.start_server_btn.setEnabled(False)
            self.stop_server_btn.setEnabled(True)
        elif status in ['localizing', 'navigating']:
            self.start_mapping_btn.setEnabled(False)
            self.save_map_btn.setEnabled(False)
            self.abort_mapping_btn.setEnabled(False)
            self.calib_checkbox.setEnabled(False)
            self.start_server_btn.setEnabled(False)
            self.stop_server_btn.setEnabled(True)
        else:  # error or disconnected
            self.start_mapping_btn.setEnabled(False)
            self.save_map_btn.setEnabled(False)
            self.abort_mapping_btn.setEnabled(False)
            self.calib_checkbox.setEnabled(False)
            self.start_server_btn.setEnabled(True)
            self.stop_server_btn.setEnabled(False)
    
    def on_start_mapping(self):
        """开始建图按钮点击事件"""
        need_calibration = self.calib_checkbox.isChecked()
        self.ros_manager.start_mapping(need_calibration)
    
    def on_save_map(self):
        """保存地图按钮点击事件"""
        # 创建输入对话框并应用暗色主题
        dialog = QInputDialog(self)
        dialog.setWindowTitle('保存地图')
        dialog.setLabelText(
            '请输入地图名称:\n\n'
            '规则：\n'
            '- 只能包含字母、数字、下划线(_)、横线(-)\n'
            '- 长度不超过50个字符\n'
            '- 不能为空'
        )
        dialog.setTextValue('')
        dialog.setStyleSheet(DarkTheme.get_inputdialog_style())
        
        ok = dialog.exec_()
        map_name = dialog.textValue()
        
        if not ok:
            return
        
        # 验证地图名
        map_name = map_name.strip()
        
        if not map_name:
            self.show_message('错误', '地图名称不能为空', QMessageBox.Warning)
            return
        
        if len(map_name) > 50:
            self.show_message('错误', '地图名称长度不能超过50个字符', QMessageBox.Warning)
            return
        
        if not re.match(r'^[a-zA-Z0-9_-]+$', map_name):
            self.show_message(
                '错误', 
                '地图名称只能包含字母、数字、下划线(_)、横线(-)\n'
                f'您输入的名称包含非法字符: {map_name}',
                QMessageBox.Warning
            )
            return
        
        # 确认保存
        reply = QMessageBox.question(
            self,
            '确认保存',
            f'确定要停止建图并保存为 "{map_name}" 吗？\n\n'
            f'地图将保存到:\nkuavo_slam/maps/{map_name}/',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )
        
        if reply == QMessageBox.Yes:
            self.ros_manager.stop_mapping(save_map=True, map_name=map_name)
    
    def on_abort_mapping(self):
        """终止建图按钮点击事件（不保存地图）"""
        # 确认对话框
        reply = QMessageBox.question(
            self,
            '确认终止',
            '确定要终止建图吗？\n\n'
            '⚠️ 警告：地图将不会被保存！\n'
            '如需保存地图，请点击"保存地图"按钮。',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.ros_manager.stop_mapping(save_map=False, map_name="")
    
    def on_start_server(self):
        """启动服务端按钮点击事件"""
        self.server_manager.start()
    
    def on_stop_server(self):
        """停止服务端按钮点击事件"""
        # 确认对话框
        reply = QMessageBox.question(
            self,
            '确认停止',
            '确定要停止服务端吗？\n这将终止所有正在运行的SLAM任务。',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.server_manager.stop()
    
    def show_message(self, title: str, message: str, icon=QMessageBox.Information):
        """显示消息对话框"""
        msg_box = QMessageBox(self)
        msg_box.setIcon(icon)
        msg_box.setWindowTitle(title)
        msg_box.setText(message)
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.setStyleSheet(DarkTheme.get_messagebox_style())
        msg_box.exec_()
    
    def _hex_to_rgb(self, hex_color):
        """将十六进制颜色转换为RGB字符串"""
        hex_color = hex_color.lstrip('#')
        r = int(hex_color[0:2], 16)
        g = int(hex_color[2:4], 16)
        b = int(hex_color[4:6], 16)
        return f"{r}, {g}, {b}"
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        # 停止状态更新线程
        if hasattr(self, 'status_thread'):
            self.status_thread.stop()
            self.status_thread.wait(1000)  # 等待最多1秒
        
        # 清理服务端资源
        if hasattr(self, 'server_manager'):
            self.server_manager.cleanup()
        
        event.accept()

