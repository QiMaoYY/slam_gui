#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UI样式定义
集中管理所有QSS样式
"""


class DarkTheme:
    """暗色主题样式"""
    
    @staticmethod
    def get_stylesheet():
        """获取完整的样式表"""
        return """
        /* 全局样式 */
        QMainWindow {
            background-color: #1e1e2e;
        }
        
        QWidget {
            background-color: #1e1e2e;
            color: #cdd6f4;
            font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;
            font-size: 14px;
        }
        
        /* 标题样式 */
        #titleLabel {
            font-size: 28px;
            font-weight: bold;
            color: #89b4fa;
            padding: 20px;
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 #313244,
                stop:1 #45475a
            );
            border-radius: 10px;
            margin-bottom: 10px;
        }
        
        /* 组框样式 */
        QGroupBox {
            border: 2px solid #45475a;
            border-radius: 10px;
            margin-top: 10px;
            padding-top: 25px;
            font-size: 16px;
            font-weight: bold;
            color: #89b4fa;
        }
        
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 20px;
            padding: 0 10px;
        }
        
        #statusGroup, #mappingGroup, #serverGroup {
            background-color: #181825;
            padding: 20px;
        }
        
        /* 状态指示器 */
        #statusIndicator {
            background-color: #45475a;
            color: #cdd6f4;
            border-radius: 8px;
            padding: 8px 16px;
            font-weight: bold;
            font-size: 15px;
        }
        
        #statusLabelText, #uptimeLabel {
            color: #bac2de;
            font-size: 15px;
        }
        
        #messageLabel {
            color: #a6adc8;
            font-size: 13px;
            padding: 10px;
            background-color: #11111b;
            border-radius: 8px;
            border-left: 3px solid #89b4fa;
        }
        
        /* 分隔线 */
        #separator {
            background-color: #45475a;
            border: none;
            height: 2px;
        }
        
        /* 复选框 */
        QCheckBox {
            color: #cdd6f4;
            font-size: 14px;
            spacing: 8px;
        }
        
        QCheckBox::indicator {
            width: 20px;
            height: 20px;
            border-radius: 4px;
            border: 2px solid #45475a;
            background-color: #313244;
        }
        
        QCheckBox::indicator:checked {
            background-color: #89b4fa;
            border-color: #89b4fa;
        }
        
        QCheckBox::indicator:checked:hover {
            background-color: #74c7ec;
        }
        
        /* 按钮通用样式 */
        QPushButton {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #45475a,
                stop:1 #313244
            );
            color: #cdd6f4;
            border: 2px solid #45475a;
            border-radius: 10px;
            padding: 15px;
            font-size: 16px;
            font-weight: bold;
        }
        
        QPushButton:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #585b70,
                stop:1 #45475a
            );
            border-color: #585b70;
        }
        
        QPushButton:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #313244,
                stop:1 #1e1e2e
            );
        }
        
        QPushButton:disabled {
            background-color: #313244;
            color: #6c7086;
            border-color: #313244;
        }
        
        /* 开始建图按钮 */
        #startMappingBtn {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #89b4fa,
                stop:1 #74c7ec
            );
            color: #1e1e2e;
            border: none;
        }
        
        #startMappingBtn:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #74c7ec,
                stop:1 #89dceb
            );
        }
        
        #startMappingBtn:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #74c7ec,
                stop:1 #89b4fa
            );
        }
        
        #startMappingBtn:disabled {
            background-color: #313244;
            color: #6c7086;
        }
        
        /* 停止建图按钮 */
        #stopMappingBtn {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #f38ba8,
                stop:1 #eba0ac
            );
            color: #1e1e2e;
            border: none;
        }
        
        #stopMappingBtn:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #eba0ac,
                stop:1 #f5c2e7
            );
        }
        
        #stopMappingBtn:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #f5c2e7,
                stop:1 #f38ba8
            );
        }
        
        #stopMappingBtn:disabled {
            background-color: #313244;
            color: #6c7086;
        }
        
        /* 启动服务端按钮 */
        #startServerBtn {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #a6e3a1,
                stop:1 #94e2d5
            );
            color: #1e1e2e;
            border: none;
        }
        
        #startServerBtn:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #94e2d5,
                stop:1 #89dceb
            );
        }
        
        #startServerBtn:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #89dceb,
                stop:1 #a6e3a1
            );
        }
        
        #startServerBtn:disabled {
            background-color: #313244;
            color: #6c7086;
        }
        
        /* 停止服务端按钮 */
        #stopServerBtn {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #f9e2af,
                stop:1 #fab387
            );
            color: #1e1e2e;
            border: none;
        }
        
        #stopServerBtn:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #fab387,
                stop:1 #eba0ac
            );
        }
        
        #stopServerBtn:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #eba0ac,
                stop:1 #f9e2af
            );
        }
        
        #stopServerBtn:disabled {
            background-color: #313244;
            color: #6c7086;
        }
        """
    
    @staticmethod
    def get_messagebox_style():
        """获取消息框样式"""
        return """
        QMessageBox {
            background-color: #1e1e2e;
            color: #cdd6f4;
        }
        QMessageBox QLabel {
            color: #cdd6f4;
            font-size: 14px;
        }
        QPushButton {
            background-color: #45475a;
            color: #cdd6f4;
            border: 2px solid #585b70;
            border-radius: 5px;
            padding: 8px 20px;
            font-size: 13px;
        }
        QPushButton:hover {
            background-color: #585b70;
        }
        """

