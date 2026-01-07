#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UI样式定义 - 现代科技风格
集中管理所有QSS样式
"""


class DarkTheme:
    """暗色主题样式 - 专业科技风格"""
    
    @staticmethod
    def get_stylesheet():
        """获取完整的样式表"""
        return """
        /* ==================== 全局样式 ==================== */
        QMainWindow {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #0a0e27,
                stop:1 #1a1d35
            );
        }
        
        QWidget {
            background-color: transparent;
            color: #e0e6f0;
            font-family: 'Roboto', 'Segoe UI', 'Microsoft YaHei', 'PingFang SC', sans-serif;
            font-size: 13px;
        }
        
        /* ==================== 标题样式 ==================== */
        #titleLabel {
            font-size: 32px;
            font-weight: 300;
            letter-spacing: 2px;
            color: #00d4ff;
            padding: 25px 20px;
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(0, 212, 255, 0.15),
                stop:0.5 rgba(0, 150, 255, 0.1),
                stop:1 rgba(138, 43, 226, 0.15)
            );
            border: 1px solid rgba(0, 212, 255, 0.3);
            border-radius: 12px;
            margin-bottom: 10px;
        }
        
        /* ==================== 组框样式 ==================== */
        QGroupBox {
            border: 1px solid rgba(0, 212, 255, 0.25);
            border-radius: 12px;
            margin-top: 15px;
            padding-top: 30px;
            font-size: 15px;
            font-weight: 500;
            letter-spacing: 1px;
            color: #00d4ff;
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 rgba(26, 29, 53, 0.9),
                stop:1 rgba(15, 18, 40, 0.9)
            );
        }
        
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 25px;
            padding: 0 15px;
            background-color: #0f1228;
            border: 1px solid rgba(0, 212, 255, 0.3);
            border-radius: 6px;
        }
        
        #statusGroup, #mappingGroup, #serverGroup {
            padding: 25px;
            box-shadow: 0 4px 15px rgba(0, 0, 0, 0.4);
        }
        
        /* ==================== 状态指示器 ==================== */
        #statusIndicator {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(0, 150, 255, 0.3),
                stop:1 rgba(0, 212, 255, 0.3)
            );
            color: #ffffff;
            border: 1px solid rgba(0, 212, 255, 0.5);
            border-radius: 10px;
            padding: 10px 20px;
            font-weight: 600;
            font-size: 14px;
            letter-spacing: 1px;
        }
        
        #statusLabelText, #uptimeLabel {
            color: #b8c5d9;
            font-size: 14px;
            font-weight: 500;
        }
        
        #messageLabel {
            color: #c0cdd9;
            font-size: 13px;
            padding: 15px;
            background: rgba(15, 18, 40, 0.6);
            border-radius: 10px;
            border-left: 3px solid #00d4ff;
            border-right: 1px solid rgba(0, 212, 255, 0.2);
            border-top: 1px solid rgba(0, 212, 255, 0.1);
            border-bottom: 1px solid rgba(0, 212, 255, 0.1);
        }
        
        /* ==================== 分隔线 ==================== */
        #separator {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(0, 212, 255, 0.1),
                stop:0.5 rgba(0, 212, 255, 0.3),
                stop:1 rgba(0, 212, 255, 0.1)
            );
            border: none;
            height: 1px;
            margin: 10px 0;
        }
        
        /* ==================== 复选框 ==================== */
        QCheckBox {
            color: #d0dae6;
            font-size: 13px;
            spacing: 10px;
            font-weight: 400;
        }
        
        QCheckBox::indicator {
            width: 22px;
            height: 22px;
            border-radius: 6px;
            border: 2px solid rgba(0, 212, 255, 0.4);
            background: rgba(15, 18, 40, 0.6);
        }
        
        QCheckBox::indicator:checked {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:1,
                stop:0 #00d4ff,
                stop:1 #0096ff
            );
            border-color: #00d4ff;
        }
        
        QCheckBox::indicator:checked:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:1,
                stop:0 #00e5ff,
                stop:1 #00b0ff
            );
        }
        
        QCheckBox::indicator:hover {
            border-color: #00d4ff;
        }
        
        /* ==================== 按钮通用样式 ==================== */
        QPushButton {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 rgba(30, 35, 60, 0.9),
                stop:1 rgba(20, 25, 50, 0.9)
            );
            color: #e0e6f0;
            border: 1px solid rgba(0, 212, 255, 0.3);
            border-radius: 10px;
            padding: 15px;
            font-size: 15px;
            font-weight: 500;
            letter-spacing: 0.5px;
        }
        
        QPushButton:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 rgba(40, 50, 80, 0.95),
                stop:1 rgba(30, 40, 70, 0.95)
            );
            border-color: rgba(0, 212, 255, 0.5);
            color: #ffffff;
        }
        
        QPushButton:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 rgba(15, 18, 40, 0.95),
                stop:1 rgba(10, 15, 35, 0.95)
            );
            border-color: rgba(0, 212, 255, 0.6);
        }
        
        QPushButton:disabled {
            background: rgba(20, 25, 50, 0.5);
            color: rgba(224, 230, 240, 0.3);
            border-color: rgba(0, 212, 255, 0.1);
        }
        
        /* ==================== 开始建图按钮 ==================== */
        #startMappingBtn {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(0, 150, 255, 0.8),
                stop:1 rgba(0, 212, 255, 0.8)
            );
            color: #ffffff;
            border: 1px solid rgba(0, 212, 255, 0.9);
            font-weight: 600;
            letter-spacing: 1px;
        }
        
        #startMappingBtn:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(0, 170, 255, 0.9),
                stop:1 rgba(0, 230, 255, 0.9)
            );
            border-color: #00e5ff;
            box-shadow: 0 0 20px rgba(0, 212, 255, 0.5);
        }
        
        #startMappingBtn:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(0, 120, 200, 0.9),
                stop:1 rgba(0, 180, 230, 0.9)
            );
        }
        
        #startMappingBtn:disabled {
            background: rgba(20, 40, 80, 0.3);
            color: rgba(224, 230, 240, 0.3);
            border-color: rgba(0, 212, 255, 0.1);
        }
        
        /* ==================== 保存地图按钮 ==================== */
        #saveMapBtn {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(0, 200, 100, 0.8),
                stop:1 rgba(0, 230, 150, 0.8)
            );
            color: #ffffff;
            border: 1px solid rgba(0, 230, 150, 0.9);
            font-weight: 600;
            letter-spacing: 1px;
        }
        
        #saveMapBtn:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(0, 220, 120, 0.9),
                stop:1 rgba(0, 250, 170, 0.9)
            );
            border-color: #00fa96;
            box-shadow: 0 0 20px rgba(0, 230, 150, 0.5);
        }
        
        #saveMapBtn:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(0, 170, 80, 0.9),
                stop:1 rgba(0, 200, 120, 0.9)
            );
        }
        
        #saveMapBtn:disabled {
            background: rgba(20, 40, 30, 0.3);
            color: rgba(224, 230, 240, 0.3);
            border-color: rgba(0, 230, 150, 0.1);
        }
        
        /* ==================== 终止建图按钮 ==================== */
        #abortMappingBtn {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(255, 80, 80, 0.7),
                stop:1 rgba(255, 120, 100, 0.7)
            );
            color: #ffffff;
            border: 1px solid rgba(255, 100, 90, 0.8);
            font-weight: 600;
            letter-spacing: 1px;
        }
        
        #abortMappingBtn:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(255, 100, 100, 0.85),
                stop:1 rgba(255, 140, 120, 0.85)
            );
            border-color: #ff8078;
            box-shadow: 0 0 20px rgba(255, 100, 90, 0.4);
        }
        
        #abortMappingBtn:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(200, 60, 60, 0.9),
                stop:1 rgba(220, 90, 70, 0.9)
            );
        }
        
        #abortMappingBtn:disabled {
            background: rgba(40, 20, 20, 0.3);
            color: rgba(224, 230, 240, 0.3);
            border-color: rgba(255, 100, 90, 0.1);
        }
        
        /* ==================== 启动服务端按钮 ==================== */
        #startServerBtn {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(138, 43, 226, 0.7),
                stop:1 rgba(100, 149, 237, 0.7)
            );
            color: #ffffff;
            border: 1px solid rgba(138, 43, 226, 0.8);
            font-weight: 600;
            letter-spacing: 1px;
        }
        
        #startServerBtn:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(158, 63, 246, 0.85),
                stop:1 rgba(120, 169, 257, 0.85)
            );
            border-color: #a855f7;
            box-shadow: 0 0 20px rgba(138, 43, 226, 0.4);
        }
        
        #startServerBtn:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(118, 33, 206, 0.9),
                stop:1 rgba(80, 129, 217, 0.9)
            );
        }
        
        #startServerBtn:disabled {
            background: rgba(30, 20, 40, 0.3);
            color: rgba(224, 230, 240, 0.3);
            border-color: rgba(138, 43, 226, 0.1);
        }
        
        /* ==================== 停止服务端按钮 ==================== */
        #stopServerBtn {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(255, 180, 0, 0.7),
                stop:1 rgba(255, 140, 0, 0.7)
            );
            color: #ffffff;
            border: 1px solid rgba(255, 160, 0, 0.8);
            font-weight: 600;
            letter-spacing: 1px;
        }
        
        #stopServerBtn:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(255, 200, 20, 0.85),
                stop:1 rgba(255, 160, 20, 0.85)
            );
            border-color: #ffa020;
            box-shadow: 0 0 20px rgba(255, 160, 0, 0.4);
        }
        
        #stopServerBtn:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:1, y2:0,
                stop:0 rgba(230, 160, 0, 0.9),
                stop:1 rgba(230, 120, 0, 0.9)
            );
        }
        
        #stopServerBtn:disabled {
            background: rgba(40, 30, 20, 0.3);
            color: rgba(224, 230, 240, 0.3);
            border-color: rgba(255, 160, 0, 0.1);
        }
        """
    
    @staticmethod
    def get_messagebox_style():
        """获取消息框样式"""
        return """
        QMessageBox {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #1a1d35,
                stop:1 #0f1228
            );
            color: #e0e6f0;
            border: 1px solid rgba(0, 212, 255, 0.3);
            border-radius: 10px;
        }
        QMessageBox QLabel {
            color: #d0dae6;
            font-size: 13px;
            padding: 10px;
        }
        QPushButton {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 rgba(0, 150, 255, 0.6),
                stop:1 rgba(0, 120, 200, 0.6)
            );
            color: #ffffff;
            border: 1px solid rgba(0, 212, 255, 0.5);
            border-radius: 6px;
            padding: 10px 25px;
            font-size: 13px;
            font-weight: 500;
        }
        QPushButton:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 rgba(0, 170, 255, 0.8),
                stop:1 rgba(0, 140, 220, 0.8)
            );
            border-color: #00d4ff;
        }
        QPushButton:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 rgba(0, 120, 200, 0.9),
                stop:1 rgba(0, 100, 180, 0.9)
            );
        }
        """
    
    @staticmethod
    def get_inputdialog_style():
        """获取输入对话框样式"""
        return """
        QInputDialog {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 #1a1d35,
                stop:1 #0f1228
            );
            color: #e0e6f0;
            border: 1px solid rgba(0, 212, 255, 0.3);
            border-radius: 10px;
        }
        QInputDialog QLabel {
            color: #d0dae6;
            font-size: 13px;
            padding: 10px;
        }
        QLineEdit {
            background: rgba(15, 18, 40, 0.8);
            color: #e0e6f0;
            border: 2px solid rgba(0, 212, 255, 0.4);
            border-radius: 8px;
            padding: 12px;
            font-size: 14px;
            font-weight: 500;
        }
        QLineEdit:focus {
            border-color: #00d4ff;
            background: rgba(20, 25, 50, 0.9);
            box-shadow: 0 0 10px rgba(0, 212, 255, 0.3);
        }
        QLineEdit:hover {
            border-color: rgba(0, 212, 255, 0.6);
        }
        QPushButton {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 rgba(0, 150, 255, 0.6),
                stop:1 rgba(0, 120, 200, 0.6)
            );
            color: #ffffff;
            border: 1px solid rgba(0, 212, 255, 0.5);
            border-radius: 6px;
            padding: 10px 25px;
            font-size: 13px;
            font-weight: 500;
        }
        QPushButton:hover {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 rgba(0, 170, 255, 0.8),
                stop:1 rgba(0, 140, 220, 0.8)
            );
            border-color: #00d4ff;
        }
        QPushButton:pressed {
            background: qlineargradient(
                x1:0, y1:0, x2:0, y2:1,
                stop:0 rgba(0, 120, 200, 0.9),
                stop:1 rgba(0, 100, 180, 0.9)
            );
        }
        """

