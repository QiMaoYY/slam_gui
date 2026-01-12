#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UI样式定义 - 简约大气沉稳深色主题
集中管理所有QSS样式
"""


class DarkTheme:
    """暗色主题（偏沉稳、低饱和）"""

    @staticmethod
    def get_stylesheet() -> str:
        return """
        /* ==================== 全局 ==================== */
        QMainWindow, QWidget {
            background-color: #0f1115;
            color: #d6dbe3;
            font-family: 'Segoe UI', 'Microsoft YaHei', 'PingFang SC', sans-serif;
            font-size: 13px;
        }

        /* ==================== 通用容器 ==================== */
        #appRoot {
            background-color: #0f1115;
        }

        #card {
            background-color: #141822;
            border: 1px solid rgba(120, 140, 170, 0.18);
            border-radius: 12px;
        }

        QLabel#sectionTitle {
            font-size: 14px;
            font-weight: 600;
            color: #e6eaf2;
        }

        QLabel#muted {
            color: rgba(214, 219, 227, 0.65);
        }

        /* ==================== 左侧菜单 ==================== */
        #sidebar {
            background-color: #10141d;
            border-right: 1px solid rgba(120, 140, 170, 0.18);
        }

        QLabel#appName {
            font-size: 16px;
            font-weight: 700;
            letter-spacing: 1px;
            color: #e6eaf2;
            padding: 10px 12px;
        }

        QLabel#appSubtitle {
            font-size: 12px;
            color: rgba(214, 219, 227, 0.6);
            padding: 0px 12px 10px 12px;
        }

        QToolButton#navBtn {
            text-align: left;
            padding: 12px 12px;
            border: none;
            border-radius: 10px;
            background: transparent;
            color: rgba(214, 219, 227, 0.85);
            font-size: 14px;
        }

        QToolButton#navBtn:hover {
            background: rgba(120, 140, 170, 0.10);
        }

        QToolButton#navBtn:checked {
            background: rgba(90, 120, 180, 0.18);
            color: #e6eaf2;
        }

        /* ==================== 右侧日志 ==================== */
        #logPanel {
            background-color: #10141d;
            border-left: 1px solid rgba(120, 140, 170, 0.18);
        }

        QPlainTextEdit#logView {
            background-color: #0b0e14;
            color: #cfd6e1;
            border: 1px solid rgba(120, 140, 170, 0.18);
            border-radius: 10px;
            padding: 10px;
            font-family: 'Consolas', 'Monaco', 'DejaVu Sans Mono', monospace;
            font-size: 12px;
        }

        /* 滚动条（简约） */
        QScrollBar:vertical {
            background: transparent;
            width: 10px;
            margin: 0px;
        }
        QScrollBar::handle:vertical {
            background: rgba(120, 140, 170, 0.25);
            border-radius: 5px;
            min-height: 20px;
        }
        QScrollBar::handle:vertical:hover {
            background: rgba(120, 140, 170, 0.35);
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0px;
        }
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
            background: transparent;
        }

        /* ==================== 输入控件 ==================== */
        QLineEdit, QComboBox {
            background-color: #0b0e14;
            color: #d6dbe3;
            border: 1px solid rgba(120, 140, 170, 0.22);
            border-radius: 10px;
            padding: 8px 10px;
        }
        QLineEdit:focus, QComboBox:focus {
            border: 1px solid rgba(90, 120, 180, 0.55);
        }

        QComboBox::drop-down {
            border: none;
            width: 28px;
        }

        QComboBox QAbstractItemView {
            background-color: #0b0e14;
            border: 1px solid rgba(120, 140, 170, 0.22);
            selection-background-color: rgba(90, 120, 180, 0.25);
            outline: 0;
        }

        /* ==================== 按钮 ==================== */
        QPushButton {
            background-color: rgba(120, 140, 170, 0.12);
            border: 1px solid rgba(120, 140, 170, 0.22);
            color: #e6eaf2;
            border-radius: 10px;
            padding: 10px 12px;
            font-size: 13px;
            font-weight: 600;
        }
        QPushButton:hover {
            background-color: rgba(120, 140, 170, 0.16);
        }
        QPushButton:pressed {
            background-color: rgba(120, 140, 170, 0.10);
        }
        QPushButton:disabled {
            color: rgba(214, 219, 227, 0.35);
            border-color: rgba(120, 140, 170, 0.12);
            background-color: rgba(120, 140, 170, 0.06);
        }

        /* 关键动作按钮：保持原objectName以复用颜色语义 */
        QPushButton#startMappingBtn {
            background-color: rgba(60, 130, 200, 0.22);
            border-color: rgba(60, 130, 200, 0.40);
        }
        QPushButton#saveMapBtn {
            background-color: rgba(60, 170, 120, 0.22);
            border-color: rgba(60, 170, 120, 0.40);
        }
        QPushButton#abortMappingBtn {
            background-color: rgba(200, 90, 90, 0.18);
            border-color: rgba(200, 90, 90, 0.35);
        }
        QPushButton#startServerBtn {
            background-color: rgba(90, 120, 180, 0.18);
            border-color: rgba(90, 120, 180, 0.35);
        }
        QPushButton#stopServerBtn {
            background-color: rgba(200, 150, 60, 0.18);
            border-color: rgba(200, 150, 60, 0.35);
        }

        /* ==================== 复选框 ==================== */
        QCheckBox {
            color: rgba(214, 219, 227, 0.85);
            spacing: 8px;
        }
        QCheckBox::indicator {
            width: 18px;
            height: 18px;
            border-radius: 6px;
            border: 1px solid rgba(120, 140, 170, 0.35);
            background: #0b0e14;
        }
        QCheckBox::indicator:checked {
            background: rgba(90, 120, 180, 0.55);
            border-color: rgba(90, 120, 180, 0.55);
        }

        /* ==================== 底部状态条 ==================== */
        #statusBar {
            background-color: #0b0e14;
            border-top: 1px solid rgba(120, 140, 170, 0.18);
        }
        QLabel#statusItem {
            color: rgba(214, 219, 227, 0.80);
            padding: 0px 10px;
        }
        QLabel#statusItemStrong {
            color: #e6eaf2;
            font-weight: 700;
            padding: 0px 10px;
        }
        """

    @staticmethod
    def get_messagebox_style() -> str:
        return """
        QMessageBox {
            background-color: #141822;
            color: #d6dbe3;
        }
        QMessageBox QLabel {
            color: #d6dbe3;
        }
        """

    @staticmethod
    def get_inputdialog_style() -> str:
        return """
        QInputDialog {
            background-color: #141822;
            color: #d6dbe3;
        }
        QInputDialog QLabel {
            color: rgba(214, 219, 227, 0.85);
        }
        QLineEdit {
            background-color: #0b0e14;
            border: 1px solid rgba(120, 140, 170, 0.22);
            border-radius: 10px;
            padding: 8px 10px;
            color: #d6dbe3;
        }
        """


