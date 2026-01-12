#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""右侧日志面板：仅展示GUI自身日志（logging/print/stderr）"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPlainTextEdit, QHBoxLayout, QPushButton


class LogPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("logPanel")

        self._title = QLabel("系统日志（GUI）")
        self._title.setObjectName("sectionTitle")

        self._view = QPlainTextEdit()
        self._view.setObjectName("logView")
        self._view.setReadOnly(True)
        self._view.setLineWrapMode(QPlainTextEdit.NoWrap)
        self._view.setMaximumBlockCount(2000)  # 限制行数，避免长期运行内存膨胀

        self._btn_clear = QPushButton("清空")
        self._btn_clear.clicked.connect(self._view.clear)

        header = QHBoxLayout()
        header.setContentsMargins(16, 16, 16, 0)
        header.addWidget(self._title)
        header.addStretch()
        header.addWidget(self._btn_clear)

        root = QVBoxLayout()
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)
        self.setLayout(root)

        root.addLayout(header)
        root.addWidget(self._view, 1)

    def append_line(self, line: str):
        if line is None:
            return
        self._view.appendPlainText(line)
        self._view.moveCursor(QTextCursor.End)
        self._view.ensureCursorVisible()


