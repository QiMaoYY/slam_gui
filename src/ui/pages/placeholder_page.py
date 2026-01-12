#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""空页面占位（导航管理/任务管理）"""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QFrame


class PlaceholderPage(QWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        root = QVBoxLayout()
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)
        self.setLayout(root)

        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        card.setLayout(layout)

        t = QLabel(title)
        t.setObjectName("sectionTitle")
        layout.addWidget(t)

        tip = QLabel("该页面功能待接入（已预留框架与接口）。")
        tip.setObjectName("muted")
        layout.addWidget(tip)

        root.addWidget(card)
        root.addStretch()


