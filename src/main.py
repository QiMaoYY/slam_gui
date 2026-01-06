#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kuavo SLAM GUI 主入口
启动SLAM系统图形化控制界面
"""

import sys
from PyQt5.QtWidgets import QApplication

from .ui.main_window import SlamMainWindow


def main():
    """主函数入口"""
    app = QApplication(sys.argv)
    
    # 设置应用程序信息
    app.setApplicationName('Kuavo SLAM GUI')
    app.setOrganizationName('Kuavo Team')
    
    # 创建并显示主窗口
    window = SlamMainWindow()
    window.show()
    
    # 运行应用程序
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

