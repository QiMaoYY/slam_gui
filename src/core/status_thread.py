#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态更新线程
负责定期查询SLAM系统状态并发送信号更新UI
"""

import rospy
from PyQt5.QtCore import QThread, pyqtSignal

from ..config.settings import config


class StatusUpdateThread(QThread):
    """状态更新线程 - 定期查询SLAM系统状态"""
    
    status_updated = pyqtSignal(str, str, int)  # status, message, uptime
    
    def __init__(self):
        super().__init__()
        self.running = True
        
    def run(self):
        """线程主循环"""
        from slam_controller.srv import GetSlamStatus, GetSlamStatusRequest
        
        rate = rospy.Rate(config.STATUS_UPDATE_RATE)
        
        while self.running and not rospy.is_shutdown():
            try:
                # 等待服务可用
                rospy.wait_for_service(
                    config.SERVICE_GET_STATUS, 
                    timeout=0.5
                )
                get_status = rospy.ServiceProxy(
                    config.SERVICE_GET_STATUS, 
                    GetSlamStatus
                )
                
                # 调用服务
                resp = get_status(GetSlamStatusRequest())
                self.status_updated.emit(resp.status, resp.message, resp.uptime_sec)
                
            except rospy.ROSException:
                self.status_updated.emit('disconnected', '无法连接到SLAM Manager', 0)
            except Exception as e:
                self.status_updated.emit('error', f'状态查询错误: {str(e)}', 0)
            
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
    
    def stop(self):
        """停止线程"""
        self.running = False

