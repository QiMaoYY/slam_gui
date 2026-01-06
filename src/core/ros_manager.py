#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS服务管理器
封装所有ROS服务调用
"""

import rospy
from PyQt5.QtWidgets import QMessageBox

from ..config.settings import config


class ROSServiceManager:
    """ROS服务管理器"""
    
    def __init__(self, parent=None):
        """
        初始化ROS服务管理器
        
        Args:
            parent: 父窗口，用于显示错误消息
        """
        self.parent = parent
        
    def start_mapping(self, need_calibration=True):
        """
        开始建图
        
        Args:
            need_calibration: 是否需要雷达校准
            
        Returns:
            bool: 是否成功
        """
        from kuavo_slam.srv import StartMapping, StartMappingRequest
        
        try:
            # 等待服务可用
            rospy.wait_for_service(
                config.SERVICE_START_MAPPING, 
                timeout=config.SERVICE_TIMEOUT
            )
            start_mapping = rospy.ServiceProxy(
                config.SERVICE_START_MAPPING, 
                StartMapping
            )
            
            # 调用服务
            req = StartMappingRequest()
            req.need_calibration = need_calibration
            
            resp = start_mapping(req)
            
            if not resp.success:
                self._show_error('失败', resp.message, QMessageBox.Warning)
                
            return resp.success
                
        except rospy.ROSException as e:
            self._show_error(
                '错误', 
                f'无法连接到SLAM Manager服务:\n{str(e)}', 
                QMessageBox.Critical
            )
            return False
        except Exception as e:
            self._show_error(
                '错误', 
                f'调用服务时发生错误:\n{str(e)}', 
                QMessageBox.Critical
            )
            return False
    
    def stop_mapping(self):
        """
        停止建图
        
        Returns:
            bool: 是否成功
        """
        from kuavo_slam.srv import StopMapping, StopMappingRequest
        
        try:
            # 等待服务可用
            rospy.wait_for_service(
                config.SERVICE_STOP_MAPPING, 
                timeout=config.SERVICE_TIMEOUT
            )
            stop_mapping = rospy.ServiceProxy(
                config.SERVICE_STOP_MAPPING, 
                StopMapping
            )
            
            # 调用服务
            req = StopMappingRequest()
            resp = stop_mapping(req)
            
            if not resp.success:
                self._show_error('失败', resp.message, QMessageBox.Warning)
                
            return resp.success
                
        except rospy.ROSException as e:
            self._show_error(
                '错误', 
                f'无法连接到SLAM Manager服务:\n{str(e)}', 
                QMessageBox.Critical
            )
            return False
        except Exception as e:
            self._show_error(
                '错误', 
                f'调用服务时发生错误:\n{str(e)}', 
                QMessageBox.Critical
            )
            return False
    
    def _show_error(self, title, message, icon=QMessageBox.Information):
        """显示错误消息"""
        if self.parent:
            self.parent.show_message(title, message, icon)
        else:
            rospy.logerr(f"[ROSServiceManager] {title}: {message}")

