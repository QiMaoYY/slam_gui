#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS服务管理器
封装所有ROS服务调用
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import rospy
from PyQt5.QtWidgets import QMessageBox

from ..config.settings import config


@dataclass
class MapEntry:
    name: str
    nav_ready: bool
    path: str
    created_at: str
    ori_pointcloud_bytes: int
    nav_pointcloud_bytes: int


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
        from slam_controller.srv import StartMapping, StartMappingRequest
        
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
    
    def stop_mapping(self, save_map=False, map_name=""):
        """
        停止建图
        
        Args:
            save_map: 是否保存地图
            map_name: 地图名称（仅当save_map=True时有效）
        
        Returns:
            bool: 是否成功
        """
        from slam_controller.srv import StopMapping, StopMappingRequest
        
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
            req.save_map = save_map
            req.map_name = map_name
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

    def list_maps(self) -> Optional[List[MapEntry]]:
        """
        获取地图列表（/slam_manager/list_maps）

        Returns:
            List[MapEntry] or None
        """
        from slam_controller.srv import ListMaps, ListMapsRequest

        try:
            rospy.wait_for_service(config.SERVICE_LIST_MAPS, timeout=config.SERVICE_TIMEOUT)
            srv = rospy.ServiceProxy(config.SERVICE_LIST_MAPS, ListMaps)
            resp = srv(ListMapsRequest())

            if not getattr(resp, "success", False):
                self._show_error("失败", getattr(resp, "message", "获取地图列表失败"), QMessageBox.Warning)
                return None

            names = list(getattr(resp, "map_names", []))
            nav_ready = list(getattr(resp, "nav_ready", []))
            paths = list(getattr(resp, "map_paths", []))
            created_at = list(getattr(resp, "created_at", []))
            ori_sizes = list(getattr(resp, "ori_pointcloud_bytes", []))
            nav_sizes = list(getattr(resp, "nav_pointcloud_bytes", []))

            n = len(names)
            entries: List[MapEntry] = []
            for i in range(n):
                entries.append(
                    MapEntry(
                        name=names[i],
                        nav_ready=bool(nav_ready[i]) if i < len(nav_ready) else False,
                        path=str(paths[i]) if i < len(paths) else "",
                        created_at=str(created_at[i]) if i < len(created_at) else "",
                        ori_pointcloud_bytes=int(ori_sizes[i]) if i < len(ori_sizes) else 0,
                        nav_pointcloud_bytes=int(nav_sizes[i]) if i < len(nav_sizes) else 0,
                    )
                )
            return entries

        except rospy.ROSException as e:
            self._show_error("错误", f"无法连接到ListMaps服务:\n{str(e)}", QMessageBox.Critical)
            return None
        except Exception as e:
            self._show_error("错误", f"获取地图列表时发生错误:\n{str(e)}", QMessageBox.Critical)
            return None

    def process_map(self, map_name: str, mode: int) -> bool:
        """
        地图处理（/slam_manager/process_map）

        mode:
          0 全流程
          1 点云清理+2D转换（点云转栅格地图）
          2 编辑2D地图（从init复制一份重新编辑，重置栅格地图）
          3 智能点云过滤（反向过滤点云）
          4 继续编辑2D地图（编辑栅格地图）
        """
        from slam_controller.srv import ProcessMap, ProcessMapRequest

        try:
            rospy.wait_for_service(config.SERVICE_PROCESS_MAP, timeout=config.SERVICE_TIMEOUT)
            srv = rospy.ServiceProxy(config.SERVICE_PROCESS_MAP, ProcessMap)

            req = ProcessMapRequest()
            req.map_name = (map_name or "").strip()
            req.mode = int(mode)
            resp = srv(req)

            if not getattr(resp, "success", False):
                self._show_error("失败", getattr(resp, "message", "地图处理失败"), QMessageBox.Warning)
                return False
            return True

        except rospy.ROSException as e:
            self._show_error("错误", f"无法连接到ProcessMap服务:\n{str(e)}", QMessageBox.Critical)
            return False
        except Exception as e:
            self._show_error("错误", f"调用ProcessMap服务时发生错误:\n{str(e)}", QMessageBox.Critical)
            return False

