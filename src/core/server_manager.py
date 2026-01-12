#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
服务端进程管理器
负责启动和停止SLAM Manager服务端
"""

import os
import signal
import subprocess
import time

import rospy
from PyQt5.QtWidgets import QMessageBox

from ..config.settings import config


class ServerProcessManager:
    """服务端进程管理器"""
    
    def __init__(self, parent=None):
        """
        初始化服务端进程管理器
        
        Args:
            parent: 父窗口，用于显示消息
        """
        self.parent = parent
        self.pid_file = config.SERVER_PID_FILE
        
    def is_running(self):
        """
        检查服务端是否正在运行
        
        Returns:
            bool: 是否运行中
        """
        if os.path.exists(self.pid_file):
            with open(self.pid_file, 'r') as f:
                pid = f.read().strip()
                return os.path.exists(f'/proc/{pid}')
        return False
    
    def start(self, show_terminal: bool = True):
        """
        启动服务端
        
        Returns:
            bool: 是否成功启动
        """
        # 检查是否已经启动
        if self.is_running():
            self._show_message('提示', '服务端已在运行中', QMessageBox.Information)
            return False
        
        # 检查脚本是否存在
        script_path = config.SLAM_MANAGER_SCRIPT
        if not os.path.exists(script_path):
            self._show_message(
                '错误', 
                f'启动脚本不存在:\n{script_path}', 
                QMessageBox.Critical
            )
            return False
        
        # 确保脚本有执行权限
        os.chmod(script_path, 0o755)
        
        # 后台启动（不显示终端）：用交互shell加载~/.bashrc（与桌面终端行为一致）
        if not show_terminal:
            return self._start_background(script_path)

        # 显示终端：尝试启动终端窗口
        for terminal in config.TERMINAL_PRIORITY:
            if self._try_start_terminal(terminal, script_path):
                return True
        
        # 所有终端都失败
        self._show_message(
            '错误', 
            f'无法启动终端窗口\n'
            f'尝试的终端: {", ".join(config.TERMINAL_PRIORITY)}', 
            QMessageBox.Critical
        )
        return False

    def _start_background(self, script_path: str) -> bool:
        """后台启动：不打开终端窗口"""
        try:
            # bash -i 会读取 ~/.bashrc；-c 执行脚本
            cmd = ["bash", "-ic", f"{script_path}"]
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            with open(self.pid_file, "w") as f:
                f.write(str(process.pid))
            rospy.loginfo(f"[ServerManager] 服务端已后台启动 (PID: {process.pid})")
            return True
        except Exception as e:
            self._show_message("错误", f"后台启动服务端失败:\n{str(e)}", QMessageBox.Critical)
            return False
    
    def stop(self):
        """
        停止服务端
        
        Returns:
            bool: 是否成功停止
        """
        try:
            # 1. 终止slam_manager.py进程
            slam_pids = self._find_slam_processes()
            for pid in slam_pids:
                try:
                    os.kill(int(pid), signal.SIGTERM)
                    rospy.loginfo(f"[ServerManager] 已发送终止信号到slam_manager进程 {pid}")
                except ProcessLookupError:
                    pass
                except Exception as e:
                    rospy.logwarn(f"[ServerManager] 无法终止进程 {pid}: {e}")
            
            # 2. 等待进程结束
            if slam_pids:
                self._wait_processes_exit(slam_pids)
            
            # 3. 关闭终端窗口
            self._close_terminal()
            
            rospy.loginfo("[ServerManager] 服务端已停止，窗口已关闭")
            return True
            
        except Exception as e:
            self._show_message(
                '错误', 
                f'停止服务端失败:\n{str(e)}', 
                QMessageBox.Critical
            )
            return False
    
    def cleanup(self):
        """清理资源（程序退出时调用）"""
        if os.path.exists(self.pid_file):
            try:
                with open(self.pid_file, 'r') as f:
                    terminal_pid = f.read().strip()
                
                if os.path.exists(f'/proc/{terminal_pid}'):
                    os.kill(int(terminal_pid), signal.SIGTERM)
                
                os.remove(self.pid_file)
            except:
                pass
    
    def _try_start_terminal(self, terminal, script_path):
        """尝试使用指定终端启动"""
        try:
            if terminal == 'gnome-terminal':
                cmd = [terminal, '--', 'bash', '-c', f'{script_path}']
            else:
                cmd = [terminal, '-e', f'bash -c "{script_path}"']
            
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
            # 保存进程信息
            with open(self.pid_file, 'w') as f:
                f.write(str(process.pid))
            
            rospy.loginfo(
                f"[ServerManager] 服务端已在新终端窗口中启动 "
                f"(PID: {process.pid}, 终端: {terminal})"
            )
            return True
            
        except FileNotFoundError:
            return False
        except Exception as e:
            rospy.logwarn(f"[ServerManager] 使用 {terminal} 启动失败: {e}")
            return False
    
    def _find_slam_processes(self):
        """查找slam_manager.py进程"""
        try:
            result = subprocess.run(
                ['pgrep', '-f', 'slam_manager.py'],
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                return [pid.strip() for pid in result.stdout.strip().split('\n') if pid.strip()]
            return []
        except:
            return []
    
    def _wait_processes_exit(self, pids):
        """等待进程退出"""
        rospy.loginfo("[ServerManager] 等待slam_manager进程结束...")
        timeout_steps = int(config.PROCESS_STOP_TIMEOUT / config.PROCESS_CHECK_INTERVAL)
        
        for i in range(timeout_steps):
            all_dead = True
            for pid in pids:
                if os.path.exists(f'/proc/{pid}'):
                    all_dead = False
                    break
            if all_dead:
                rospy.loginfo("[ServerManager] slam_manager进程已结束")
                return
            time.sleep(config.PROCESS_CHECK_INTERVAL)
    
    def _close_terminal(self):
        """关闭终端窗口"""
        if not os.path.exists(self.pid_file):
            return
        
        try:
            with open(self.pid_file, 'r') as f:
                terminal_pid = f.read().strip()
            
            # 查找并终止所有子进程
            result = subprocess.run(
                ['pgrep', '-P', terminal_pid],
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                child_pids = result.stdout.strip().split('\n')
                for pid in child_pids:
                    if pid:
                        try:
                            os.kill(int(pid), signal.SIGTERM)
                        except:
                            pass
            
            # 终止终端主进程
            if os.path.exists(f'/proc/{terminal_pid}'):
                os.kill(int(terminal_pid), signal.SIGTERM)
                rospy.loginfo(f"[ServerManager] 已关闭终端窗口 (PID: {terminal_pid})")
            
            # 清理PID文件
            os.remove(self.pid_file)
            
        except (ProcessLookupError, ValueError):
            pass
        except Exception as e:
            rospy.logwarn(f"[ServerManager] 关闭终端窗口时出错: {e}")
    
    def _show_message(self, title, message, icon=QMessageBox.Information):
        """显示消息"""
        if self.parent:
            self.parent.show_message(title, message, icon)
        else:
            rospy.loginfo(f"[ServerManager] {title}: {message}")

