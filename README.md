# Kuavo SLAM 图形化控制界面

现代化的SLAM系统图形控制界面，基于PyQt5实现，提供直观的建图、定位和导航控制功能。

## 特性

- 🎨 **现代暗色主题** - 基于 Catppuccin Mocha 配色，长时间使用不疲劳
- 📊 **实时状态监控** - 2Hz自动刷新系统状态、运行时间等信息
- 🎮 **简洁操作** - 清晰的按钮布局，一键式操作
- 🔔 **智能提示** - 状态变化和操作结果实时反馈
- 🛡️ **安全确认** - 关键操作需要二次确认，防止误操作
- 🏗️ **工程化架构** - 模块化设计，易于扩展和维护

## 项目结构

```
slam_gui/
├── src/                      # 源代码目录
│   ├── ui/                   # UI相关模块
│   │   ├── main_window.py   # 主窗口
│   │   └── styles.py        # 样式定义
│   ├── core/                 # 核心功能模块
│   │   ├── ros_manager.py   # ROS服务管理
│   │   ├── server_manager.py # 服务端进程管理
│   │   └── status_thread.py # 状态更新线程
│   ├── config/               # 配置模块
│   │   └── settings.py      # 全局配置
│   └── main.py              # 主入口
├── slam_gui_main.py         # 启动脚本
├── run_gui.sh               # Shell启动脚本
├── requirements.txt         # Python依赖
└── README.md               # 本文档
```

## 系统要求

### 软件依赖
- Python 3.8+
- ROS Noetic/Melodic
- PyQt5
- 已编译的 kuavo_slam 功能包

### 硬件要求
- 支持 x86_64 和 ARM64 架构

## 快速开始

### 1. 安装依赖

对于 ARM 架构（如 Jetson）：
```bash
# PyQt5 使用系统包（已自动配置软链接到conda环境）
sudo apt install python3-pyqt5 python3-pyqt5.qtsvg
```

对于 x86_64 架构：
```bash
pip install -r requirements.txt
```

### 2. 启动GUI

```bash
cd /media/data/slam_ws/slam_gui
./run_gui.sh
```

或直接运行：
```bash
python3 slam_gui_main.py
```

## 功能说明

### 状态显示区域

**当前状态指示器**
- 🟢 绿色"空闲" - 系统就绪，可以开始新任务
- 🔵 蓝色"建图中" - 正在进行建图任务
- 🟠 橙色"定位中" - 正在进行定位
- 🟣 紫色"导航中" - 正在执行导航任务
- 🔴 红色"错误" - 系统出现错误
- ⚫ 灰色"未连接" - 无法连接到SLAM Manager

### 建图控制

1. **开始建图**
   - 勾选"启用雷达校准"（首次建图推荐）
   - 点击"🚀 开始建图"按钮
   - 观察状态变为"建图中"

2. **停止建图**
   - 点击"⏹ 停止建图"按钮
   - 确认对话框选择"是"
   - 系统自动保存地图并清理资源

### 服务端控制

1. **启动服务端**
   - 点击"▶ 启动服务端"按钮
   - 自动在新终端窗口启动 SLAM Manager
   - 等待状态变为"空闲"

2. **停止服务端**
   - 点击"⏹ 停止服务端"按钮
   - 确认对话框选择"是"
   - 自动终止进程并关闭终端窗口

## ROS 服务接口

GUI 通过以下 ROS Service 与 SLAM Manager 通信：

- `/slam_manager/start_mapping` - 启动建图
- `/slam_manager/stop_mapping` - 停止建图
- `/slam_manager/get_status` - 获取状态（2Hz自动查询）

## 配置

所有配置项集中在 `src/config/settings.py` 中，包括：

- 路径配置
- ROS服务配置
- UI配置
- 状态配置
- 终端配置
- 进程管理配置

## 开发

### 添加新功能

1. 在 `src/core/` 中添加新的功能模块
2. 在 `src/ui/` 中添加对应的UI组件
3. 在 `src/config/settings.py` 中添加相关配置
4. 在 `src/ui/main_window.py` 中集成新功能

### 自定义主题

修改 `src/ui/styles.py` 中的 `DarkTheme` 类即可自定义界面样式。

## 故障排除

### GUI 无法启动
- 检查 PyQt5 是否正确安装
- 检查 ROS 环境是否配置
- 查看终端错误信息

### 显示"未连接"状态
1. 检查 roscore 是否运行：`rosnode list`
2. 检查 slam_manager 是否运行：`rosservice list | grep slam_manager`
3. 点击"启动服务端"按钮启动 SLAM Manager

### 按钮无反应
- 查看终端输出的错误信息
- 检查 ROS Master 连接状态
- 确认 slam_manager 服务可用

## 版本历史

- v1.0.0 (2026-01-06) - 工程化重构版本
  - 模块化架构
  - 分离UI、业务逻辑和配置
  - 改进代码可维护性和可扩展性

- v0.1.0 (2026-01-05) - 初始版本
  - 基础建图控制功能
  - 服务端管理功能

## 作者

Kuavo Team

## 许可证

本项目遵循与 Kuavo SLAM 系统相同的许可证。
