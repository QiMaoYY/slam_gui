#!/bin/bash
################################################################################
# Kuavo SLAM GUI 启动脚本
# 功能：启动图形化SLAM控制界面
#
# 使用方法：
#   ./run_gui.sh
#
# 作者：Kuavo Team
# 日期：2026-01-05
################################################################################

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GUI_SCRIPT="${SCRIPT_DIR}/slam_gui_main.py"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 打印带颜色的信息
print_info() {
    echo -e "${CYAN}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}   Kuavo SLAM GUI 启动器${NC}"
    echo -e "${BLUE}================================${NC}"
}

# 主函数
main() {
    print_header
    echo ""
    
    # 检查conda环境
    print_info "检查Python环境..."
    if [[ -z "${CONDA_DEFAULT_ENV}" ]]; then
        print_warning "未检测到conda环境"
        print_info "尝试激活 demo 环境..."
        
        # 尝试初始化conda
        if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
            source "${HOME}/miniconda3/etc/profile.d/conda.sh"
        elif [[ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]]; then
            source "${HOME}/anaconda3/etc/profile.d/conda.sh"
        elif [[ -f "/opt/conda/etc/profile.d/conda.sh" ]]; then
            source "/opt/conda/etc/profile.d/conda.sh"
        else
            print_error "无法找到conda初始化脚本"
            print_info "请手动激活环境: conda activate demo"
            exit 1
        fi
        
        # 激活demo环境
        conda activate demo
        if [[ $? -ne 0 ]]; then
            print_error "无法激活conda环境: demo"
            print_info "请手动运行: conda activate demo"
            exit 1
        fi
    fi
    
    print_success "Python环境: ${CONDA_DEFAULT_ENV}"
    
    # 检查Python版本
    PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
    print_info "Python版本: ${PYTHON_VERSION}"
    
    # 检查ROS环境
    print_info "检查ROS环境..."
    if [[ -z "${ROS_MASTER_URI}" ]]; then
        print_warning "未检测到ROS环境变量"
        print_info "尝试加载ROS环境..."
        
        # 尝试source ROS setup
        if [[ -f "/opt/ros/noetic/setup.bash" ]]; then
            source /opt/ros/noetic/setup.bash
        elif [[ -f "/opt/ros/melodic/setup.bash" ]]; then
            source /opt/ros/melodic/setup.bash
        else
            print_error "无法找到ROS安装"
            exit 1
        fi
        
        # 加载工作空间环境
        WORKSPACE_SETUP="/media/data/slam_ws/devel/setup.bash"
        if [[ -f "${WORKSPACE_SETUP}" ]]; then
            source "${WORKSPACE_SETUP}"
            print_success "已加载工作空间环境"
        else
            print_warning "工作空间未编译: ${WORKSPACE_SETUP}"
            print_info "请先编译工作空间: cd /media/data/slam_ws && catkin_make"
        fi
    fi
    
    print_success "ROS Master: ${ROS_MASTER_URI}"
    
    # 检查PyQt5是否安装
    print_info "检查PyQt5依赖..."
    python3 -c "import PyQt5" 2>/dev/null
    if [[ $? -ne 0 ]]; then
        print_warning "PyQt5未安装"
        print_info "正在安装PyQt5..."
        pip install PyQt5 -i https://pypi.tuna.tsinghua.edu.cn/simple
        
        if [[ $? -ne 0 ]]; then
            print_error "PyQt5安装失败"
            print_info "请手动安装: pip install PyQt5"
            exit 1
        fi
        print_success "PyQt5安装完成"
    else
        print_success "PyQt5已安装"
    fi
    
    # 检查GUI脚本是否存在
    if [[ ! -f "${GUI_SCRIPT}" ]]; then
        print_error "GUI脚本不存在: ${GUI_SCRIPT}"
        exit 1
    fi
    
    # 添加执行权限
    chmod +x "${GUI_SCRIPT}"
    
    # 检查roscore是否运行
    print_info "检查ROS Master状态..."
    rostopic list &>/dev/null
    if [[ $? -ne 0 ]]; then
        print_warning "无法连接到ROS Master"
        print_info "请确保roscore已启动: roscore"
        print_info "GUI将继续启动，但功能可能受限"
        echo ""
    else
        print_success "ROS Master连接正常"
    fi
    
    # 启动GUI
    echo ""
    print_info "启动SLAM GUI..."
    print_info "主脚本: slam_gui_main.py"
    echo ""
    print_success "GUI启动中，请稍候..."
    echo ""
    
    # 运行GUI程序（使用新的模块化入口）
    cd "${SCRIPT_DIR}"
    python3 slam_gui_main.py
    
    # 检查退出状态
    EXIT_CODE=$?
    echo ""
    if [[ ${EXIT_CODE} -eq 0 ]]; then
        print_success "GUI已正常退出"
    else
        print_error "GUI异常退出，退出码: ${EXIT_CODE}"
    fi
    
    exit ${EXIT_CODE}
}

# 捕获Ctrl+C信号
trap 'echo ""; print_warning "收到中断信号，正在退出..."; exit 130' INT TERM

# 执行主函数
main "$@"

