#!/usr/bin/env bash

# 不用 set -u，避免 source ROS setup 时 AMENT_TRACE_SETUP_FILES 之类变量触发 unbound variable
set -Ee -o pipefail

BASE="/home/agi/x2_deploy_workspace"
COMP="$BASE/competition"

export PS1='\u@\h:\w\$ '

# 加载 ROS 和你刚编译成功的 x86_64 aimdk_msgs overlay
source /opt/ros/humble/setup.bash
source "$COMP/ros2_overlay/install/setup.bash"

cd "$COMP"

echo "当前目录：$(pwd)"
echo "检查 tkinter..."
python3 - <<'PY'
import tkinter
print("tkinter OK")
PY

echo "启动 Tk 遥控器..."
python3 "$COMP/tk_remote_keyboard_proxy.py"
