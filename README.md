# RAICOM 2026 智慧养老组任务一运行手册

本文件用于下一次打开虚拟机后，按步骤恢复任务一运行环境。  
目标：启动 MuJoCo 仿真、启动 mc 运动控制模块、启动 Tk 遥控器，控制机器人进入交互区。

---

## 0. 核心文件

任务一主要使用：

```text
competition/run_tk_remote.sh              # 启动任务一 Tk 遥控器
competition/tk_remote_keyboard_proxy.py   # Tk 遥控器主程序
competition/ros2_overlay/                 # x86_64 环境下重新编译的 aimdk_msgs
```

注意：

```text
不要删除 competition/ros2_overlay/
不要删除 competition/run_tk_remote.sh
不要删除 competition/tk_remote_keyboard_proxy.py
```

---

## 1. 每次启动需要几个终端

一共打开 3 个 Ubuntu 终端：

```text
终端 1：启动 MuJoCo 仿真平台
终端 2：启动 mc 运动控制模块
终端 3：启动任务一 Tk 遥控器
```

启动顺序：

```text
1. 终端 1 启动仿真
2. 终端 2 启动 mc
3. MuJoCo 窗口点击 Reset
4. 终端 3 启动遥控器
5. 遥控器点击 SD
6. 等机器人站稳 2 秒
7. 遥控器点击 LD
8. 使用摇杆控制机器人进入交互区
9. 到达后点击急停
```

---

## 2. 终端 1：启动 MuJoCo 仿真平台

打开第一个 Ubuntu 终端，执行：

```bash
docker start x2_deploy
docker exec -it x2_deploy bash --noprofile --norc
```

进入容器后执行：

```bash
export PS1='\u@\h:\w\$ '

source /opt/ros/humble/setup.bash
source /home/agi/x2_deploy_workspace/competition/ros2_overlay/install/setup.bash

cd /home/agi/x2_deploy_workspace/Raicom2026/sim_mujoco/bin
./start_sim.sh -s
```

说明：

```text
docker start x2_deploy
启动比赛 Docker 容器。

docker exec -it x2_deploy bash --noprofile --norc
进入容器，并跳过旧的 .bashrc，避免加载错误的 aimdk_msgs 路径。

source /opt/ros/humble/setup.bash
加载 ROS2 Humble 环境。

source /home/agi/x2_deploy_workspace/competition/ros2_overlay/install/setup.bash
加载重新编译后的 x86_64 aimdk_msgs 环境。

./start_sim.sh -s
启动 MuJoCo 比赛仿真场景。
```

成功后会出现 MuJoCo 窗口。  
这个终端不要关闭。

---

## 3. 终端 2：启动 mc 运动控制模块

打开第二个 Ubuntu 终端，执行：

```bash
docker exec -it x2_deploy bash --noprofile --norc
```

进入容器后执行：

```bash
export PS1='\u@\h:\w\$ '

source /opt/ros/humble/setup.bash
source /home/agi/x2_deploy_workspace/competition/ros2_overlay/install/setup.bash

cd /home/agi/x2_deploy_workspace/Raicom2026/mc/bin
./em_run.sh
```

说明：

```text
./em_run.sh
启动 mc 运动控制模块。

SD / LD 模式切换、机器人行走控制都依赖 mc。
如果 mc 没有启动，遥控器里点击 SD 或 LD 会失败。
```

这个终端不要关闭。

---

## 4. MuJoCo 窗口 Reset

终端 1 和终端 2 都启动后，回到 MuJoCo 窗口。

在 MuJoCo 左侧面板点击：

```text
Simulation -> Reset
```

说明：

```text
Reset 用于让机器人回到初始状态。
如果机器人倒地、卡住或姿态异常，先点击 Reset。
```

注意：Reset 是 MuJoCo 窗口里的按钮，不是终端命令。

---

## 5. 终端 3：启动任务一 Tk 遥控器

打开第三个 Ubuntu 终端，执行：

```bash
docker exec -it x2_deploy bash --noprofile --norc
```

进入容器后执行：

```bash
export PS1='\u@\h:\w\$ '

source /opt/ros/humble/setup.bash
source /home/agi/x2_deploy_workspace/competition/ros2_overlay/install/setup.bash

cd /home/agi/x2_deploy_workspace/competition
./run_tk_remote.sh
```

说明：

```text
./run_tk_remote.sh
启动任务一 Tk 图形遥控器。

遥控器会调用 tk_remote_keyboard_proxy.py。
tk_remote_keyboard_proxy.py 会启动官方 keyboard.py，并通过 w/s/q/e/space 控制机器人。
```

成功后会出现 Tk 遥控器窗口。

---

## 6. 遥控器操作顺序

遥控器窗口出现后，按以下顺序操作：

```text
1. 点击 SD 稳定站立
2. 等机器人站稳 2 秒
3. 点击 LD 行走模式
4. 缓慢推动左侧 Forward 摇杆
5. 如果方向不正，短促推动右侧 Angular 摇杆
6. 机器人进入交互区后，点击急停 E-STOP
7. 确认机器人稳定停住
```

按钮含义：

```text
SD 稳定站立
让机器人进入稳定站立模式。

LD 行走模式
让机器人进入行走模式。

Forward 前进摇杆
控制机器人向前走。

Angular 旋转摇杆
控制机器人左右旋转。

急停 E-STOP
立即停止机器人。

退出 EXIT
关闭遥控器，并发送停止指令。
```

比赛建议：

```text
优先使用 Forward 前进。
Angular 只做短促微调。
不要长时间大幅旋转，否则容易小碎步或失稳。
到达交互区后立即急停。
```

---

## 7. 键盘快捷键

当遥控器窗口获得焦点时，也可以使用键盘：

```text
W：前进加一档
S：前进减一档
Q：左转
E：右转
X：前进归零
C：旋转归零
空格：急停
```

比赛中最重要的是：

```text
空格：急停
```

---

## 8. 常见问题处理

### 8.1 SD / LD 点击失败

现象：

```text
Service not available
```

原因：

```text
mc 没有启动，或者 mc 服务还没有注册完成。
```

处理：

回到终端 2，确认 `./em_run.sh` 正在运行。

也可以在终端 3 检查服务：

```bash
ros2 service list -t | grep -i -E "aimdk|mc|action|set"
```

如果能看到：

```text
/aimdk_5Fmsgs/srv/SetMcAction
```

说明 mc 服务已经存在，可以重新点击 SD / LD。

---

### 8.2 aimdk_msgs 类型支持错误

现象：

```text
Could not import 'rosidl_typesupport_c' for package 'aimdk_msgs'
```

原因：

```text
当前终端加载了错误的 aarch64 aimdk_msgs 环境。
```

处理：

不要这样进入容器：

```bash
docker exec -it x2_deploy bash
```

应该这样进入容器：

```bash
docker exec -it x2_deploy bash --noprofile --norc
```

然后重新加载正确环境：

```bash
source /opt/ros/humble/setup.bash
source /home/agi/x2_deploy_workspace/competition/ros2_overlay/install/setup.bash
```

---

### 8.3 容器提示符变成 bash-5.1$

这是正常现象，因为使用了：

```bash
bash --noprofile --norc
```

如果想恢复显示路径，执行：

```bash
export PS1='\u@\h:\w\$ '
```

---

### 8.4 机器人倒地或姿态异常

处理顺序：

```text
1. 点击遥控器急停 E-STOP
2. 回到 MuJoCo 窗口
3. 点击 Simulation -> Reset
4. 遥控器中重新点击 SD
5. 等待机器人站稳
6. 点击 LD
7. 继续遥控
```

---

### 8.5 仿真严重卡死

如果 MuJoCo 窗口卡死，终端无响应，可以在宿主机新开终端执行：

```bash
docker stop x2_deploy
docker start x2_deploy
```

然后重新按照本文档：

```text
终端 1 -> 终端 2 -> Reset -> 终端 3
```

的顺序启动。

---

## 9. 比赛当天极简流程

### 终端 1

```bash
docker start x2_deploy
docker exec -it x2_deploy bash --noprofile --norc
export PS1='\u@\h:\w\$ '
source /opt/ros/humble/setup.bash
source /home/agi/x2_deploy_workspace/competition/ros2_overlay/install/setup.bash
cd /home/agi/x2_deploy_workspace/Raicom2026/sim_mujoco/bin
./start_sim.sh -s
```

### 终端 2

```bash
docker exec -it x2_deploy bash --noprofile --norc
export PS1='\u@\h:\w\$ '
source /opt/ros/humble/setup.bash
source /home/agi/x2_deploy_workspace/competition/ros2_overlay/install/setup.bash
cd /home/agi/x2_deploy_workspace/Raicom2026/mc/bin
./em_run.sh
```

### MuJoCo

```text
Simulation -> Reset
```

### 终端 3

```bash
docker exec -it x2_deploy bash --noprofile --norc
export PS1='\u@\h:\w\$ '
source /opt/ros/humble/setup.bash
source /home/agi/x2_deploy_workspace/competition/ros2_overlay/install/setup.bash
cd /home/agi/x2_deploy_workspace/competition
./run_tk_remote.sh
```

### 遥控器

```text
SD -> 等 2 秒 -> LD -> Forward 前进 -> Angular 微调 -> 急停
```
