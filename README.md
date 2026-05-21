# ros_work - 魔力元宝 ROS 机器人项目

2026 睿抗机器人开发者大赛 (RAICOM) 魔力元宝赛题 - 工业组

## 项目结构

| 功能包 | 说明 |
|--------|------|
| `state_machine_pkg` | **工业组省赛任务一状态机** — 顺序执行状态机框架 |
| `ar_tf_arm_5_pkg` | 主控节点 — 集成导航/机械臂/AR定位/吸盘 |
| `arm_control_pkg` | 机械臂控制 — SwiftPro 机械臂 + 吸盘 |
| `auto_charge_pkg` | 自动回充 — 导航到充电桩并精确定位 |
| `nav_goal_pkg` | 导航客户端 — 向 move_base 发送目标点 |
| `romove_pkg` | 相对移动客户端 — 小范围位置微调 |

## 状态机 (state_machine_pkg)

### 任务一流程

```
Init → WaitStart → NavToPickPoint1 → RecognizeAR1 → Grasp1 → Store1
         → NavToPickPoint2 → RecognizeAR2 → Grasp2 → Store2
         → NavToDeparture → WaitUnload → NavToCharge
         → DoCharge → NavToDepartureFinal → Finished
```

### 评分项对应

| 子任务 | 分值 | 对应状态 |
|--------|------|----------|
| 取货点一 | 30分 | NavToPickPoint1 → RecognizeAR1 → Grasp1 → Store1 |
| 取货点二 | 30分 | NavToPickPoint2 → RecognizeAR2 → Grasp2 → Store2 |
| 货物出库 | 20分 | NavToDeparture → WaitUnload |
| 自主回充 | 30分 | NavToCharge → DoCharge |
| 返回出发区 | 20分 | NavToDepartureFinal |

### 编译

```bash
cd /home/reicom2025/ros_workspace
source /opt/ros/noetic/setup.bash
source devel/setup.bash  # 如果已编译过
catkin_make
```

### 运行状态机（实际机器人）

```bash
# 终端1: 启动机器人驱动和导航
roslaunch your_robot_bringup.launch

# 终端2: 运行状态机
rosrun state_machine_pkg state_machine_node

# 终端3: 发布开始信号（或接裁判系统）
rostopic pub /competition/start std_msgs/Empty "{}"
```

### 测试模式（无需硬件）

```bash
# 终端1: 启动 ROS 核心
roscore

# 终端2: 运行测试节点
rosrun state_machine_pkg test_state_machine_node
```

测试节点会自动发布开始信号，状态机在模拟模式下执行完整流程。

### 参数配置

所有参数可通过 roslaunch 或 rosparam 设置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~ar_id_1` | 1 | 1号位AR码ID |
| `~ar_id_2` | 3 | 2号位AR码ID |
| `~ar_dist` | 0.4 | AR识别距离(m) |
| `~pick_point_1_x/y` | 0.5/1.5 | 1号加工中心坐标 |
| `~pick_point_2_x/y` | 1.5/1.5 | 2号加工中心坐标 |
| `~departure_x/y` | 0.0/0.0 | 出发区坐标 |
| `~charge_x/y` | 0.0/2.0 | 充电桩坐标 |
| `~timeout_nav` | 60.0 | 导航超时(秒) |
| `~timeout_recognize` | 15.0 | AR识别超时(秒) |
