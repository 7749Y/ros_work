# 任务一 单节点使用流程
> 前提：必须已安装并配置好虚拟资源环境

## 一、前置准备：创建功能包
1. 在 `ros_workspace` 下创建 `src_Task_one` 文件夹（推荐在 VS Code 中创建）
2. 打开终端，切换到 `src_Task_one` 目录
3. 执行命令创建功能包
   ```bash
   catkin_create_pkg <包名> rospy roscpp <依赖项1> <依赖项2>

依赖项为头文件 / 前面的部分，可直接复制使用
在功能包的 src 目录下创建 .cpp 文件
必须修改 CMakeLists.txt，添加编译节点与链接节点
不会配置可直接下载现成的 CMakeLists.txt 使用

二、编译功能包
终端切换到 ros_workspace 路径
执行编译命令
catkin_make

三、运行节点
1.启动虚拟仿真
roslaunch oryxbot_navigation demo_nav_2d.launch
2.启动对应服务:
开启相对移动
roslaunch relative_move relative_move.launch
开启底盘二次定位
roslaunch ar_pose ar_base_sim.launch
机械臂服务后续更新
3.刷新环境变量
source ~/ros_workspace/devel/setup.bash
4.运行节点
rosrun <包名> <节点名>