# Gazebo 操作说明

适用版本：**Gazebo Classic 11 + ROS 2 Humble**。本手册用于软件入门与讲解，其他版本的界面、命令和插件可能不同。Classic 已于 2025 年 1 月结束维护；当前项目沿用现有环境。

[打开图文交互版](Gazebo操作说明.html) · [查看项目开发与演示手册](wheel_leg_demo/doc/开发与演示手册.md)

## 1. 简介

Gazebo 是机器人仿真软件：在虚拟场景中放置机器人、障碍物和传感器，观察运动、碰撞及传感器数据，用于开发和验证机器人程序。

它能模拟重力、摩擦、刚体接触、关节和相机等传感器。是否真的进行了这些模拟，取决于模型、物理参数和插件的配置；画面里机器人在动，不代表已经实现真实动力学控制。

**Gazebo 负责虚拟世界；ROS 2 负责程序之间的通信和算法组织。** 两者通过插件连接：Gazebo 向 ROS 2 提供状态和传感器数据，ROS 2 向仿真发送速度、关节或轨迹指令。Gazebo 可以单独运行；ROS 2 不需要 roscore。

`gzserver` 是服务端，运行世界、物理和插件；`gzclient` 是图形客户端，显示场景与提供操作界面。RViz 是 ROS 数据可视化工具，不负责物理仿真，也不是打开 Gazebo 的前置条件。

本项目使用 Gazebo 展示轮足机器人的规划与动作。规划结果由算法计算，执行器按指令更新模型和关节位姿，属于**运动学演示**，没有实现接触平衡与电机力矩控制。

![当前项目的真实 Gazebo 场景](wheel_leg_demo/doc/results/3.0_地图点位/全景.png)

下方交互版提供软件关系图；最后一章的视频展示本项目效果，不是 Gazebo 软件操作录像。

## 2. 操作

**启动：本机终端执行。** Docker 服务应处于 active；如未启动，在本机执行 sudo systemctl start docker。

```bash
systemctl is-active docker
xhost +SI:localuser:root
docker start hfun_humble_cuda_dev
docker exec -it -e DISPLAY="$DISPLAY" hfun_humble_cuda_dev bash --norc
```

**准备环境：每个新容器终端执行。** 以下设置只影响当前终端及其启动的程序；ROS 2 通信设置保持一致。

```bash
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export ROS_DOMAIN_ID=76
export ROS_LOCALHOST_ONLY=1
cd /root/Projects/gazebo/gazebo_manual/wheel_leg_demo
source install/setup.bash
```

**下面三种启动方式选一种，同一默认端口不要重复启动。**

| 目的 | 命令（容器内） | 说明 |
|---|---|---|
| 打开空场景 | gazebo | 启动服务端和图形界面，不加载 ROS 插件 |
| 打开指定场景 | gazebo /root/Projects/gazebo/gazebo_manual/wheel_leg_demo/src/wheel_leg_demo/worlds/two_steps.world | 查看场景文件；不会自动装入 Xacro 机器人和项目执行器 |
| 打开完整项目 | ros2 launch wheel_leg_demo demo.launch.py | 组装场景、机器人和执行插件，启动仿真 |

完整项目首次使用或修改 C++ 后，先在 /root/Projects/gazebo/gazebo_manual/wheel_leg_demo 执行：

```bash
colcon build --symlink-install --base-paths src --packages-select wheel_leg_demo
source install/setup.bash
```

**界面怎么看：** 左侧 World 查看模型、连杆、关节及属性；Insert 插入模型；中央为三维视口；上方工具栏用于选择、移动、旋转等；底部提供播放/暂停、单步及时间信息。在线模型库可能需要网络，本地模型可离线使用。

| 操作 | 鼠标、键盘或菜单 | 作用 |
|---|---|---|
| 平移观察视角 | 在空白处按住鼠标左键拖动 | 移动观察位置；选择模型后注意不要误用模型移动工具 |
| 旋转观察视角 | 按住鼠标中键拖动 | 绕观察点旋转 |
| 拉近或拉远 | 鼠标滚轮；或右键拖动 | 改变观察距离 |
| 聚焦模型 | 双击模型 | 将视角转向模型 |
| 跟随模型 | 模型右键菜单中的 Follow（如当前配置提供） | 观察移动中的机器人 |
| 移动、旋转模型 | 选择工具栏对应工具，拖动坐标轴 | 修改模型位姿；搭建时建议先暂停 |
| 暂停、继续 | 底部播放/暂停按钮 | 停止或推进仿真时间 |
| 单步 | 暂停后点击底部单步按钮 | 推进一个仿真步，不是执行一个机器人动作 |
| 复位 | Edit 菜单中的 Reset Model Poses / Reset World | 前者复位模型位姿，后者还会重置世界时间；不重置外部 ROS 节点状态 |
| 保存场景 | File → Save World As；Ctrl+S 保存 | 保存世界；不自动同步源 Xacro、YAML 或 ROS 程序 |
| 结束前台程序 | 启动终端 Ctrl+C | 停止当前 launch 或 Gazebo 进程 |

鼠标操作按默认相机控制说明；自定义配置可能不同。菜单名称及工具栏提示是当前安装版本的操作依据。

**常用命令：** Gazebo 命令检查仿真；ROS 2 命令检查插件提供的通信接口。先 list，再对实际存在的名字调用 info、echo 或 service。

```bash
# Gazebo Classic：版本、当前模型与 Gazebo 自己的传输话题
gazebo --version
gz model --list
gz topic -l

# ROS 2：节点、话题、服务与参数
ros2 node list
ros2 topic list
ros2 topic info /wheel_leg_demo/odom
ros2 topic echo /wheel_leg_demo/odom --once
ros2 service list -t
ros2 param list /robot_state_publisher
```

Gazebo Transport 的 gz topic 与 ROS 2 的 ros2 topic 属于不同通信系统，名字不一定一致。本项目 ROS 话题需要通过项目 launch 加载插件才会出现。

```bash
# 本项目已加载 gazebo_ros 时：暂停、恢复仿真
ros2 service call /pause_physics std_srvs/srv/Empty "{}"
ros2 service call /unpause_physics std_srvs/srv/Empty "{}"

# 项目自定义：查看动作状态、变足式、变轮式
ros2 topic echo /wheel_leg_demo/status --once
ros2 service call /wheel_leg_demo/to_leg std_srvs/srv/Trigger "{}"
ros2 service call /wheel_leg_demo/to_wheel std_srvs/srv/Trigger "{}"
```

项目自定义服务不是 Gazebo 的通用功能。Accepted 表示接收，完成情况需要查询状态；不要与正在运行的自动任务同时发送手动动作。完整规划和演示命令见项目开发与演示手册。

**关闭：** 有任务时先停止任务，再在规划、仿真各自的启动终端按 Ctrl+C；exit 离开容器。只关闭 Gazebo 窗口可能仍保留服务端。需要停止整个容器时，在本机执行 docker stop hfun_humble_cuda_dev。

项目运行中优先使用项目的 reset_demo（需启动任务节点）或 reset 服务；Gazebo 世界复位不会同步重置规划器与任务状态。

## 3. 文件说明

文件扩展名是入口，实际要看它描述的是世界、模型、启动过程还是参数。world 文件通常也使用 SDF 格式。

| 文件 | 功能 | 主要内容 |
|---|---|---|
| .world | 定义整个仿真世界 | 地面、光源、物理设置、模型、世界插件 |
| .sdf | 描述世界或模型，Gazebo 原生格式 | model、link、joint、sensor、plugin 等 |
| .urdf | 描述机器人的连杆和关节树 | link、joint、visual、collision、inertial |
| .urdf.xacro | 用变量、宏和表达式组织 URDF | 重复部件、尺寸参数；加载前展开为 URDF |
| model.config | Gazebo 模型库的描述入口 | 模型名称、版本及对应的 SDF 文件 |
| .dae / .stl / .obj | 外观或碰撞网格资源 | 三维形状；不包含完整关节和控制逻辑 |
| .launch.py | ROS 2 启动文件 | 启动节点、Gazebo、加载参数和模型 |
| .yaml | 参数配置 | 尺寸、初始位姿、算法限制等；需要程序读取才生效 |
| .cpp / .so | 插件源码 / 编译后的动态库 | 传感器、状态接口、驱动或自定义仿真行为 |
| CMakeLists.txt / package.xml | ROS 2 工程构建与依赖 | 编译目标、依赖、运行资源安装位置 |

**模型内部的基本关系：** model 是整体；link 是刚性部件；joint 连接部件并规定运动方向、类型和限位。visual 决定看见什么，collision 决定碰撞形状，inertial 定义质量与惯性。外观网格不等于碰撞模型。

本项目对应文件：

```text
src/wheel_leg_demo/
├── worlds/two_steps.world       场景：平台、楼梯、障碍物
├── urdf/robot.urdf.xacro         机器人：部件与关节
├── config/demo.yaml             点位、机器人与规划参数
├── launch/demo.launch.py        组装并启动仿真
├── launch/planning.launch.py    启动规划与任务
├── src/demo_node.cpp            运动学执行插件源码
├── CMakeLists.txt               编译和资源安装
└── package.xml                 包信息与依赖
```

demo.launch.py 会展开 Xacro，用 gz sdf 转换成 SDF，再把机器人和执行插件加入临时 world。因此只打开源 two_steps.world，不会得到完整项目。修改临时 world 不会改变项目源码。

## 4. 场景与机器人搭建

**先定单位与坐标。** 长度用米、角度用弧度、质量用千克。本项目 world 的 z 向上；机器人本体约定 x 向前、y 向左、z 向上。SDF pose 的顺序为 x y z roll pitch yaw；模型位姿与连杆内部位姿通常属于不同层级的坐标。

**创建场景与障碍物：** 打开空场景，暂停，通过工具栏插入 Box、Sphere 等基本形状，选择后用移动/旋转工具布置，用属性面板检查尺寸与位姿；保存为 world。需要多个部件组成一个模型时，使用 Edit → Model Editor。适合先直观看布局，再将参数写回源文件。

下面是可以直接保存并打开的独立场景。地面上有一个 0.6 × 0.8 × 0.6 m 的静态障碍物；其中心 z=0.3 m，所以底面刚好接地。

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="practice">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <model name="obstacle">
      <static>true</static>
      <pose>1 0 0.3 0 0 0</pose>
      <link name="body">
        <collision name="collision">
          <geometry><box><size>0.6 0.8 0.6</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.6 0.8 0.6</size></box></geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

保存到容器 /root/Projects/practice.world 后执行 gazebo /root/Projects/practice.world。sun、ground_plane 需要本地标准模型资源可用；本例不含 ROS 插件。固定平台和楼梯通常设 static=true，避免作为自由物体掉落。

**搭建机器人：** 先用盒子、圆柱等基本形状表示机身、轮子与腿；每个独立运动的刚体建一个 link，再用 joint 连接。轮轴用 continuous 或 revolute，直线收轮机构可用 prismatic，固定部件用 fixed。用轴向、原点、限位确定关节如何运动，最后添加质量、惯性和碰撞，再接入驱动或自定义插件。

下面说明 URDF 轮关节的连接方式，片段依赖已定义的 base_link 和 left_wheel，不能单独作为完整机器人加载。

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <origin xyz="0 0.21 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

关节定义只规定连接与允许的运动，**不会自己转动**。ROS 的 robot_state_publisher 根据机器人描述和关节状态发布 TF，也不驱动 Gazebo；需要插件或控制器实际执行指令。当前项目使用自定义运动学插件，通用物理驱动机器人则需要相应轮驱动、关节控制及物理参数。

**修改现有项目：** 在 two_steps.world 修改场景几何，在 robot.urdf.xacro 修改机器人，在 demo.yaml 同步规划器使用的障碍物、范围和尺寸。修改 C++ 后重新编译；模型、配置和 launch 改动后重启仿真。改变机器人的结构尺寸还需要检查执行器与足式规划的几何假设。

例如改变楼梯高度，必须同时核对 world 中两级台阶及上平台的尺寸和中心高度，以及 YAML 的 scene.stair.rise、上平台点位高度；不能只改其中一个文件。GUI 中拖动障碍物不会自动更新规划器的已知地图。

检查结果时，先看外观和尺寸，再查看碰撞几何、关节轴与初始位姿；真实动力学仿真还需检查质量、惯量、接触与驱动。保存源码后重启，确认模型没有穿插、楼梯连续、机器人站在预期位置。

[观看本项目三画面演示视频](wheel_leg_demo/doc/results/4_自动演示/三画面同步演示_1080p.mp4)

视频展示轮式行驶、变形、足式上楼及规划线；它说明本项目如何使用 Gazebo，不代表已完成真实物理步态控制。
