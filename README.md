# Wheel Leg Demo

ROS 2 / Gazebo 轮足机器人规划演示：A→E 轮式绕障，变足式后 E→X 上楼，再恢复轮式驶向 B。全局二维 A*、轮式精简 TEB、足步 A*＋五次轨迹。执行为运动学展示，不含接触平衡或电机力矩控制。

## 环境

Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11、C++17。Python 用于 launch。依赖在 src/wheel_leg_demo/package.xml 中声明，包含 Gazebo ROS、Ceres、TBB、Xacro 等。其他 ROS / Gazebo 版本未经验证。

工作空间根目录就是本 README 所在目录。可克隆到任意位置，不需要 /root/Projects 或作者的用户名。

## 克隆、依赖与编译

在已安装 ROS 2 Humble 和 Gazebo Classic 11 的环境中：

```bash
git clone <仓库URL> wheel_leg_demo
cd wheel_leg_demo
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
# 如尚未安装：sudo apt install python3-rosdep python3-colcon-common-extensions
# 首次配置 rosdep 时执行一次：sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src --packages-select wheel_leg_demo
source install/setup.bash
```

使用未加载旧项目 install 环境的新终端。编译、安装和运行不依赖源码所在的绝对路径；build/install 中生成的路径属于本机环境，换目录或换电脑必须重新构建。上传 GitLab 时不提交 build、install、log，仓库的 .gitignore 已排除它们。

## 启动演示

每个终端先进入本工作空间并执行：

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
export ROS_DOMAIN_ID=76
export ROS_LOCALHOST_ONLY=1
```

终端1：

```bash
ros2 launch wheel_leg_demo demo.launch.py
# 无界面验证可加 gui:=false
```

终端2：

```bash
ros2 launch wheel_leg_demo planning.launch.py task:=true
```

终端3：等仿真服务和机器人状态就绪后执行。

```bash
ros2 service call /wheel_leg_demo/reset_demo std_srvs/srv/Trigger "{}"
ros2 topic echo /wheel_leg_demo/task_status --qos-durability transient_local
# 看到 READY 后 Ctrl+C 退出状态查看，再执行：
ros2 service call /wheel_leg_demo/start_demo std_srvs/srv/Trigger "{}"
ros2 topic echo /wheel_leg_demo/task_status --qos-durability transient_local
```

Accepted 仅表示接收；DONE 才表示完成。停止使用 stop_demo，复位使用 reset_demo。结束时先停止任务，再在两个启动终端 Ctrl+C。

## 结果目录

默认写到运行用户的 ~/.local/state/wheel_leg_demo：

```text
global/baseline/  独立全局规划结果
wheel_local/      独立轮式局部结果
foot_local/       独立足式局部结果
task/             完整任务结果
```

自定义结果根目录（示例使用绝对路径以避免工作目录歧义）：

```bash
ros2 launch wheel_leg_demo planning.launch.py task:=true results_dir:="$PWD/results"
```

如写到源码内的 results 目录，请自行加入本地忽略规则，避免将临时运行记录提交。doc/results 是已归档验收材料，不会被默认运行覆盖。完整任务将各子模块记录归入 task/<case_name>；同名案例重跑会覆盖。可用 ros2 param set /global_planner task.case_name manual_02 更换案例名称。

config/demo.yaml 中各 output_dir 是相对 results_dir 的子目录；也允许填绝对路径。全局输出可单独通过 global_output_dir 启动参数覆盖。直接运行 global_planner 可使用 output_dir、wheel.output_dir、foot.output_dir、task.output_dir ROS 参数；默认同样位于用户结果目录。不要在 YAML 里写未展开的 ~ 或 $HOME。

## 文件

```text
src/wheel_leg_demo/   ROS 包：算法、插件、模型、场景、配置与 launch
doc/                 PRD、开发手册、离线讲解与历史验收材料
wheel_leg_demo.code-workspace  可选的 VS Code 工作空间入口
```

VS Code可通过“打开工作区”选择wheel_leg_demo.code-workspace，CMake路径随仓库位置变化。

详细操作：[开发与演示手册](doc/开发与演示手册.md)。项目要求：[PRD](doc/项目PRD.md)。离线讲解：[轮足项目](doc/轮足项目.html)，内嵌图片与视频，可单独分享。

Docker 是本机提供环境的方式，不是源码必需条件。同事可使用兼容的原生环境或自己的容器；容器需正确配置图形界面、ROS 通信以及依赖。公司镜像的访问权限由公司提供。
