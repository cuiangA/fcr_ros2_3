# 团队快速上手指南

本文档面向已经加入 GitHub 仓库的四位同学，用于帮助大家快速理解自己负责的模块、判断代码中已经完成到什么程度、配置运行环境，并合理使用 vibe coder 类工具辅助阅读和小步开发。

本文默认沿用当前分组：

| 组别 | 负责方向 | 对应任务 |
| --- | --- | --- |
| B 组 | 硬件平台与基础控制 | 任务 2、5，以及部分任务 10 |
| C 组 | 视觉感知与目标状态估计 | 任务 3、4 |
| D 组 | 视觉伺服与协同控制 | 任务 6、7 |
| E 组 | 任务运镜与人机交互 | 任务 8、9，以及部分 bringup/可视化 |

如果后续组名调整，只需要按“负责方向”对应替换组名即可。

## 1. 先理解项目主流程

所有同学先统一理解这条主链路：

```text
相机图像 / 深度图
  -> YOLO 检测
  -> 多目标跟踪
  -> 目标 3D 状态估计
  -> MVP / IBVS / PBVS 视觉伺服控制
  -> ControlAllocator 控制分配
  -> /cmd_vel + /cmd_gimbal
  -> 底盘 + 云台执行
  -> /platform/state 反馈
```

当前项目处于 V2.5 左右：MVP 闭环、目标跟踪、深度估计、IBVS/PBVS、控制分配、仿真和平台抽象基本具备；真实 YOLO 推理、真实硬件协议、MPC/QP、完整运镜 Action 和上层交互还需要继续补齐。

每个同学第一次打开仓库时，建议先让 vibe coder 只读这些文件，不要改代码：

```text
README.md
docs/project_documentation.md
docs/uml/09_layered_architecture.puml
D:/githubRepository/notes/fcr_ros2/任务划分.md
```

推荐第一条提示词：

```text
请只阅读本仓库和我指定的文档，不要修改任何文件。
请用 10 分钟项目导读的方式说明：
1. 这个项目的总体目标是什么；
2. 从感知层到执行层的数据流是什么；
3. 当前代码已经完成哪些能力；
4. 哪些能力还只是框架或 TODO；
5. 我应该先看哪些 package。
所有结论都请标注对应文件路径。
```

## 2. 环境配置

本项目统一按原生 Ubuntu 22.04 + ROS 2 Humble 配置。需要参与真实硬件调试的同学应使用原生 Ubuntu 机器，或至少能远程访问线下 Ubuntu 调试机。

### 2.1 推荐环境

```text
操作系统：Ubuntu 22.04 LTS
ROS 版本：ROS 2 Humble Hawksbill
构建工具：colcon + ament_cmake
依赖管理：rosdep
C++ 标准：C++17
Python：Python 3
```

先检查系统版本：

```bash
lsb_release -a
uname -m
```

期望结果：

```text
Ubuntu 22.04.x LTS
x86_64 或 aarch64
```

### 2.2 设置 UTF-8 locale

ROS 2 官方安装流程要求系统 locale 支持 UTF-8。新装系统或最小化系统先执行：

```bash
locale
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

最后一次 `locale` 输出中应能看到 `UTF-8`。

### 2.3 启用 Ubuntu Universe 软件源

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
```

如果 `add-apt-repository` 询问是否继续，按回车确认。

### 2.4 添加 ROS 2 apt 源

ROS 2 Humble 当前推荐通过 `ros2-apt-source` 包配置 apt 源和 key：

```bash
sudo apt update
sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

不要使用旧的 `apt-key` 方式配置 ROS 源。

### 2.5 安装 ROS 2 Humble

更新软件列表并升级系统关键包：

```bash
sudo apt update
sudo apt upgrade -y
```

安装桌面版 ROS 2 Humble。桌面版包含 ROS 基础库、RViz、常用 demo 和可视化工具，适合本项目：

```bash
sudo apt install -y ros-humble-desktop
```

安装 ROS 开发工具：

```bash
sudo apt install -y ros-dev-tools
```

如果机器只用于无图形的远程构建，可以把 `ros-humble-desktop` 换成 `ros-humble-ros-base`；但本项目需要 RViz/Gazebo 调试，推荐安装 desktop。

### 2.6 配置 ROS 环境变量

当前终端立即生效：

```bash
source /opt/ros/humble/setup.bash
```

写入 `~/.bashrc`，以后每次打开终端自动加载：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

检查 ROS 命令是否可用：

```bash
which ros2
ros2 --version
```

### 2.7 验证 ROS 2 安装

终端 1：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

如果 listener 能持续收到 `I heard`，说明 C++ 和 Python ROS 2 节点都能正常通信。

### 2.8 安装本项目常用依赖

先安装通用开发工具：

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake python3-pip \
  python3-colcon-common-extensions python3-rosdep \
  python3-vcstool python3-argcomplete \
  python3-opencv python3-numpy
```

再安装本项目常用 ROS 包：

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-ros
```

如果需要 Foxglove Web 可视化，再尝试安装：

```bash
sudo apt install -y ros-humble-foxglove-bridge
```

如果这一步提示找不到包，不影响基础编译和控制链调试，先跳过即可。

### 2.9 初始化 rosdep

首次使用 rosdep 的机器执行：

```bash
sudo rosdep init
rosdep update
```

如果 `sudo rosdep init` 提示已经初始化过，直接执行：

```bash
rosdep update
```

### 2.10 配置 Git

第一次在机器上提交代码前配置姓名和邮箱：

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```

建议先配置好 SSH key 或 GitHub CLI，再 clone 仓库。不会配 SSH 的同学可以先用 HTTPS clone。

### 2.11 拉取和构建仓库

```bash
source /opt/ros/humble/setup.bash
git clone <仓库地址> fcr_ros2_3
cd fcr_ros2_3
./build.sh
source install/setup.bash
```

仓库自带的 `build.sh` 会做三件事：

```text
1. source /opt/ros/humble/setup.bash
2. rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
3. colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
```

如果 rosdep 暂时有问题，可以先用于阅读和局部验证：

```bash
./build.sh --skip-rosdep
```

只构建自己负责的包及其依赖可以用：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to perception_pkg
source install/setup.bash
```

注意：修改 C++ 后需要重新 `colcon build`；每次新开终端都需要：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

也可以把当前工作区写入 `~/.bashrc`，但只建议固定在一份工作区开发时这么做：

```bash
echo "source ~/fcr_ros2_3/install/setup.bash" >> ~/.bashrc
```

如果你有多个 ROS 2 工作区，不要把多个 `install/setup.bash` 都写进 `~/.bashrc`，否则容易出现包版本混乱。

### 2.12 硬件调试权限

B 组或需要接真实串口/CAN 设备的同学额外配置：

```bash
sudo usermod -aG dialout $USER
```

执行后需要注销并重新登录。检查串口设备：

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

如果后续使用 CAN 调试工具：

```bash
sudo apt install -y can-utils
```

### 2.13 环境自检命令

```bash
source /opt/ros/humble/setup.bash
cd ~/fcr_ros2_3
source install/setup.bash
ros2 --version
colcon list
ros2 pkg list | grep vision_servo
ros2 interface show vision_servo_msgs/msg/Target
ros2 launch servo_control_pkg mvp_mock_test.launch.py --show-args
```

如果这些命令能正常输出，说明基础环境、工作区构建和接口生成基本可用。

### 2.14 常见问题

| 现象 | 常见原因 | 处理方式 |
| --- | --- | --- |
| `ros2: command not found` | 没有 source ROS 环境 | 执行 `source /opt/ros/humble/setup.bash` |
| `Package not found` | 工作区未构建或未 source | 执行 `./build.sh` 后 `source install/setup.bash` |
| `rosdep: command not found` | 没安装 rosdep | 执行 `sudo apt install python3-rosdep` |
| `rosdep init` 报已存在 | 已初始化过 | 直接执行 `rosdep update` |
| `colcon: command not found` | 没安装 colcon | 执行 `sudo apt install python3-colcon-common-extensions` |
| Gazebo/RViz 启动失败 | 图形环境或依赖缺失 | 先确认安装了 `ros-humble-desktop` 和 `ros-humble-gazebo-ros-pkgs` |
| 串口无权限 | 用户不在 `dialout` 组 | 执行 `sudo usermod -aG dialout $USER` 后重新登录 |

## 3. 合理使用 vibe coder 的方式

### 3.1 第一阶段只让它读代码

不要一上来就说“帮我完善整个模块”。第一轮应该让它做“代码地图”和“完成度判断”。

通用提示词：

```text
你现在是 FCR ROS2 项目的代码阅读助手。
请只阅读我负责的 package，不要修改代码。
请输出：
1. 这个 package 的职责；
2. 主要节点、launch、config、接口文件；
3. 输入 topic/service/action；
4. 输出 topic/service/action；
5. 当前已完成、框架完成、待补齐的功能；
6. 最小运行验证命令。
每条结论都要附文件路径。
```

完成度判断建议统一分三类：

| 状态 | 含义 |
| --- | --- |
| 已完成/可运行 | 代码逻辑完整，可以通过 launch、topic 或仿真做最小验证 |
| 框架完成 | 类、节点、接口、参数已存在，但核心算法、真实协议或具体策略还没填 |
| 待补齐 | 只有 TODO、头文件、占位实现，或还没有对应 package/node |

### 3.2 第二阶段再让它小步修改

真正改代码前，先让 vibe coder 给出计划：

```text
我准备实现 <具体小功能>。
请先不要修改代码，只输出：
1. 需要改哪些文件；
2. 为什么改这些文件；
3. 是否会影响其他组；
4. 改完以后用什么命令验证；
5. 有没有需要先确认的接口变化。
```

确认后再让它修改。每次只做一个小目标，例如：

```text
把 DetectionNode 的 ONNX Runtime 模型加载补上。
不要改 Target.msg，不要改 topic 名称，不要重构 perception_pkg 整体结构。
修改后给出构建命令和最小验证命令。
```

### 3.3 不建议让 AI 做的事

这些事情不要让 AI 自己拍脑袋完成：

- 不要在没有协议文档的情况下让它“实现真实底盘串口协议”。
- 不要让它随意修改 `vision_servo_msgs` 中的字段。
- 不要让它同时重构多个 package。
- 不要让它随意改 topic 名、service 名、frame_id、QoS。
- 不要只看 AI 总结就认为功能已完成，必须结合 launch、topic、源码和构建结果确认。

如果确实需要改公共接口，先在群里同步，因为这会影响 B/C/D/E 所有组。

## 4. GitHub 协作规范

### 4.1 分支建议

每组在自己的分支上做：

```bash
git checkout -b b-platform-control/<你的名字或功能名>
git checkout -b c-perception-state/<你的名字或功能名>
git checkout -b d-servo-control/<你的名字或功能名>
git checkout -b e-cinematic-hmi/<你的名字或功能名>
```

不要直接向主分支提交。

### 4.2 PR 描述模板

```text
标题：[C] 补齐 DetectionNode ONNX 推理入口

本次改动：
- ...

影响范围：
- package:
- topic/service/action:
- 是否修改公共接口：是/否

验证方式：
- colcon build ...
- ros2 launch ...
- ros2 topic echo ...

需要其他组注意：
- ...
```

### 4.3 不提交的内容

不要提交：

```text
build/
install/
log/
.vscode/个人配置
临时模型文件
本地录制的大 rosbag
```

模型文件、标定文件和 rosbag 如果确实要共享，先约定存放方式，不要随手塞进普通代码 PR。

## 5. B 组：硬件平台与基础控制

### 5.1 负责范围

B 组负责把底盘、云台、IMU、里程计和平台状态抽象成统一 ROS 2 接口。当前主要对应：

```text
任务 2：硬件驱动与设备抽象
任务 5：底盘与云台基础控制
任务 10：平台侧仿真、安全限幅、状态反馈的一部分
```

核心目标是让上层控制组不用关心真实硬件协议，只通过 `/cmd_vel`、`/cmd_gimbal` 和 `/platform/state` 与平台交互。

### 5.2 先读哪些文件

```text
src/robot_platform_pkg/package.xml
src/robot_platform_pkg/CMakeLists.txt
src/robot_platform_pkg/launch/platform.launch.py
src/robot_platform_pkg/config/chassis_params.yaml
src/robot_platform_pkg/config/gimbal_params.yaml
src/robot_platform_pkg/config/imu_params.yaml
src/robot_platform_pkg/config/odometry_params.yaml
src/robot_platform_pkg/include/robot_platform_pkg/hardware_interfaces/
src/robot_platform_pkg/include/robot_platform_pkg/kinematics/three_wheel_omni_kinematics.hpp
src/robot_platform_pkg/src/chassis_driver_node.cpp
src/robot_platform_pkg/src/gimbal_driver_node.cpp
src/robot_platform_pkg/src/imu_driver_node.cpp
src/robot_platform_pkg/src/odometry_node.cpp
src/robot_platform_pkg/src/platform_manager_node.cpp
src/vision_servo_msgs/msg/PlatformState.msg
src/vision_servo_msgs/msg/GimbalCmd.msg
```

B 组给 vibe coder 的阅读提示词：

```text
我是 B 组，负责硬件平台与基础控制。
请只阅读 robot_platform_pkg、相关 vision_servo_msgs 接口和 bringup 中平台相关 launch。
请整理：
1. 底盘、云台、IMU、里程计、PlatformManager 的数据流；
2. 每个节点订阅和发布的话题；
3. use_sim 参数如何切换真实/仿真后端；
4. 当前哪些是真实实现，哪些是仿真实现或 TODO；
5. 我可以用哪些命令做最小验证。
不要修改代码。
```

### 5.3 当前完成度判断

| 模块 | 当前状态 | 说明 |
| --- | --- | --- |
| 平台 launch | 已完成/可运行 | `platform.launch.py` 能启动底盘、云台、IMU、里程计、平台管理节点 |
| 三轮全向运动学 | 已完成/可运行 | 已有运动学头文件，可作为底盘速度和轮速转换基础 |
| 底盘 driver 节点 | 框架完成 | 订阅 `/cmd_vel`，支持 `use_sim`；真实 LEKIWI 串口通信仍是 TODO |
| 云台 driver 节点 | 框架完成 | 订阅 `/cmd_gimbal`，支持 `use_sim`；真实 DJI RS2 CAN 通信仍是 TODO |
| IMU driver 节点 | 框架完成 | 发布 `/imu/data`，支持 `use_sim`；真实 BNO055 I2C 通信仍是 TODO |
| 里程计节点 | 已完成/可运行 | 订阅 `/cmd_vel` 和 `/imu/data`，发布 `/odom` 和 TF |
| PlatformManager | 已完成/可运行 | 聚合 `/odom`、`/imu/data`、`/joint_states` 到 `/platform/state` |
| 设备心跳/连接状态 | 待补齐 | 当前生产环境连接状态和部分云台速度仍有 TODO |

### 5.4 B 组最小运行验证

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_platform_pkg
source install/setup.bash
```

启动仿真平台后端：

```bash
ros2 launch robot_platform_pkg platform.launch.py use_sim:=true
```

另开终端观察：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list
ros2 topic echo /platform/state
ros2 topic echo /odom
ros2 topic echo /imu/data
```

发布底盘速度命令：

```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.2}}}"
```

发布云台命令：

```bash
ros2 topic pub -r 5 /cmd_gimbal vision_servo_msgs/msg/GimbalCmd \
"{yaw_rate: 0.1, pitch_rate: 0.0, hold_yaw: false, hold_pitch: false}"
```

### 5.5 B 组与其他组接口

| 对接方 | B 组提供 | B 组依赖 |
| --- | --- | --- |
| D 组 | `/cmd_vel`、`/cmd_gimbal`、`/platform/state` 的稳定语义 | D 组输出命令不能超过约定速度范围 |
| E 组 | 手动接管、回中、状态查询的底层接口 | E 组不要绕过平台层直接控制硬件 |
| C 组 | 相机/深度相机后续可归入平台或感知驱动边界 | C 组需要明确图像 topic 和 camera frame |

线上同学可以做 B 组的接口梳理、仿真后端、参数和状态聚合；真实硬件协议和实机验证需要线下同学配合。

## 6. C 组：视觉感知与目标状态估计

### 6.1 负责范围

C 组负责把图像和深度数据变成稳定的目标状态。当前主要对应：

```text
任务 3：图像采集与视觉感知
任务 4：目标跟踪与状态估计
```

核心目标是输出 `/perception/targets_3d`，供 D 组视觉伺服控制使用。

### 6.2 先读哪些文件

```text
src/perception_pkg/package.xml
src/perception_pkg/CMakeLists.txt
src/perception_pkg/launch/perception.launch.py
src/perception_pkg/config/detection_params.yaml
src/perception_pkg/config/tracking_params.yaml
src/perception_pkg/config/depth_params.yaml
src/perception_pkg/include/perception_pkg/
src/perception_pkg/src/detection_node.cpp
src/perception_pkg/src/tracking_node.cpp
src/perception_pkg/src/multi_object_tracker.cpp
src/perception_pkg/src/depth_estimator_node.cpp
src/perception_pkg/src/perception_pipeline.cpp
src/vision_servo_msgs/msg/Target.msg
src/vision_servo_msgs/msg/TargetArray.msg
src/vision_servo_msgs/srv/SetTrackingTarget.srv
src/simulation_pkg/scripts/mock_detector.py
src/simulation_pkg/scripts/camera_simulator.py
```

C 组给 vibe coder 的阅读提示词：

```text
我是 C 组，负责视觉感知与目标状态估计。
请只阅读 perception_pkg、Target/TargetArray 接口、SetTrackingTarget 服务，以及 simulation_pkg 中 mock_detector/camera_simulator。
请整理：
1. /camera/image_raw 到 /perception/targets_3d 的完整数据流；
2. detection、tracking、depth_estimator、perception_pipeline 的职责区别；
3. 当前 YOLO 推理、跟踪、深度估计分别完成到什么程度；
4. 哪些参数在 config yaml 中；
5. 如何在没有真实相机和模型时用 mock 数据验证。
不要修改代码。
```

### 6.3 当前完成度判断

| 模块 | 当前状态 | 说明 |
| --- | --- | --- |
| DetectionNode | 框架完成 | 订阅 `/camera/image_raw`，发布 `/perception/detections`；`load_model()` 和 `infer()` 仍是占位 |
| TrackingNode | 已完成/可运行 | 基于检测结果做多目标跟踪，发布 `/perception/tracks`，支持 `SetTrackingTarget` |
| MultiObjectTracker | 已完成/可运行 | 使用 Kalman + IoU 关联维护目标 ID |
| DepthEstimatorNode | 已完成/可运行 | 融合 `/camera/depth/image_raw`、`/camera/camera_info` 和检测结果，输出 3D target |
| PerceptionPipeline | 已完成/可运行 | 可组合节点模式，将检测、跟踪、深度估计放在一个进程中 |
| Sony 相机真实接入 | 待补齐 | 当前还没有完整 Sony driver/采集卡接入 |
| YOLO ONNX/TensorRT 推理 | 待补齐 | 这是 C 组最关键的后续代码目标 |

### 6.4 C 组最小运行验证

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to perception_pkg simulation_pkg bringup_pkg
source install/setup.bash
```

查看感知 launch 参数：

```bash
ros2 launch perception_pkg perception.launch.py --show-args
```

启动感知管线：

```bash
ros2 launch perception_pkg perception.launch.py use_composition:=false
```

另开终端观察接口：

```bash
ros2 topic list | grep perception
ros2 topic info /perception/detections -v
ros2 topic info /perception/tracks -v
ros2 topic info /perception/targets_3d -v
ros2 service list | grep tracking
```

如果没有真实相机和 YOLO 模型，可以先用 bringup 的 mock detector 绕过 YOLO：

```bash
ros2 launch bringup_pkg fcr_bringup.launch.py use_sim:=true use_mock_detector:=true use_rviz:=false
```

目标选择服务验证。独立节点模式下服务名通常是 `/tracking/set_tracking_target`；可组合管线模式下通常是 `/perception_pipeline/set_tracking_target`，以 `ros2 service list | grep tracking` 的实际输出为准。

```bash
ros2 service call /tracking/set_tracking_target vision_servo_msgs/srv/SetTrackingTarget \
"{target_id: -1, class_name: 'person', enable: true}"
```

### 6.5 C 组与其他组接口

| 对接方 | C 组提供 | C 组依赖 |
| --- | --- | --- |
| D 组 | `/perception/targets_3d`，目标 bbox、center、position、tracking_id | D 组需要说明控制器需要哪些字段和 frame |
| E 组 | `SetTrackingTarget` 服务，目标 ID/类别选择能力 | E 组需要明确 UI/语音如何选择目标 |
| B 组 | 相机和深度图 topic 约定 | B 组或平台侧需要提供真实相机数据入口 |

C 组非常适合线上同学做，因为大部分工作可以通过源码、mock 数据、录制数据和模型文件完成。

## 7. D 组：视觉伺服与协同控制

### 7.1 负责范围

D 组负责从目标状态计算底盘和云台命令。当前主要对应：

```text
任务 6：视觉伺服与跟随控制
任务 7：优化控制与高级协同
```

核心目标是接收 `/perception/targets_3d` 和 `/platform/state`，输出 `/cmd_vel`、`/cmd_gimbal` 和 `/servo/state`。

### 7.2 先读哪些文件

```text
src/servo_control_pkg/package.xml
src/servo_control_pkg/CMakeLists.txt
src/servo_control_pkg/plugins.xml
src/servo_control_pkg/launch/servo_control.launch.py
src/servo_control_pkg/launch/mvp_mock_test.launch.py
src/servo_control_pkg/launch/mvp_2d_sim.launch.py
src/servo_control_pkg/config/mvp_follow_controller.yaml
src/servo_control_pkg/config/ibvs_params.yaml
src/servo_control_pkg/config/pbvs_params.yaml
src/servo_control_pkg/config/allocator_params.yaml
src/servo_control_pkg/include/servo_control_pkg/servo_controller_base.hpp
src/servo_control_pkg/include/servo_control_pkg/ibvs_controller.hpp
src/servo_control_pkg/include/servo_control_pkg/pbvs_controller.hpp
src/servo_control_pkg/include/servo_control_pkg/control_allocator.hpp
src/servo_control_pkg/include/servo_control_pkg/mpc_controller.hpp
src/servo_control_pkg/include/servo_control_pkg/rl_controller.hpp
src/servo_control_pkg/src/mvp_follow_controller_node.cpp
src/servo_control_pkg/src/servo_manager_node.cpp
src/servo_control_pkg/src/ibvs_controller.cpp
src/servo_control_pkg/src/pbvs_controller.cpp
src/servo_control_pkg/src/control_allocator.cpp
src/servo_control_pkg/src/velocity_commander_node.cpp
src/vision_servo_msgs/msg/ServoState.msg
src/vision_servo_msgs/srv/SetServoMode.srv
src/vision_servo_msgs/action/VisualServo.action
```

D 组给 vibe coder 的阅读提示词：

```text
我是 D 组，负责视觉伺服与协同控制。
请只阅读 servo_control_pkg、ServoState/SetServoMode/VisualServo 接口，以及与控制输入输出相关的消息。
请整理：
1. MVP 跟拍、IBVS、PBVS、ControlAllocator、ServoManager 的关系；
2. 每个控制器需要的输入字段；
3. /perception/targets_3d、/platform/state、/camera/camera_info 如何进入控制闭环；
4. /cmd_vel、/cmd_gimbal、/servo/state 如何发布；
5. MPC/RL 当前是实现完成、框架完成还是占位。
不要修改代码。
```

### 7.3 当前完成度判断

| 模块 | 当前状态 | 说明 |
| --- | --- | --- |
| MvpFollowControllerNode | 已完成/可运行 | 可用 mock target 和 2D 仿真验证，包含滤波、死区、限幅 |
| IBVSController | 已完成/可运行 | 使用交互矩阵和阻尼伪逆计算相机速度 |
| PBVSController | 已完成/可运行 | 基于 3D 目标位置计算相机速度 |
| ControlAllocator | 已完成/可运行 | 将相机速度分配为底盘 Twist 和云台 yaw/pitch rate |
| ServoManagerNode | 已完成/可运行 | 50Hz 控制循环，支持插件加载、`SetServoMode` 和 `VisualServo` |
| VelocityCommanderNode | 框架完成 | 当前主要处理线速度，角速度/云台通道仍有 TODO |
| MPCController | 待补齐 | 当前是头文件和接口设想，未接 QP 求解器 |
| RLController | 待补齐 | 当前是头文件和接口设想，未接策略模型 |

### 7.4 D 组最小运行验证

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to servo_control_pkg simulation_pkg robot_platform_pkg bringup_pkg
source install/setup.bash
```

MVP mock 验证：

```bash
ros2 launch servo_control_pkg mvp_mock_test.launch.py scenario:=left
```

另开终端观察输出：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_gimbal
```

2D 闭环仿真：

```bash
ros2 launch servo_control_pkg mvp_2d_sim.launch.py rviz:=true target_motion:=circle
```

完整仿真控制链：

```bash
ros2 launch bringup_pkg fcr_sim_bringup.launch.py controller_plugin:=servo_control_pkg::PBVSController
```

模式切换服务：

```bash
ros2 service call /servo/set_mode vision_servo_msgs/srv/SetServoMode "{mode: 1}"
```

模式枚举：

```text
0 = IBVS
1 = PBVS
2 = HYBRID
3 = MPC
4 = RL
```

注意：当前 2、3、4 主要还是扩展入口，不代表 HYBRID/MPC/RL 已完整实现。

### 7.5 D 组与其他组接口

| 对接方 | D 组提供 | D 组依赖 |
| --- | --- | --- |
| C 组 | 对 `/perception/targets_3d` 字段和 frame 的需求 | C 组提供稳定 target、depth、bbox 和 tracking_id |
| B 组 | `/cmd_vel`、`/cmd_gimbal` 命令 | B 组提供 `/platform/state` 和执行器约束 |
| E 组 | `SetServoMode`、`VisualServo`、`ServoState` | E 组提供任务触发、模式切换和上层控制入口 |

D 组适合线上开发，但参数整定和真实效果需要 C/B 组的数据或仿真环境配合。

## 8. E 组：任务运镜与人机交互

### 8.1 负责范围

E 组负责把“用户想做什么”转成 ROS 2 Service 或 Action，包括跟拍开始/停止、目标选择、模式切换、运镜任务和可视化操作。当前主要对应：

```text
任务 8：任务规划与运镜 Action
任务 9：人机交互与上层指令
部分任务 1：bringup、接口调用、可视化入口
```

核心目标不是直接控制电机，而是通过已有接口调用 C/D/B 组能力。

### 8.2 先读哪些文件

```text
src/vision_servo_msgs/action/VisualServo.action
src/vision_servo_msgs/srv/SetServoMode.srv
src/vision_servo_msgs/srv/SetTrackingTarget.srv
src/vision_servo_msgs/msg/ServoState.msg
src/vision_servo_msgs/msg/PlatformState.msg
src/servo_control_pkg/src/servo_manager_node.cpp
src/bringup_pkg/launch/fcr_bringup.launch.py
src/bringup_pkg/launch/fcr_sim_bringup.launch.py
src/bringup_pkg/launch/fcr_debug.launch.py
src/bringup_pkg/rviz/fcr_system.rviz
docs/project_documentation.md
```

E 组给 vibe coder 的阅读提示词：

```text
我是 E 组，负责任务运镜与人机交互。
请只阅读 vision_servo_msgs 的 action/srv、servo_manager_node 中 action/service 相关代码，以及 bringup_pkg。
请整理：
1. 当前有哪些 service/action 可以给上层调用；
2. VisualServo.action 的 goal/result/feedback 各字段含义；
3. SetServoMode 和 SetTrackingTarget 如何用于 UI/语音/遥控；
4. 当前已有的 bringup 和 RViz/Foxglove 入口；
5. 哪些运镜功能还没有实现，需要新增什么节点或 action。
不要修改代码。
```

### 8.3 当前完成度判断

| 模块 | 当前状态 | 说明 |
| --- | --- | --- |
| `VisualServo.action` 接口 | 已完成/可运行 | 能表达长周期伺服任务、超时、误差容限、反馈和取消 |
| ServoManager Action Server | 已完成/可运行 | `/servo/visual_servo` 已在 `servo_manager_node` 中实现 |
| `SetServoMode` | 已完成/可运行 | 可切换 IBVS/PBVS 等模式，MPC/RL 目前仍是占位映射 |
| `SetTrackingTarget` | 已完成/可运行 | 可按 ID 或类别选择/停止跟踪目标 |
| bringup 启动入口 | 已完成/可运行 | 有生产、仿真、调试启动文件 |
| RViz/Foxglove 入口 | 框架完成 | RViz 配置存在，Foxglove bridge 在 launch 中可选 |
| 运镜轨迹节点 | 待补齐 | 目前还没有推近、拉远、环绕、侧向跟拍的轨迹生成器 |
| Web/语音/遥控交互 | 待补齐 | 当前主要通过 ROS 2 CLI、launch 和 RViz 调试 |

### 8.4 E 组最小运行验证

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to servo_control_pkg robot_platform_pkg simulation_pkg bringup_pkg
source install/setup.bash
```

查看接口：

```bash
ros2 interface show vision_servo_msgs/action/VisualServo
ros2 interface show vision_servo_msgs/srv/SetServoMode
ros2 interface show vision_servo_msgs/srv/SetTrackingTarget
```

启动仿真控制链：

```bash
ros2 launch bringup_pkg fcr_sim_bringup.launch.py
```

另开终端查看 action/service：

```bash
ros2 action list
ros2 action info /servo/visual_servo
ros2 service list | grep servo
ros2 service list | grep tracking
```

发送模式切换：

```bash
ros2 service call /servo/set_mode vision_servo_msgs/srv/SetServoMode "{mode: 1}"
```

发送 VisualServo goal：

```bash
ros2 action send_goal /servo/visual_servo vision_servo_msgs/action/VisualServo \
"{target_id: -1, class_name: 'person', desired_depth: 2.0, feature_tolerance: 0.05, timeout: {sec: 30, nanosec: 0}}"
```

### 8.5 E 组后续实现路线

E 组可以先不做完整 Web 或语音，先做一个“任务命令节点”：

```text
command_node
  -> 调用 /tracking/set_tracking_target 或 /perception_pipeline/set_tracking_target
  -> 调用 /servo/set_mode
  -> 发送 /servo/visual_servo action goal
  -> 订阅 /servo/state 和 /platform/state
  -> 输出当前任务状态
```

再扩展运镜：

```text
Cinematic command
  -> 轨迹生成器
  -> 动态 desired_depth / desired framing
  -> VisualServo 或后续 CinematicShot action
```

建议先做这些最小任务：

```text
1. start_follow：选择目标并启动 VisualServo
2. stop_follow：取消 VisualServo
3. switch_mode：切换 IBVS/PBVS
4. recenter_gimbal：调用或预留云台回中入口
5. shot_push_in / shot_pull_out：先用 desired_depth 变化模拟推近/拉远
```

E 组非常适合线上同学做，因为主要工作是接口调用、任务状态机、UI/语音原型和文档化。

## 9. 四组协作边界

### 9.1 不要随便改公共接口

这些文件属于全组共享契约：

```text
src/vision_servo_msgs/msg/Target.msg
src/vision_servo_msgs/msg/TargetArray.msg
src/vision_servo_msgs/msg/ServoState.msg
src/vision_servo_msgs/msg/PlatformState.msg
src/vision_servo_msgs/msg/GimbalCmd.msg
src/vision_servo_msgs/srv/SetServoMode.srv
src/vision_servo_msgs/srv/SetTrackingTarget.srv
src/vision_servo_msgs/action/VisualServo.action
```

改这些文件前必须先同步，因为改接口会导致下游包重新生成、编译和适配。

### 9.2 建议固定的接口契约

| 数据 | 生产者 | 消费者 |
| --- | --- | --- |
| `/camera/image_raw` | 相机驱动/仿真 | C 组 Detection/Pipeline |
| `/camera/depth/image_raw` | 深度相机/仿真 | C 组 DepthEstimator |
| `/camera/camera_info` | 相机驱动/仿真 | C 组、D 组 |
| `/perception/detections` | C 组 Detection | C 组 Tracking/Depth |
| `/perception/tracks` | C 组 Tracking | 调试/可视化 |
| `/perception/targets_3d` | C 组 Depth/Pipeline | D 组 ServoManager |
| `/platform/state` | B 组 PlatformManager | D 组、E 组 |
| `/cmd_vel` | D 组 Servo/MVP | B 组 Chassis/Odometry |
| `/cmd_gimbal` | D 组 Servo/MVP | B 组 Gimbal |
| `/servo/state` | D 组 ServoManager | E 组、可视化 |
| `/servo/set_mode` | D 组 ServoManager | E 组 UI/语音/CLI |
| `/servo/visual_servo` | D 组 ServoManager | E 组任务层 |
| `/tracking/set_tracking_target` 或 `/perception_pipeline/set_tracking_target` | C 组 Tracking/Pipeline | E 组目标选择 |

### 9.3 线上/线下分工建议

| 工作 | 是否适合线上 | 说明 |
| --- | --- | --- |
| C 组感知代码阅读、YOLO 接入、跟踪调试 | 适合 | 可用视频、图片、mock、rosbag 或离线数据 |
| D 组控制器、仿真、MPC/QP 原型 | 适合 | mock 和 2D/Gazebo 仿真足够支撑大部分开发 |
| E 组 Action、CLI/UI/语音原型、文档 | 适合 | 基本不依赖真实硬件 |
| B 组平台抽象、仿真后端、状态聚合 | 部分适合 | 接口和仿真可线上做，真实硬件协议/实机联调需要线下 |
| 真实底盘、RS2、IMU、相机联调 | 不适合纯线上 | 必须有设备、线缆、电源、安全空间和现场观察 |

## 10. 每位同学的第一天任务清单

每位同学第一天不建议直接写新功能，先完成这些：

```text
1. 成功 clone 仓库并完成 build。
2. 阅读 README.md 和 docs/project_documentation.md。
3. 用 vibe coder 生成自己负责模块的代码地图。
4. 跑通自己组的最小 launch 或接口查看命令。
5. 写一个本组完成度清单：已完成、框架完成、待补齐。
6. 在 GitHub issue 或群里同步自己准备做的第一个小目标。
```

每日同步模板：

```text
我负责：B/C/D/E 组 - <具体模块>

今天读了：
- 文件/包：

当前完成度判断：
- 已完成：
- 框架完成：
- 待补齐：

我准备改：
- 小目标：
- 影响文件：
- 是否涉及公共接口：是/否

验证方式：
- build:
- launch:
- topic/service/action:

需要其他组提供：
- ...
```

## 11. 推荐的 vibe coder 提问模板

### 11.1 代码地图模板

```text
请为我负责的 <package/module> 生成代码地图。
输出格式：
1. 文件树；
2. 节点列表；
3. 每个节点的输入/输出；
4. 参数文件；
5. launch 入口；
6. 当前完成度；
7. 最小验证命令。
要求：只基于仓库已有代码，不要猜测未实现内容。
```

### 11.2 完成度审查模板

```text
请审查 <模块名> 在当前代码中的完成度。
请按“已完成/框架完成/待补齐”分类。
每一项必须引用具体文件路径和关键函数/类名。
不要提出大范围重构建议，重点说明我接下来最应该补哪 1-2 个小功能。
```

### 11.3 修改计划模板

```text
我想实现 <小功能>。
请先只写修改计划，不要改代码。
计划需要包括：
1. 修改哪些文件；
2. 新增哪些参数或接口；
3. 是否影响其他组；
4. 如何构建；
5. 如何运行验证；
6. 失败时如何回退。
```

### 11.4 修改后自检模板

```text
请检查你刚才的改动：
1. 是否引入了新的 package 依赖；
2. package.xml 和 CMakeLists.txt 是否同步；
3. topic/service/action 名称是否变化；
4. 是否影响 B/C/D/E 其他组；
5. 给出我应该运行的 build 和 ros2 验证命令。
```

## 12. 最重要的原则

1. 先读懂数据流，再写代码。
2. 先跑最小验证，再做复杂联调。
3. 先在 mock/仿真中验证，再上真实硬件。
4. 先问清楚接口影响，再改公共消息。
5. 每次只让 vibe coder 解决一个清晰的小问题。
6. AI 的结论必须能落到文件路径、函数、topic、service、action 和运行命令上。
