# vision_servo_msgs CMakeLists.txt
# 纯接口定义包 — 自定义消息、服务和动作的代码生成
#
# 该包不包含任何运行时节点，仅负责：
#   1. 定义视觉伺服系统所需的全部通信接口（.msg / .srv / .action）
#   2. 通过 rosidl_generate_interfaces 自动生成 C++ 和 Python 的类型代码
#   3. 其他功能包通过 package.xml 依赖此包获取类型定义
#
# C++17 要求：rosidl 代码生成器需要 C++17 特性（折叠表达式、结构化绑定）

cmake_minimum_required(VERSION 3.8)
project(vision_servo_msgs)

# 设置 C++17 标准（rosidl 生成代码的最低要求）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ament_cmake：ROS2 CMake 构建基础
# rosidl_default_generators：消息/服务/动作的代码生成宏
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces 的 DEPENDENCIES 包必须事先 find_package
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── 接口定义与代码生成 ──────────────────────────────────────────
# rosidl_generate_interfaces 宏会自动为每个 .msg / .srv / .action 生成：
#   - C++ 头文件：include/<pkg>/msg/<Name>.hpp
#   - Python 模块：<pkg>.msg.<Name>
# DEPENDENCIES 声明本包接口依赖的外部消息类型
rosidl_generate_interfaces(${PROJECT_NAME}
  # ── 消息定义 ──────────────────────────────────────────────────
  "msg/Target.msg"          # 视觉伺服跟踪目标（位姿 + 类别 + 置信度）
  "msg/TargetArray.msg"     # 多目标列表（用于同时跟踪多个物体）
  "msg/ServoState.msg"      # 伺服控制器内部状态（误差、收敛率、控制模式）
  "msg/PlatformState.msg"   # 机器人平台状态（底盘速度 + 云台角度）
  "msg/GimbalCmd.msg"       # 云台控制指令（yaw/pitch 角度或角速度）
  "msg/GimbalStatus.msg"    # 云台运行状态与 CAN/协议诊断信息
  "msg/PerceptionMonitor.msg" # 远程视觉观察层的结构化状态与性能指标
  # ── 服务定义 ──────────────────────────────────────────────────
  "srv/SetTrackingTarget.srv" # 设置/切换跟踪目标（请求：目标 ID，响应：成功/失败）
  "srv/SetServoMode.srv"      # 切换伺服控制模式（IBVS/PBVS/MPC/RL）
  "srv/CalibrateCamera.srv"   # 触发相机标定流程（内参 + 外参）
  # ── 动作定义 ──────────────────────────────────────────────────
  "action/VisualServo.action" # 视觉伺服长周期任务（可取消、可反馈进度）
  DEPENDENCIES
    std_msgs            # 基础消息类型（Header, String, Float32 等）
    geometry_msgs       # 几何消息类型（Pose, Twist, Vector3 等）
    sensor_msgs         # 传感器消息类型（Image, CameraInfo 等）
    action_msgs         # 动作基类（Goal, Result, Feedback）
    builtin_interfaces  # 内置时间类型（Duration, Time）
)

ament_package()
