# FCR ROS2 Visual Servoing Workspace (fcr_ros2_3)

基于 ROS2 Humble 的工业级视觉伺服移动机器人项目框架。
LEKIWI 全向底盘 + 云台 + YOLO 目标检测 + IBVS/PBVS 视觉伺服跟踪。

---

## 1. 完整目录树

```
fcr_ros2_3/
├── README.md                          # 本文档
├── build.sh                           # 一键构建脚本
├── .gitignore
├── .vscode/
│   └── settings.json
└── src/
    ├── vision_servo_msgs/             # ★ 消息/服务/动作 独立定义包
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── msg/
    │   │   ├── Target.msg             # 目标检测/跟踪结果（2D+3D）
    │   │   ├── TargetArray.msg        # 多目标数组
    │   │   ├── ServoState.msg         # 视觉伺服状态（误差、雅可比、收敛）
    │   │   ├── PlatformState.msg      # 统一平台状态（底盘+云台+IMU）
    │   │   └── GimbalCmd.msg          # 云台速度指令
    │   ├── srv/
    │   │   ├── SetTrackingTarget.srv   # 设置跟踪目标
    │   │   ├── SetServoMode.srv        # 运行时切换伺服模式
    │   │   └── CalibrateCamera.srv     # 触发相机标定
    │   └── action/
    │       └── VisualServo.action      # 长时间伺服任务（反馈+取消）
    │
    ├── perception_pkg/                # ★ 视觉节点统一包
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/perception_pkg/
    │   │   ├── detection_node.hpp      # YOLO 检测节点
    │   │   ├── tracking_node.hpp       # SORT/ByteTrack 多目标跟踪
    │   │   ├── depth_estimator.hpp     # 深度估计（RGBD/双目）
    │   │   ├── perception_pipeline.hpp # 可组合流水线（intra-process）
    │   │   └── qos.hpp
    │   ├── src/
    │   │   ├── detection_node.cpp
    │   │   ├── tracking_node.cpp
    │   │   ├── depth_estimator_node.cpp
    │   │   └── perception_pipeline.cpp
    │   ├── config/
    │   │   ├── detection_params.yaml
    │   │   ├── tracking_params.yaml
    │   │   └── depth_params.yaml
    │   └── launch/
    │       └── perception.launch.py
    │
    ├── servo_control_pkg/             # ★ 视觉伺服+控制算法统一包
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── plugins.xml                 # pluginlib 控制器注册
    │   ├── include/servo_control_pkg/
    │   │   ├── servo_controller_base.hpp # ★ 抽象基类（所有控制器继承）
    │   │   ├── ibvs_controller.hpp       # IBVS 图像雅可比控制器
    │   │   ├── pbvs_controller.hpp       # PBVS 位姿误差控制器
    │   │   ├── mpc_controller.hpp        # MPC 模型预测控制（论文扩展）
    │   │   ├── rl_controller.hpp         # RL 强化学习控制（论文扩展）
    │   │   ├── control_allocator.hpp     # 控制分配：相机速度→底盘+云台
    │   │   └── qos.hpp
    │   ├── src/
    │   │   ├── servo_controller_base.cpp
    │   │   ├── ibvs_controller.cpp
    │   │   ├── pbvs_controller.cpp
    │   │   ├── control_allocator.cpp
    │   │   ├── servo_manager_node.cpp    # 主控节点：加载插件、启停伺服
    │   │   └── velocity_commander_node.cpp
    │   ├── config/
    │   │   ├── ibvs_params.yaml
    │   │   ├── pbvs_params.yaml
    │   │   └── allocator_params.yaml
    │   └── launch/
    │       └── servo_control.launch.py
    │
    ├── robot_platform_pkg/            # ★ 硬件接口统一包
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/robot_platform_pkg/
    │   │   ├── hardware_interfaces/
    │   │   │   ├── chassis_interface.hpp    # ★ 抽象底盘接口
    │   │   │   ├── gimbal_interface.hpp     # ★ 抽象云台接口
    │   │   │   ├── imu_interface.hpp        # ★ 抽象IMU接口
    │   │   │   └── odometry_interface.hpp   # ★ 抽象里程计接口
    │   │   ├── kinematics/
    │   │   │   └── three_wheel_omni_kinematics.hpp  # LEKIWI 三轮全向
    │   │   └── utils/
    │   │       ├── serial_utils.hpp
    │   │       ├── can_utils.hpp            # DJI RS2 CAN协议
    │   │       └── math_utils.hpp
    │   ├── src/
    │   │   ├── chassis_driver_node.cpp
    │   │   ├── gimbal_driver_node.cpp
    │   │   ├── imu_driver_node.cpp
    │   │   ├── odometry_node.cpp
    │   │   └── platform_manager_node.cpp    # 统一平台状态聚合
    │   ├── config/
    │   │   ├── chassis_params.yaml
    │   │   ├── gimbal_params.yaml
    │   │   ├── imu_params.yaml
    │   │   └── odometry_params.yaml
    │   └── launch/
    │       └── platform.launch.py
    │
    ├── bringup_pkg/                   # ★ 一键启动包
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/
    │   │   └── global_params.yaml
    │   ├── launch/
    │   │   ├── fcr_bringup.launch.py      # 生产模式一键启动
    │   │   ├── fcr_sim_bringup.launch.py  # 仿真模式一键启动
    │   │   └── fcr_debug.launch.py        # 调试模式（逐个节点、verbose log）
    │   └── rviz/
    │       └── fcr_system.rviz
    │
    └── simulation_pkg/                # ★ 仿真包
        ├── CMakeLists.txt
        ├── package.xml
        ├── urdf/
        │   ├── lekiwi_base.xacro          # 底盘（三轮全向+IMU）
        │   ├── lekiwi_gimbal.xacro        # 云台（yaw+pitch+相机光学帧）
        │   ├── lekiwi_sensors.xacro       # 深度相机+IMU Gazebo插件
        │   └── lekiwi_full.urdf.xacro     # 完整机器人（组合上述模块）
        ├── worlds/
        │   └── fcr_world.world            # Gazebo仿真世界（含目标物）
        ├── config/
        │   └── gazebo_params.yaml
        ├── launch/
        │   ├── gazebo.launch.py           # 启动Gazebo+spawn机器人
        │   ├── spawn_robot.launch.py      # 仅URDF发布+robot_state_publisher
        │   └── rviz.launch.py             # RViz2可视化
        └── scripts/
            ├── target_simulator.py        # 运动目标生成器
            └── camera_simulator.py        # 虚拟相机（测试模式）
```

---

## 2. 包依赖关系图

```
                    ┌──────────────────┐
                    │ vision_servo_msgs │  ← 消息/服务/动作 独立定义
                    └───────┬──────────┘
                            │ 被所有功能包依赖
          ┌─────────┬───────┼───────┬──────────┐
          ▼         ▼       ▼       ▼          ▼
  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────┐
  │perception│ │  servo   │ │  robot   │ │simulation │
  │   _pkg   │ │ _control │ │_platform │ │   _pkg    │
  │          │ │   _pkg   │ │   _pkg   │ │(URDF/Gaz) │
  └────┬─────┘ └────┬─────┘ └────┬─────┘ └─────┬─────┘
       │             │            │              │
       │  视觉结果    │  控制指令   │  状态反馈     │  URDF
       │  ────────▶  │  ────────▶ │              │
       │             │            │              │
       └─────────────┴────────────┴──────────────┘
                            │
                            ▼
                    ┌──────────────┐
                    │  bringup_pkg │  ← 聚合所有包，分层启动
                    └──────────────┘

依赖关系 (package.xml depend/exec_depend):

bringup_pkg ──exec──▶ perception_pkg, servo_control_pkg, robot_platform_pkg, simulation_pkg
perception_pkg ──depend──▶ vision_servo_msgs (+ cv_bridge, image_transport, OpenCV)
servo_control_pkg ──depend──▶ vision_servo_msgs (+ pluginlib, Eigen3)
robot_platform_pkg ──depend──▶ vision_servo_msgs (+ tf2_ros, nav_msgs, Eigen3)
simulation_pkg ──exec──▶ gazebo_ros_pkgs, xacro, robot_state_publisher
所有功能包 ──depend──▶ rclcpp, rclcpp_components
```

---

## 3. Topic / Message / Service / Action 设计

### 3.1 Topic 流图

```
┌─────────────┐    /camera/image_raw        ┌──────────────────┐
│  相机驱动    │ ──────────────────────────▶  │  Detection Node  │
│ (RealSense/ │    (sensor_msgs/Image)      │  (YOLO TensorRT)  │
│  OAK-D)     │                             └────────┬─────────┘
└─────────────┘                                      │
                                              /perception/detections
                                              (TargetArray)
                                                     │
                                                     ▼
                     ┌──────────────────────────────────────┐
                     │           Tracking Node               │
                     │       (SORT / ByteTrack)              │
                     └────────────┬─────────────────────────┘
                                  │
                    /perception/tracks (TargetArray, with persistent IDs)
                                  │
                                  ▼
                     ┌──────────────────────────────────────┐
                     │        Depth Estimator Node           │
                     │    (depth image → 3D coordinates)     │
                     └────────────┬─────────────────────────┘
                                  │
                    /perception/targets_3d (TargetArray, with 3D position)
                                  │
                                  ▼
  ┌────────────────┐    ┌──────────────────────────────┐
  │ Platform Mgr   │    │      Servo Manager Node       │
  │ /platform/state│◀───│  (pluginlib → IBVS/PBVS/MPC)  │
  │ (PlatformState)│    └──────────┬───────────────────┘
  └────────────────┘               │
                     ┌─────────────┴─────────────┐
                     │                           │
               /cmd_vel                   /cmd_gimbal
          (TwistStamped)                 (GimbalCmd)
                     │                           │
                     ▼                           ▼
          ┌──────────────┐           ┌──────────────┐
          │Chassis Driver│           │Gimbal Driver │
          │  /odom       │           │ /gimbal/state│
          └──────────────┘           └──────────────┘

监控/调试 Topics:
  /servo/state        (ServoState)    – 伺服状态、误差、雅可比条件数
  /target/marker      (Marker)        – RViz 3D目标标记
  /perception/detections_markers (MarkerArray) – 检测框可视化
```

### 3.2 Message 定义

| Message | 用途 | 关键字段 |
|---------|------|---------|
| `Target` | 单目标检测/跟踪结果 | id, class_name, bbox[4], center[2], confidence, position[3], velocity[3], depth_confidence |
| `TargetArray` | 多目标数组 | Header, Target[] targets, int32 tracking_id |
| `ServoState` | 伺服状态 | state (IDLE/CONVERGING/TRACKING/LOST/ERROR), feature_error[6], norm_error, condition_number, camera_velocity[6], gimbal_velocity[2], chassis_velocity[3] |
| `PlatformState` | 平台聚合状态 | chassis_pose[3], chassis_velocity[3], gimbal_yaw/pitch/yaw_rate/pitch_rate, angular_velocity[3], orientation, emergency_stop, system_mode |
| `GimbalCmd` | 云台速度指令 | yaw_rate, pitch_rate, hold_yaw, hold_pitch |

### 3.3 Service 定义

| Service | 用途 | Request | Response |
|---------|------|---------|----------|
| `SetTrackingTarget` | 选择跟踪目标 | target_id, class_name, enable | success, message, assigned_id |
| `SetServoMode` | 运行时切换伺服模式 | mode (IBVS=0, PBVS=1, HYBRID=2, MPC=3, RL=4) | success, message |
| `CalibrateCamera` | 触发标定 | calibration_type, target_pattern | success, message, calibration_file |

### 3.4 Action 定义

| Action | 用途 | Goal | Feedback | Result |
|--------|------|------|----------|--------|
| `VisualServo` | 长时间伺服任务 | target_id, class_name, desired_depth, feature_tolerance, timeout | progress, current_error, camera_velocity[6], servo_state | success, message, final_error, elapsed_time |

---

## 4. MultiThreadedExecutor 线程划分方案

```
┌─────────────────────────────────────────────────────────────┐
│                   MultiThreadedExecutor                      │
│                    (线程数 = CPU核心数)                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────────┐  ┌─────────────────────┐           │
│  │  Callback Group 1   │  │  Callback Group 2   │           │
│  │  "Sensor I/O"        │  │  "Perception"       │           │
│  │  (2 线程)            │  │  (2-4 线程)          │           │
│  │                      │  │                      │           │
│  │  • Camera driver     │  │  • YOLO 推理 (GPU)  │           │
│  │  • IMU driver (1kHz)│  │  • 目标跟踪          │           │
│  │  • 图像订阅回调       │  │  • 深度估计          │           │
│  │                      │  │  • 流水线组合节点     │           │
│  └─────────────────────┘  └─────────────────────┘           │
│                                                              │
│  ┌─────────────────────┐  ┌─────────────────────┐           │
│  │  Callback Group 3   │  │  Callback Group 4   │           │
│  │  "Servo Control"    │  │  "Hardware I/O"      │           │
│  │  (1 线程, 高优先级)  │  │  (2 线程)            │           │
│  │                      │  │                      │           │
│  │  • 50Hz 控制循环     │  │  • 底盘驱动 (串口)   │           │
│  │  • IBVS 雅可比计算   │  │  • 云台驱动 (CAN)    │           │
│  │  • 控制分配          │  │  • 里程计发布 (50Hz) │           │
│  │  • Servo Action Svr │  │  • TF 广播           │           │
│  └─────────────────────┘  └─────────────────────┘           │
│                                                              │
│  ┌─────────────────────┐                                     │
│  │  Callback Group 5   │                                     │
│  │  "Monitoring"       │                                     │
│  │  (1 线程)            │                                     │
│  │                      │                                     │
│  │  • /servo/state pub │                                     │
│  │  • /platform/state  │                                     │
│  │  • Service servers  │                                     │
│  │  • 参数回调          │                                     │
│  └─────────────────────┘                                     │
└─────────────────────────────────────────────────────────────┘

实现方式 (rclcpp):
  auto executor = rclcpp::executors::MultiThreadedExecutor(
    rclcpp::ExecutorOptions(), /*num_threads=*/8);
  
  // 将节点分配到不同的 Callback Group
  sensor_node->set_callback_group(sensor_group);     // Group 1
  perception_node->set_callback_group(perception_group); // Group 2
  servo_node->set_callback_group(control_group);      // Group 3
  driver_node->set_callback_group(hardware_group);    // Group 4
```

---

## 5. QoS 设计

| 数据类型 | Reliability | History | Depth | Durability | 理由 |
|---------|-------------|---------|-------|------------|------|
| 原始图像 `/camera/image_raw` | BEST_EFFORT | KEEP_LAST | 1 | VOLATILE | 容忍丢帧，始终取最新帧 |
| 深度图像 `/camera/depth/image_raw` | BEST_EFFORT | KEEP_LAST | 1 | VOLATILE | 同上 |
| 相机内参 `/camera/camera_info` | RELIABLE | KEEP_LAST | 1 | TRANSIENT_LOCAL | 静态数据，late joiner 需要 |
| 检测结果 `/perception/detections` | RELIABLE | KEEP_LAST | 5 | VOLATILE | 每帧都需要，小缓冲区 |
| 跟踪结果 `/perception/tracks` | RELIABLE | KEEP_LAST | 5 | VOLATILE | 同上 |
| 3D目标 `/perception/targets_3d` | RELIABLE | KEEP_LAST | 5 | VOLATILE | 控制输入，延迟敏感 |
| 底盘速度指令 `/cmd_vel` | RELIABLE | KEEP_LAST | 10 | VOLATILE | 必须可靠送达，低延迟 |
| 云台速度指令 `/cmd_gimbal` | RELIABLE | KEEP_LAST | 10 | VOLATILE | 同上 |
| IMU数据 `/imu/data` | BEST_EFFORT | KEEP_LAST | 5 | VOLATILE | 高频数据(100Hz)，容忍偶尔丢包 |
| 里程计 `/odom` | RELIABLE | KEEP_LAST | 10 | VOLATILE | 定位关键数据 |
| 平台状态 `/platform/state` | RELIABLE | KEEP_LAST | 10 | TRANSIENT_LOCAL | late joiner 获取当前状态 |
| 伺服状态 `/servo/state` | RELIABLE | KEEP_LAST | 5 | VOLATILE | 监控/调试用 |
| TF (`/tf`, `/tf_static`) | RELIABLE | KEEP_LAST | 100 | TRANSIENT_LOCAL(/tf_static) | ROS2 默认 |

---

## 6. 实机与仿真共用代码组织

```
实机与仿真共用策略（Factory Pattern + Launch Argument）：

┌─────────────────────────────────────────────────────┐
│                 Launch Argument                       │
│             use_sim := true / false                   │
└─────────────┬───────────────────┬───────────────────┘
              │                   │
      use_sim=true          use_sim=false
              │                   │
              ▼                   ▼
┌─────────────────┐   ┌─────────────────┐
│ SimulatedDriver  │   │  RealDriver     │
│ (内存模拟实现)    │   │  (硬件串口/CAN)  │
└────────┬────────┘   └────────┬────────┘
         │                     │
         └──────────┬──────────┘
                    │
                    ▼
         ┌───────────────────┐
         │ IChassisInterface │ ◀── 共用抽象接口
         │ IGimbalInterface  │      (定义在 robot_platform_pkg)
         │ IIMUInterface     │
         │ IOdometryInterface│
         └───────────────────┘
                    │
                    ▼
         ┌───────────────────┐
         │   上层算法节点      │
         │  (完全相同，无感知)  │
         │                    │
         │  • perception_pkg │
         │  • servo_control  │
         └───────────────────┘

具体实现：
  robot_platform_pkg/include/hardware_interfaces/
    chassis_interface.hpp   ← 纯虚接口
      ├── 实机: lekiwi_chassis_driver.cpp   (串口→电机指令)
      └── 仿真: simulated_chassis_driver.cpp (内存→Gazebo插件)

启动方式：
  # 实机
  ros2 launch bringup_pkg fcr_bringup.launch.py use_sim:=false

  # 仿真
  ros2 launch bringup_pkg fcr_sim_bringup.launch.py use_sim:=true
```

---

## 7. 可扩展架构（毕业设计 + 论文扩展）

```
可扩展控制器架构（pluginlib + 抽象基类）：

        ServoControllerBase (抽象基类)
        ┌─────────────────────────────┐
        │ + initialize(fx,fy,cx,cy)   │  ← 相机参数初始化
        │ + setDesiredFeatures(s*,d)  │  ← 设置期望特征
        │ + computeVelocity(Target,dt)│  ← ★ 核心接口（纯虚）
        │   → camera_velocity[6]      │
        │ + getServoState()           │  ← 状态查询
        │ + getControllerType()       │  ← 类型标识
        └──────────────┬──────────────┘
                       │
       ┌───────┬───────┼───────┬───────┐
       ▼       ▼       ▼       ▼       ▼
   ┌──────┐┌──────┐┌──────┐┌──────┐┌──────┐
   │ IBVS ││ PBVS ││Hybrid││ MPC  ││  RL  │
   │      ││      ││(2.5D)││      ││      │
   └──────┘└──────┘└──────┘└──────┘└──────┘
   已实现   已实现   占位    占位    占位

运行时切换：
  ros2 service call /servo/set_mode vision_servo_msgs/srv/SetServoMode "{mode: 1}"
  # 0=IBVS, 1=PBVS, 2=HYBRID, 3=MPC, 4=RL

添加新控制器的步骤（适用于论文扩展）：
  1. 继承 ServoControllerBase
  2. 实现 computeVelocity() 方法
  3. 在 plugins.xml 注册新类
  4. 在 CMakeLists.txt 添加编译目标
  5. 更新 SetServoMode 的 plugin_map 映射表
```

---

## 8. Launch 架构图

```
                        fcr_sim_bringup.launch.py
                        (一键仿真启动入口)
                               │
          ┌────────────────────┼────────────────────┐
          ▼                    ▼                    ▼
   gazebo.launch.py     rviz.launch.py     fcr_bringup.launch.py
   (Gazebo + spawn)     (RViz2可视化)      (use_sim:=true)
                                                    │
                         fcr_bringup.launch.py      │
                         (一键实机启动入口)           │
                                │                   │
          ┌─────────────────────┼───────────────────┘
          ▼                     ▼
   platform.launch.py   perception.launch.py   servo_control.launch.py
   ┌──────┬──────┬──────┐   ┌──────┬──────┐   ┌──────────┬──────────┐
   │chass │gimbal│ imu  │   │detect│ track│   │  servo   │velocity  │
   │driver│driver│driver│   │ node │ node │   │ manager  │commander │
   └──────┴──────┴──────┘   └──────┴──────┘   └──────────┴──────────┘
        │                              │              │
   odometry_node              depth_estimator   /cmd_vel → chassis
        │                              │         /cmd_gimbal → gimbal
   platform_manager                    │
        │                              │
        └──────────┬───────────────────┘
                   ▼
            /platform/state      /perception/targets_3d
                   │                      │
                   └──────────┬───────────┘
                              ▼
                       servo_manager
                     (IBVS/PBVS/MPC/RL)

调试模式（独立节点，verbose日志）：
  fcr_debug.launch.py → 七个节点逐个启动，--log-level debug
```

---

## 9. Node Graph（运行时节点图）

```
┌────────────┐   ┌──────────┐   ┌───────────┐
│camera_sim  │   │target_sim│   │ imu_driver│
│(sim only)  │   │(sim only)│   │           │
└─────┬──────┘   └─────┬────┘   └─────┬─────┘
      │image_raw       │/target/pose   │/imu/data
      ▼                │               ▼
┌──────────┐           │        ┌───────────┐
│detection │           │        │odometry   │
│  node    │           │        │  node     │
└────┬─────┘           │        └─────┬─────┘
     │/detections      │              │/odom
     ▼                 │              ▼
┌──────────┐           │        ┌───────────┐
│tracking  │           │        │platform   │
│  node    │           │        │ manager   │
└────┬─────┘           │        └─────┬─────┘
     │/tracks          │              │/platform/state
     ▼                 │              │
┌────────────┐         │              │
│depth_estim │         │              │
│   node     │         │              │
└─────┬──────┘         │              │
      │/targets_3d     │              │
      └────────┬───────┘              │
               ▼                      ▼
        ┌────────────────────────────────┐
        │       servo_manager            │
        │  (pluginlib → IBVS/PBVS/MPC)  │
        └──────────┬─────────────────────┘
                   │
         ┌─────────┴─────────┐
         ▼                   ▼
   ┌──────────┐       ┌──────────┐
   │ chassis  │       │ gimbal   │
   │ driver   │       │ driver   │
   └──────────┘       └──────────┘

━━━ 仿真专用节点 (-- use_sim:=true)
─── 实机/仿真共用节点
```

---

## 10. 快速开始

```bash
# 1. 安装依赖
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs
rosdep update
rosdep install --from-paths src --ignore-src -y

# 2. 构建
./build.sh

# 3. 启动（实机）
source install/setup.bash
ros2 launch bringup_pkg fcr_bringup.launch.py use_sim:=false

# 4. 启动（仿真）
ros2 launch bringup_pkg fcr_sim_bringup.launch.py

# 5. 调试模式
ros2 launch bringup_pkg fcr_debug.launch.py debug_level:=debug

# 6. 切换伺服模式
ros2 service call /servo/set_mode vision_servo_msgs/srv/SetServoMode "{mode: 1}"

# 7. 发送伺服目标
ros2 action send_goal /servo/visual_servo vision_servo_msgs/action/VisualServo \
  "{target_id: -1, desired_depth: 2.0, feature_tolerance: 0.01}"
```

---

## 版本演进

| 版本 | 工作空间 | 特点 |
|------|---------|------|
| v1 | fcr_ros2 | HSV颜色检测 + PID控制 + 抽象驱动 |
| v2 | fcr_ros2_1 | YOLO+ByteTrack + P控制 + DJI RS2 CAN |
| v3 | fcr_ros2_2 | PD+FF控制 + 仿真闭环 + Foxglove |
| **v4** | **fcr_ros2_3** | **工业级架构 + pluginlib可扩展控制器 + Gazebo仿真 + Action/Service + 论文扩展支持** |
