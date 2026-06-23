# FCR ROS2 新人快速上手指南

更新时间：2026-06-22

这份文档给刚加入项目组的同学使用。目标是让新人先在脑子里建立项目地图，再尽快跑通一个可观察的闭环，最后知道自己应该从哪里开始贡献。

## 1. 项目一句话

本项目是一个基于 ROS2 Humble 的智能跟拍机器人系统，当前代码重点是视觉伺服跟随闭环，最终目标是扩展成基于 TRON2 底盘、DJI RS2 云台和 Sony ZV-E10 II 相机的智能 Vlog 跟拍机器人。

最终系统要实现：

```text
Sony 图像
  -> YOLO 检测
  -> 多目标跟踪
  -> 目标状态估计
  -> IBVS/PBVS/MPC 控制
  -> TRON2 底盘 + DJI RS2 云台协同运动
```

一句更直观的话：不是简单“人在哪机器人去哪”，而是让机器人在遥控、自主跟随、语音控制和预设运镜之间切换，拍出稳定、平滑、有构图意识的 Vlog 画面。

## 2. 当前进度

当前项目大致处在 V2.5：

| 阶段 | 状态 | 说明 |
|---|---|---|
| V1 MVP | 基本完成 | mock target 或 2D 仿真能跑通 `/target/current -> /cmd_vel + /cmd_gimbal` |
| V2 稳定化 | 大部分完成 | 已有滤波、死区、限幅、目标丢失保护和 MVP 跟随控制 |
| V3 混合视觉伺服 | 算法已有，架构待拆 | IBVS、PBVS、ControlAllocator 已有，后续要拆成更清晰节点 |
| V4 MPC 优化 | 刚起步 | `MPCController` 目前主要是头文件骨架 |

几个重要事实：

- `DetectionNode` 的 YOLO 模型加载和 `infer()` 仍是占位实现。
- 真实底盘、真实 DJI RS2、真实 IMU 驱动还没有补齐，目前主要是仿真后端。
- 现有代码里平台包仍保留 LEKIWI/三轮全向底盘抽象；最终硬件目标会迁移/适配到 TRON2。
- 最可靠的入门方式是先跑 MVP mock 和 2D 仿真，不要一上来接真实硬件。

## 3. 仓库地图

```text
fcr_ros2_3/
├── README.md                         # 项目总览，当前状态最权威入口
├── build.sh                          # ROS2 Humble 构建脚本
├── doc/
│   └── build_failure_report.md        # 曾经的构建问题和修复记录
├── docs/
│   ├── v4_landing_task_breakdown.md   # 当前任务拆分和里程碑
│   ├── newcomer_onboarding.md         # 本文档
│   └── uml/                           # 包图、类图、序列图、状态机等 PlantUML
└── src/
    ├── vision_servo_msgs/             # msg/srv/action 接口定义
    ├── perception_pkg/                # 检测、跟踪、深度估计
    ├── servo_control_pkg/             # MVP、IBVS、PBVS、控制分配、MPC/RL 骨架
    ├── robot_platform_pkg/            # 底盘、云台、IMU、里程计、平台状态聚合
    ├── simulation_pkg/                # Gazebo、URDF、mock target、2D 仿真
    └── bringup_pkg/                   # 一键启动、RViz 配置、调试 launch
```

## 4. 先跑起来：30 分钟路线

推荐环境是 Ubuntu 22.04 + ROS2 Humble。第一次构建如果依赖已经装好，可以用 `--skip-rosdep`；如果缺依赖，先不用这个参数，让脚本跑 `rosdep install`。

```bash
cd ~/fcr_ros2/fcr_ros2_3
source /opt/ros/humble/setup.bash
./build.sh
source install/setup.bash
```

如果 `rosdep` 或网络环境暂时不方便：

```bash
./build.sh --skip-rosdep
source install/setup.bash
```

### 路线 A：最轻量 mock 测试

这个不依赖 Gazebo，也不依赖真实相机/硬件。适合确认控制器能否正常输出命令。

```bash
ros2 launch servo_control_pkg mvp_mock_test.launch.py scenario:=left
```

另开终端检查输出：

```bash
source install/setup.bash
ros2 topic echo /target/current
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_gimbal
```

可试的 `scenario`：

```text
center, left, right, up, down, far, near, lost, sinusoidal
```

### 路线 B：2D 闭环仿真

这个会让控制输出反过来影响仿真机器人位置，比 mock 更接近真实闭环。

```bash
ros2 launch servo_control_pkg mvp_2d_sim.launch.py target_motion:=circle rviz:=true
```

可试的目标运动：

```text
static, circle, line, figure8
```

重点观察：

- `/target/current` 是否连续输出目标。
- `/platform/state` 是否更新平台/云台状态。
- `/cmd_vel` 是否随目标距离和云台偏角变化。
- `/cmd_gimbal` 是否随图像误差变化。

### 路线 C：完整 Gazebo 仿真

这个启动 Gazebo、机器人模型、目标轨迹、伺服管理器和仿真云台驱动。

```bash
ros2 launch bringup_pkg fcr_sim_bringup.launch.py trajectory:=circle speed:=0.3
```

常用参数：

```text
controller_plugin:=servo_control_pkg::PBVSController
controller_plugin:=servo_control_pkg::IBVSController
trajectory:=circle
trajectory:=line
trajectory:=figure8
allocation_ratio:=0.5
```

## 5. 先理解这条数据流

完整系统的数据流可以先记成下面这条主线：

```text
/camera/image_raw
  -> detection_node
  -> /perception/detections
  -> tracking_node
  -> /perception/tracks
  -> depth_estimator_node
  -> /perception/targets_3d
  -> servo_manager_node 或 mvp_follow_controller_node
  -> /cmd_vel + /cmd_gimbal
  -> chassis_driver_node + gimbal_driver_node
```

MVP 路径更短：

```text
mock_target_publisher_node 或 simple_robot_sim_node
  -> /target/current
  -> mvp_follow_controller_node
  -> /cmd_vel + /cmd_gimbal
```

读代码时先抓这几个入口：

| 目标 | 文件 |
|---|---|
| MVP 跟拍控制主逻辑 | `src/servo_control_pkg/src/mvp_follow_controller_node.cpp` |
| IBVS 控制器 | `src/servo_control_pkg/src/ibvs_controller.cpp` |
| PBVS 控制器 | `src/servo_control_pkg/src/pbvs_controller.cpp` |
| 控制分配 | `src/servo_control_pkg/src/control_allocator.cpp` |
| YOLO 检测占位 | `src/perception_pkg/src/detection_node.cpp` |
| 跟踪器 | `src/perception_pkg/src/multi_object_tracker.cpp` |
| 深度估计 | `src/perception_pkg/src/depth_estimator_node.cpp` |
| 底盘驱动节点 | `src/robot_platform_pkg/src/chassis_driver_node.cpp` |
| 云台驱动节点 | `src/robot_platform_pkg/src/gimbal_driver_node.cpp` |
| 平台状态聚合 | `src/robot_platform_pkg/src/platform_manager_node.cpp` |

## 6. 核心接口速查

| 接口 | 类型 | 作用 |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | 相机原始图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 相机内参，建议 TRANSIENT_LOCAL |
| `/perception/detections` | `vision_servo_msgs/TargetArray` | 2D 检测结果 |
| `/perception/tracks` | `vision_servo_msgs/TargetArray` | 带 track_id 的目标 |
| `/perception/targets_3d` | `vision_servo_msgs/TargetArray` | 带 3D 位置/距离的目标 |
| `/target/current` | `vision_servo_msgs/TargetArray` | MVP 控制器当前目标输入 |
| `/platform/state` | `vision_servo_msgs/PlatformState` | 底盘、云台、IMU 聚合状态 |
| `/cmd_vel` | `geometry_msgs/TwistStamped` 或 `Twist` | 底盘速度命令 |
| `/cmd_gimbal` | `vision_servo_msgs/GimbalCmd` | 云台 yaw/pitch 速度命令 |
| `/servo/state` | `vision_servo_msgs/ServoState` | 伺服控制状态和误差 |
| `/servo/set_mode` | `SetServoMode.srv` | IBVS/PBVS/HYBRID/MPC/RL 模式切换 |
| `/servo/visual_servo` | `VisualServo.action` | 长时间视觉伺服任务 |

模式编号：

```text
0 = IBVS
1 = PBVS
2 = HYBRID
3 = MPC
4 = RL
```

切换示例：

```bash
ros2 service call /servo/set_mode vision_servo_msgs/srv/SetServoMode "{mode: 1}"
```

## 7. 新人第一天应该做什么

建议按这个顺序，不要跳：

1. 读 `README.md` 的项目状态、包结构、数据流。
2. 跑通 `mvp_mock_test.launch.py`，确认 `/cmd_vel` 和 `/cmd_gimbal` 有输出。
3. 跑通 `mvp_2d_sim.launch.py`，观察目标运动和控制闭环。
4. 打开 `mvp_follow_controller_node.cpp`，理解三条控制通道：

```text
图像水平误差 ex -> 云台 yaw
图像竖直误差 ey -> 云台 pitch
距离误差 ez -> 底盘 vx
云台 yaw 偏角 q_yaw -> 底盘 wz
```

5. 从 `docs/v4_landing_task_breakdown.md` 里认领一个 P0/P1 任务。

## 8. 按角色快速上手

| 方向 | 先读 | 先跑 | 适合认领的任务 |
|---|---|---|---|
| 感知 | `detection_node.cpp`, `tracking_node.cpp`, `depth_estimator_node.cpp` | `fcr_bringup.launch.py use_mock_detector:=true` | 补 YOLO 推理、真实相机、目标选择 |
| 控制 | `mvp_follow_controller_node.cpp`, `ibvs_controller.cpp`, `pbvs_controller.cpp` | `mvp_2d_sim.launch.py` | 参数调优、丢失处理、速度变化率限制 |
| 硬件 | `chassis_driver_node.cpp`, `gimbal_driver_node.cpp`, hardware interface headers | `platform.launch.py use_sim:=true` | TRON2 适配、RS2 状态反馈、急停 |
| 仿真 | `simulation_pkg/scripts/*`, `simulation_pkg/urdf/*` | `fcr_sim_bringup.launch.py` | 轨迹扩展、Marker 可视化、URDF 对齐 |
| 系统集成 | `bringup_pkg/launch/*`, `vision_servo_msgs/*` | 三条启动路线都跑 | launch 测试清单、Topic 图、参数表 |
| 语音/交互 | 任务表 I1-I8 | 先用文本 topic 模拟 intent | `voice_control_pkg`、intent parser、安全策略 |
| MPC/优化 | `mpc_controller.hpp`, `control_allocator.cpp` | 先做离线或 2D 仿真 | QP formulation、MPC 最小原型、实验对比 |

## 9. 常用调试命令

看节点和话题：

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

看类型：

```bash
ros2 topic info /cmd_vel -v
ros2 interface show vision_servo_msgs/msg/Target
ros2 interface show vision_servo_msgs/msg/PlatformState
```

看数据：

```bash
ros2 topic echo /target/current
ros2 topic echo /platform/state
ros2 topic echo /servo/state
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_gimbal
```

看 TF：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_optical_link
```

记录实验：

```bash
ros2 bag record /target/current /platform/state /servo/state /cmd_vel /cmd_gimbal
```

## 10. 改代码前的安全约定

- 先在 mock 或 2D 仿真里验证，再接 Gazebo，最后才碰真实硬件。
- 任何真实底盘测试都从低速开始，旁边必须有人能急停。
- 控制节点退出、目标丢失、通信超时，都必须让底盘和云台进入安全停止。
- 新增 topic/service/action 时优先放到 `vision_servo_msgs`，不要在业务包里随手建接口。
- 控制相关 topic 要显式设置 QoS，命令类 topic 建议 `RELIABLE + KEEP_LAST`。
- 不要在 ROS2 callback 里写长时间阻塞逻辑，尤其是服务调用、文件 I/O、sleep。

## 11. 当前最值得做的上手任务

| 优先级 | 任务 | 为什么适合新人 |
|---|---|---|
| P0 | 写 `docs/runbook_current.md` | 能逼自己跑通所有 launch，顺便补齐项目知识 |
| P0 | 写 `docs/system_graph.md` | 能快速理解节点、topic、service、TF |
| P0 | 整理 `docs/parameter_reference.md` | 能理解控制器和仿真参数如何影响效果 |
| P0 | 增强 2D 仿真场景 | 风险低，反馈快，能帮助控制调参 |
| P1 | YOLO 推理补全 | 感知闭环的关键缺口 |
| P1 | `/target/current` 统一出口 | 让控制器不依赖感知内部细节 |
| P1 | 底盘/云台安全停车测试 | 真实硬件前必须补齐 |
| P1 | 语音 intent 文档和文本模拟节点 | 不依赖硬件，能推进上层交互 |

## 12. 常见困惑

**为什么 README 写 LEKIWI，但最终目标是 TRON2？**

当前代码平台层最初按 LEKIWI/三轮全向抽象搭建，很多上层接口已经和具体底盘解耦。后续 TRON2 适配应尽量保持 `/cmd_vel`、`/odom`、`base_link` 等上层接口不变。

**为什么不直接上 MPC？**

MPC 依赖稳定目标状态、可靠平台状态、真实速度/角度反馈和安全约束。当前更重要的是先把真实感知、硬件闭环、安全层和 V3 控制结构打稳。

**为什么底盘不直接追图像水平误差？**

现有 MVP 设计是云台快速修正画面，底盘慢速追云台 yaw 偏角。这样画面更稳，底盘不会因为检测框抖动频繁左右扭。

**我应该先看 UML 还是代码？**

先看 `README.md` 和本文档，再跑 2D 仿真。跑起来之后再看 `docs/uml/`，效果会好很多。

## 13. 推荐阅读顺序

```text
README.md
  -> docs/newcomer_onboarding.md
  -> docs/v4_landing_task_breakdown.md
  -> docs/uml/README.md
  -> 对应方向的源码
```

新人入组的第一个验收目标：

```text
能独立构建项目，
能跑通 MVP mock 和 2D 仿真，
能说清楚 /target/current -> /cmd_vel + /cmd_gimbal 的数据链，
能从任务表中认领一个 P0/P1 小任务。
```
