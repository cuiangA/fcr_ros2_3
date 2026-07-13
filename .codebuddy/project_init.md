# FCR ROS2 项目初始化上下文（.codebuddy/project_init.md）

> 本文件由 AI 在审查 `docs/team_quick_start.md` 与整个仓库后生成，用于后续会话快速启动。
> 生成时间：2026-07-13。如仓库结构大幅变动，请更新本文件。

## 1. 项目定位

基于 ROS 2 Humble 的智能跟拍机器人项目：LEKIWI 三轮全向底盘 + DJI RS2 云台相机 + YOLO 目标检测 + IBVS/PBVS 视觉伺服。

当前阶段：**V2.5**。MVP 闭环、目标跟踪、深度估计、IBVS/PBVS、控制分配、仿真和平台抽象基本具备；真实 YOLO 推理、真实硬件协议、MPC/QP、完整运镜 Action 和上层交互仍待补齐。

主数据流：

```text
相机图像 / 深度图
  -> YOLO 检测（DetectionNode，推理占位）
  -> 多目标跟踪（MultiObjectTracker，已完成）
  -> 目标 3D 状态估计（DepthEstimator，已完成）
  -> MVP / IBVS / PBVS 视觉伺服控制
  -> ControlAllocator 控制分配
  -> /cmd_vel + /cmd_gimbal
  -> 底盘 + 云台执行（当前为仿真后端）
  -> /platform/state 反馈
```

## 2. 包结构（6 个）

| 包 | 职责 | 关键入口 |
| --- | --- | --- |
| `vision_servo_msgs` | 接口定义（5 msg / 3 srv / 1 action） | msg/, srv/, action/ |
| `perception_pkg` | 感知管线：检测→跟踪→深度估计 | launch/perception.launch.py, src/*.cpp |
| `servo_control_pkg` | 伺服控制：IBVS/PBVS/MPC/RL + MVP 跟拍 | launch/, plugins.xml, src/*.cpp |
| `robot_platform_pkg` | 硬件平台：底盘/云台/IMU/里程计 | launch/platform.launch.py |
| `simulation_pkg` | 仿真：Python 脚本 | scripts/coop_sim.py, scripts/servo_bag_eval_node.py |
| `bringup_pkg` | 一键启动、可视化 | launch/*.launch.py, rviz/fcr_system.rviz |

## 3. 接口契约（公共，修改需全组同步）

位于 `src/vision_servo_msgs/`：
- msg: `Target.msg`, `TargetArray.msg`, `ServoState.msg`, `PlatformState.msg`, `GimbalCmd.msg`
- srv: `SetTrackingTarget.srv`, `SetServoMode.srv`, `CalibrateCamera.srv`
- action: `VisualServo.action`

注意：`CalibrateCamera.srv` 真实存在，但 `team_quick_start.md` §9.1 的公共接口清单漏列，改它也需要全组同步。

## 4. 各组职责与完成度（已核对代码）

| 组 | 方向 | 已完成/可运行 | 框架完成 | 待补齐 |
| --- | --- | --- | --- | --- |
| B | 硬件平台与基础控制 | 平台 launch、三轮全向运动学、里程计、PlatformManager | 底盘/云台/IMU driver（仅仿真后端，真实通信 TODO） | 设备心跳/连接状态 |
| C | 视觉感知与目标状态估计 | 跟踪、深度估计、可组合管线 | DetectionNode（`infer()` 为空，YOLO 推理占位） | Sony 相机真实接入、YOLO ONNX/TensorRT |
| D | 视觉伺服与协同控制 | MVP 跟拍、IBVS、PBVS、ControlAllocator、ServoManager | VelocityCommander（仅处理 linear，角速度 TODO） | MPCController、RLController（仅头文件） |
| E | 任务运镜与人机交互 | VisualServo Action、SetServoMode、SetTrackingTarget、bringup 入口、RViz | Foxglove 入口 | 运镜轨迹节点、Web/语音/遥控交互 |

## 5. 构建与运行

```bash
source /opt/ros/humble/setup.bash
./build.sh                 # rosdep + colcon (C++17 Release)
source install/setup.bash
```

可用的最小验证入口（均真实存在）：
- MVP mock：`ros2 launch servo_control_pkg mvp_mock_test.launch.py scenario:=left`
- 2D 闭环：`ros2 launch servo_control_pkg mvp_2d_sim.launch.py rviz:=true target_motion:=circle`
- 仿真启动：`ros2 launch bringup_pkg fcr_bringup.launch.py use_sim:=true use_rviz:=false`
- 模式切换：`ros2 service call /servo/set_mode vision_servo_msgs/srv/SetServoMode "{mode: 1}"`

## 6. 已知文档/仓库问题（重要，勿照搬错误命令）

1. **`fcr_sim_bringup.launch.py` 不存在**。README §7 与 `team_quick_start.md` §7.4/§8.4 引用了它，但 `bringup_pkg/launch/` 下没有该文件。正确仿真入口是 `fcr_bringup.launch.py use_sim:=true`。
2. **`mock_detector.py` 缺失**。文档 §6.4 让 C 组用 `use_mock_detector:=true` 绕过 YOLO，但 `fcr_bringup.launch.py:55-61` 引用了 `simulation_pkg` 的 `mock_detector.py`，而该脚本不存在 → 该条件分支启动会失败。要么补脚本，要么从文档去掉此验证路径。
3. **README §3.4 误标仿真脚本为 ✅完成**：`target_simulator.py`、`mock_target_publisher.py`、`simple_robot_sim_node.py`、`camera_simulator.py` 全仓均不存在。`simulation_pkg/scripts/` 实际只有 `coop_sim.py`、`servo_bag_eval_node.py`。
4. **文档 §1 引用仓库外 Windows 路径** `D:/githubRepository/notes/fcr_ros2/任务划分.md`，应改为 `docs/project_documentation.md`。
5. **文档 §9.1 公共接口清单漏列 `CalibrateCamera.srv`**。

## 7. 协作边界（改前先同步）

- 不要随意改 `vision_servo_msgs` 下的任何 msg/srv/action（影响 B/C/D/E 全组）。
- 不要改 topic 名、service 名、frame_id、QoS。
- 改公共接口前先在群里同步。
- 每次只让 AI 解决一个清晰的小问题，结论须落到文件路径/函数/topic/运行命令。
