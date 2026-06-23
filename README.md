# FCR ROS2 Visual Servoing Workspace (fcr_ros2_3)

基于 ROS2 Humble 的智能 Vlog 跟拍机器人项目。目标硬件为 TRON2 底盘 + DJI RS2 云台 + Sony ZV-E10 II + Orbbec 335 深度相机；当前代码保留 LEKIWI/三轮全向仿真抽象，用于验证视觉伺服和运镜控制链路。

---

## 项目当前阶段

按照[四阶段渐进路线](https://github.com/cuiangA/fcr_ros2_3)（V1 MVP → V2 稳定化 → V3 混合视觉伺服 → V4 MPC优化），**当前处于 V2.5**：

```
V1  MVP          ████████████████████░  90%  控制闭环完整, YOLO 推理待填充
V2  稳定化       █████████████████░░░░  85%  滤波/死区/限幅/目标管理齐全
V3  混合视觉伺服  ██████████████░░░░░░░  70%  IBVS/PBVS/Allocator/热切换已实现
V4  MPC/优化      █░░░░░░░░░░░░░░░░░░░░   5%  仅头文件骨架
```

| 阶段 | 控制方案 | 状态 |
|------|---------|------|
| V1 | 图像误差控制 + 距离控制 + 云台偏角补偿 | ✅ 基本完成（YOLO 推理为占位） |
| V2 | V1 + 滤波 + 死区 + 限幅 + 目标锁定 | ✅ 大部分完成 |
| V3 | 云台 IBVS + 底盘 PBVS + 控制分配 | ✅ 算法完成，架构待拆分 |
| V4 | 虚拟拍摄位姿 / QP / MPC 协同控制 | ⚠️ 已加入虚拟运镜骨架，MPC 未开始 |

---

## 1. 包结构

```
fcr_ros2_3/
├── build.sh
├── src/
│   ├── vision_servo_msgs/       # 接口定义包（7 msg, 3 srv, 2 action）
│   ├── perception_pkg/          # 感知管线（检测→跟踪→深度估计）
│   ├── servo_control_pkg/       # 伺服控制（IBVS/PBVS/MPC/RL + MVP跟拍）
│   ├── robot_platform_pkg/      # 硬件平台（底盘/云台/IMU/里程计）
│   ├── simulation_pkg/          # 仿真（Gazebo URDF + Python脚本）
│   └── bringup_pkg/             # 一键启动（launch/config/rviz）
```

---

## 2. 数据流

```
/camera/image_raw  ──→  DetectionNode(YOLO)  ──→  /perception/detections
                         [⚠ 推理占位，仿真用 mock]         │
                                                          ▼
                                                   TrackingNode(SORT)
                                                   /perception/tracks
                                                          │
                                                          ▼
                                                   DepthEstimatorNode
                                                   /perception/targets_3d
                                                          │
/platform/state  ←──  PlatformManagerNode  ←──  ServoManagerNode
                                                         │
                                              ┌──────────┼──────────┐
                                              ▼                     ▼
                                         /cmd_vel              /cmd_gimbal
                                      (TwistStamped)          (GimbalCmd)
                                              │                     │
                                              ▼                     ▼
                                        ChassisDriver         GimbalDriver
```

### 轻量替代路径（MVP 跟拍）

```
mock_target_publisher / simple_robot_sim
        ↓
/target/current
        ↓
mvp_follow_controller_node    ← 自包含，不依赖 pluginlib / ControlAllocator
        ↓
/cmd_vel + /cmd_gimbal
```

### 实验性运镜链路（真实目标 + 虚拟拍摄位姿）

```
/target/current
      ↓
TargetStateEstimatorNode
      ↓
/target/state  ───────────────┐
                               ↓
                      ShotReferenceGeneratorNode
                               ↓
                         /shot/reference
                               ↓
              ┌────────────────┴────────────────┐
              ↓                                 ↓
     BasePoseControllerNode           GimbalTargetControllerNode
              ↓                                 ↓
    /control/cmd_vel_raw          /control/cmd_gimbal_raw
              └────────────────┬────────────────┘
                               ↓
                    CommandSafetyFilterNode
                               ↓
                    /cmd_vel + /cmd_gimbal
```

核心原则：云台始终用真实目标做软锁定和构图控制；底盘在跟随模式下追真实目标的固定相对位姿，在运镜模式下追随围绕真实目标生成的虚拟拍摄位姿。

---

## 3. 各模块实现状态

### 3.1 perception_pkg — 感知管线

| 模块 | 状态 | 说明 |
|------|------|------|
| YOLO 检测 (DetectionNode) | ⚠️ 占位 | `infer()` 为空；架构完整，支持 ONNX/TensorRT/OpenCV DNN 三种推理路径 |
| 多目标跟踪 (MultiObjectTracker) | ✅ 完成 | 8 状态 Kalman [x,y,w,h,vx,vy,vw,vh] + IoU 匈牙利贪心匹配 |
| 深度估计 (DepthEstimatorNode) | ✅ 完成 | 双缓存异步融合：深度图采样（优先）→ bbox 面积推算（回退） |
| 目标状态估计 (TargetStateEstimatorNode) | ✅ 新增 | 将相机光学坐标系目标点 TF 到 `odom`，并估计目标速度 |
| 组合流水线 (PerceptionPipeline) | ✅ 完成 | ComposableNode，detection→tracking→depth 三阶段零拷贝 |

**跟踪器核心逻辑** ([multi_object_tracker.cpp](src/perception_pkg/src/multi_object_tracker.cpp))：

```
每帧：predict() → associate(IoU 贪心) → Kalman correct() → 创建新轨迹 / 删除超龄轨迹
仅输出 total_visible_count ≥ min_hits(3) 的已确认轨迹，防止假阳性
```

**深度估计两级回退** ([depth_estimator_node.cpp](src/perception_pkg/src/depth_estimator_node.cpp))：

```
方法1（置信度 0.8）：bbox 中心点深度图采样 → Z = depth / 1000
方法2（置信度 0.5）：depth = sqrt((fx·fy·real_w·real_h) / area_px)  → 无需深度图
反投影：X = (u-cx)/fx·Z, Y = (v-cy)/fy·Z
```

### 3.2 servo_control_pkg — 伺服控制

| 模块 | 状态 | 说明 |
|------|------|------|
| IBVS 控制器 | ✅ 完成 | 6x6 交互矩阵，SVD 阻尼伪逆，自适应增益 |
| PBVS 控制器 | ✅ 完成 | 平移 + 旋转解耦（光轴叉积） |
| ControlAllocator | ✅ 完成 | 优先级分配：云台优先旋转，底盘负责平移+剩余旋转 |
| ServoManagerNode | ✅ 完成 | 50Hz 控制循环，pluginlib 加载，VisualServo Action，SetServoMode 服务 |
| MvpFollowControllerNode | ✅ 完成 | 三通道解耦 P 控制 + 单点 IBVS 模式，两级滤波，死区 |
| ShotReferenceGeneratorNode | ✅ 新增 | 生成跟随/运镜共用的虚拟拍摄位姿，支持 orbit/dolly/truck/arc/pan/recenter |
| BasePoseControllerNode | ✅ 新增 | 底盘追踪虚拟拍摄位姿，输出 `/control/cmd_vel_raw` |
| GimbalTargetControllerNode | ✅ 新增 | 云台始终追真实目标图像位置，输出 `/control/cmd_gimbal_raw` |
| CommandSafetyFilterNode | ✅ 新增 | 急停、陈旧数据检测、限幅，统一输出最终 `/cmd_vel` 和 `/cmd_gimbal` |
| MPC 控制器 | ❌ 占位 | 仅有头文件，QP 求解器接口预留 |
| RL 控制器 | ❌ 占位 | 仅有头文件，ONNX 推理接口预留 |

**MVP 跟拍控制架构** ([mvp_follow_controller_node.cpp](src/servo_control_pkg/src/mvp_follow_controller_node.cpp))：

```
内环（云台，高带宽）：ex,ey → P 或 2x3 角速度 IBVS → gimbal_yaw/pitch
外环（底盘，低带宽）：ez = Z - Z_desired → base_vx
                      q_yaw（云台偏角）→ base_wz   ← 串级：底盘不直接追 ex
信号链：raw → LP(α=0.5) → deadband → P gain → LP(α=0.3) → clamp → output
```

**控制律**：

```
gimbal_yaw   = -Kx · e_x          (e_x = (cx - W/2) / (W/2))
gimbal_pitch = -Ky · e_y
base_vx      =  Kz · e_z          (e_z = Z - Z_desired)
base_wz      =  Kb · q_yaw        (追云台偏角，不追图像误差)
```

### 3.3 robot_platform_pkg — 硬件平台

| 模块 | 状态 | 说明 |
|------|------|------|
| 三轮全向运动学 | ✅ 完成 | 120° 对称布局，正/逆运动学矩阵预计算 |
| 底盘驱动 (LEKIWI) | ⚠️ 仿真完成 | 真实串口通信为 TODO |
| 云台驱动 (DJI RS2) | ⚠️ 仿真完成 | 真实 CAN 总线通信为 TODO |
| IMU 驱动 (BNO055) | ⚠️ 仿真完成 | 真实 I2C 通信为 TODO |
| 里程计 | ✅ 完成 | 轮式里程计 + IMU 融合 |
| PlatformManager | ✅ 完成 | 聚合底盘/云台/IMU 状态为 PlatformState |

全部使用 Factory Pattern：`use_sim` 参数切换真实/模拟实现，上层算法节点不感知硬件模式。

### 3.4 simulation_pkg — 仿真

| 模块 | 状态 | 说明 |
|------|------|------|
| URDF/XACRO 机器人模型 | ✅ 完成 | 底盘 + 云台 + 深度相机 + IMU，模块化组合 |
| Gazebo 世界 | ✅ 完成 | 含目标物 |
| target_simulator.py | ✅ 完成 | 圆形/8字形/直线轨迹，支持 TF 变换链 |
| mock_target_publisher.py | ✅ 完成 | 合成 TargetArray（center/left/right/up/down/far/near/lost/sinusoidal） |
| simple_robot_sim_node.py | ✅ 完成 | 轻量 2D 闭环仿真（不依赖 Gazebo），用于 MVP 测试 |
| virtual shot markers | ✅ 新增 | 2D 仿真中显示虚拟拍摄位姿和目标-拍摄位姿连线 |
| camera_simulator.py | ✅ 完成 | 发布 camera_info（TRANSIENT_LOCAL QoS） |

---

## 4. 接口定义

### Messages（7个）

| Message | 关键字段 |
|---------|---------|
| `Target` | id, class_name, bbox[4], center[2], confidence, position[3], velocity[3], depth_confidence |
| `TargetArray` | Header, Target[] targets, int32 tracking_id |
| `ServoState` | state (IDLE/CONVERGING/TRACKING/LOST/ERROR), feature_error[6], condition_number, camera_velocity[6], gimbal_velocity[2], chassis_velocity[3] |
| `PlatformState` | chassis_pose[3], chassis_velocity[3], gimbal_yaw/pitch/yaw_rate/pitch_rate, angular_velocity[3], emergency_stop, system_mode |
| `GimbalCmd` | yaw_rate, pitch_rate, hold_yaw, hold_pitch |
| `TargetState` | odom frame 下的目标 position/velocity/speed，保留 image_center/bbox 给云台构图 |
| `ShotReference` | 虚拟底盘位姿、运镜类型、状态机状态、速度预算、构图目标 |

### Services（3个）

| Service | 用途 |
|---------|------|
| `SetTrackingTarget` | 按 ID 或类别选择跟踪目标 |
| `SetServoMode` | 运行时热切换 IBVS(0)/PBVS(1)/HYBRID(2)/MPC(3)/RL(4) |
| `CalibrateCamera` | 触发相机标定 |

### Actions（2个）

| Action | 用途 |
|--------|------|
| `VisualServo` | 长时间伺服任务，含目标设定、误差容限、超时、实时反馈和取消 |
| `CinematicShot` | 目标相对运镜任务，支持推拉、绕拍、侧跟、弧线推进、取消和反馈 |

---

## 5. QoS 策略

| 数据类型 | Reliability | History | Depth | Durability |
|---------|-------------|---------|-------|------------|
| 图像 `/camera/image_raw` | BEST_EFFORT | KEEP_LAST | 1 | VOLATILE |
| 深度图 `/camera/depth/image_raw` | BEST_EFFORT | KEEP_LAST | 1 | VOLATILE |
| 相机内参 `/camera/camera_info` | RELIABLE | KEEP_LAST | 1 | TRANSIENT_LOCAL |
| 检测/跟踪/3D目标 | RELIABLE | KEEP_LAST | 5 | VOLATILE |
| 控制指令 `/cmd_vel`, `/cmd_gimbal` | RELIABLE | KEEP_LAST | 10 | VOLATILE |
| IMU `/imu/data` | BEST_EFFORT | KEEP_LAST | 5 | VOLATILE |
| 平台状态 `/platform/state` | RELIABLE | KEEP_LAST | 10 | TRANSIENT_LOCAL |
| 伺服状态 `/servo/state` | RELIABLE | KEEP_LAST | 5 | VOLATILE |
| 目标状态 `/target/state` | RELIABLE | KEEP_LAST | 10 | VOLATILE |
| 运镜参考 `/shot/reference` | RELIABLE | KEEP_LAST | 10 | VOLATILE |

---

## 6. 可扩展控制器架构

```
ServoControllerBase (抽象基类)
├── IBVSController    ✅   v = -λ · L⁺ · (s - s*)
├── PBVSController    ✅   v_trans = +Kt·(P-P*), ω 来自光轴叉积
├── Hybrid(2.5D)      ⚠️   映射到 IBVSController
├── MPCController     ❌   有限时域 QP 优化（论文扩展）
└── RLController      ❌   12维观测→6维动作（论文扩展）
```

运行时切换：

```bash
ros2 service call /servo/set_mode vision_servo_msgs/srv/SetServoMode "{mode: 1}"
# 0=IBVS, 1=PBVS, 2=HYBRID, 3=MPC, 4=RL
```

添加新控制器只需继承基类 → 实现 `computeVelocity()` → 注册到 `plugins.xml`。

---

## 7. 快速开始

```bash
# 构建
./build.sh
source install/setup.bash

# MVP mock 测试（不依赖 Gazebo）
ros2 launch servo_control_pkg mvp_mock_test.launch.py scenario:=left

# MVP 2D 闭环仿真
ros2 launch servo_control_pkg mvp_2d_sim.launch.py rviz:=true

# 虚拟运镜 2D 闭环仿真
ros2 launch servo_control_pkg virtual_shot_2d_sim.launch.py shot_name:=orbit rviz:=true

# Foxglove 2D 环绕运镜验证：目标慢速直线运动，机器人连续绕目标运镜
ros2 launch servo_control_pkg orbit_foxglove_2d_sim.launch.py

# 如果已安装 foxglove_bridge，可同时启动 WebSocket bridge
ros2 launch servo_control_pkg orbit_foxglove_2d_sim.launch.py foxglove_bridge:=true

# 完整仿真（Gazebo + 全部节点）
ros2 launch bringup_pkg fcr_sim_bringup.launch.py

# 切换伺服模式
ros2 service call /servo/set_mode vision_servo_msgs/srv/SetServoMode "{mode: 1}"

# 查看控制输出
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_gimbal
ros2 topic echo /shot/reference
```

Foxglove 可视化建议添加：

- `3D Panel`：Fixed frame 设为 `odom`
- `/tf`：查看 `odom -> base_link -> gimbal_yaw_link -> camera_link -> camera_optical_link`
- `/odom`：机器人位姿
- `/visualization_marker_array`：机器人、目标、目标轨迹、机器人轨迹、环绕半径、虚拟拍摄位姿
- `/shot/reference`：确认 `shot_type=5`、`state=4/5`、`orbit_radius` 和 `orbit_angle`

---

## 8. 待实现

### 高优先级

- **YOLO 模型加载和推理** — `detection_node.cpp` 的 `load_model()` 和 `infer()` 当前为空，需接入 ONNX Runtime 或 TensorRT 实现真实目标检测

### 中优先级

- **真实硬件驱动** — LEKIWI 底盘串口、DJI RS2 云台 CAN 总线、BNO055 IMU I2C 通信均为 TODO
- **TRON2 / DJI RS2 / Sony / Orbbec 实机接入** — 当前新增运镜链路已预留接口，真实 SDK、外参标定和设备状态反馈为 TODO
- **Sony 与 Orbbec 外参标定** — 推荐二者同装云台，固定 `sony_optical_frame` 与 `orbbec_optical_frame` 的 TF；标定流程和标定文件加载为 TODO
- **障碍物和碰撞约束** — 当前虚拟拍摄位姿只考虑目标相对几何，不包含局部避障、安全距离和可通行区域
- **`velocity_commander_node` 完善** — 当前仅处理 Twist linear 分量，角速度读取为 TODO

### 低优先级（论文扩展方向）

- **MPC 控制器** — 有限时域优化，QP 求解器（OSQP/qpOASES）
- **RL 控制器** — 12 维观测 → 6 维连续动作，ONNX/TorchScript 推理
- **QP 优化型控制分配** — 统一代价函数 + 约束优化

---

## 9. 关键设计决策

1. **底盘不直接追图像水平误差** — 底盘 `wz` 追云台 yaw 偏角，形成串级结构。云台快速修正短时偏移，底盘慢速消除长期偏角，避免画面抖动。
2. **双缓存异步融合** — 深度图和检测结果频率不匹配时，任意一路到达即触发融合，避免时间同步延迟。
3. **插件化控制器** — pluginlib + 抽象基类，运行时热切换，方便论文扩展。
4. **Factory Pattern 实/仿共用** — 硬件接口层通过 `use_sim` 参数切换，上层算法代码完全不变。
5. **ComposableNode 零拷贝** — 感知三阶段同进程顺序调用，消除序列化开销。
6. **真实目标和虚拟拍摄位姿解耦** — 云台数据流始终追真实目标；底盘数据流在运镜模式下追虚拟拍摄位姿，在跟随模式下追固定目标相对位姿。
7. **目标速度进入运镜状态机** — `TargetStateEstimatorNode` 在 `odom` 下估计目标速度；当目标速度或所需底盘速度超过预算时，`ShotReferenceGeneratorNode` 进入限速、保持或跟随回退状态。
