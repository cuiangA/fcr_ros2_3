# FCR ROS2 Visual Servoing Workspace (fcr_ros2_3)

基于 ROS2 Humble 的智能跟拍机器人项目。LEKIWI 三轮全向底盘 + DJI RS2 云台相机 + YOLO 目标检测 + IBVS/PBVS 视觉伺服。

---

## 项目当前阶段

按照[四阶段渐进路线](https://github.com/cuiangA/fcr_ros2_3)（V1 MVP → V2 稳定化 → V3 混合视觉伺服 → V4 MPC优化），**当前处于 V2.5**：

```
V1  MVP          ████████████████████░  90%  控制闭环与感知代码完整，待相机标定实测
V2  稳定化       █████████████████░░░░  85%  滤波/死区/限幅/目标管理齐全
V3  混合视觉伺服  ██████████████░░░░░░░  70%  IBVS/PBVS/Allocator/热切换已实现
V4  MPC/优化      █░░░░░░░░░░░░░░░░░░░░   5%  仅头文件骨架
```

| 阶段 | 控制方案 | 状态 |
|------|---------|------|
| V1 | 图像误差控制 + 距离控制 + 云台偏角补偿 | ✅ 基本完成（待双相机实测） |
| V2 | V1 + 滤波 + 死区 + 限幅 + 目标锁定 | ✅ 大部分完成 |
| V3 | 云台 IBVS + 底盘 PBVS + 控制分配 | ✅ 算法完成，架构待拆分 |
| V4 | QP / MPC 协同控制 | ❌ 未开始 |

---

## 1. 包结构

```
fcr_ros2_3/
├── build.sh
├── src/
│   ├── vision_servo_msgs/       # 接口定义包（5 msg, 3 srv, 1 action）
│   ├── perception_pkg/          # 感知管线（Sony 检测→目标跟踪）
│   ├── servo_control_pkg/       # 伺服控制（IBVS/PBVS/MPC/RL + MVP跟拍）
│   ├── robot_platform_pkg/      # 硬件平台（底盘/云台/IMU/里程计）
│   ├── simulation_pkg/          # 仿真（Gazebo URDF + Python脚本）
│   └── bringup_pkg/             # 一键启动（launch/config/rviz）
```

---

## 2. 数据流

```
/sony/image_raw  ──→  DetectionNode(YOLO ONNX/TensorRT) ──→ /perception/detections
                                                          │
                                                          ▼
                                                   TrackingNode(SORT)
                                                   /perception/tracks
                                                          │
                                                          ▼
                                                   /perception/tracks
                                                          │（深度融合后续实现）
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

---

## 3. 各模块实现状态

### 3.1 perception_pkg — 感知管线

| 模块 | 状态 | 说明 |
|------|------|------|
| YOLO 检测 (DetectionNode) | 🚧 待Jetson验收 | YOLOv5/v8 ONNX/TensorRT、可测前后处理、NMS、后台最新帧推理和诊断 |
| 多目标跟踪 (MultiObjectTracker) | 🚧 待Jetson验收 | 8 状态 Kalman `[x,y,w,h,vx,vy,vw,vh]` + 时间尺度预测 + Hungarian全局IoU关联 |
| 深度估计 (DepthEstimatorNode) | ⚠️ 历史兼容 | 保留原仿真代码，不进入当前默认启动链 |
| 双相机融合 | ⏳ 后续阶段 | 本轮不实现；待相机驱动、安装方式和标定方案确定后单独设计 |

**跟踪器核心逻辑** ([multi_object_tracker.cpp](src/perception_pkg/src/multi_object_tracker.cpp))：

```
每帧：predict() → associate(IoU + Hungarian) → Kalman correct() → 创建新轨迹 / 删除超龄轨迹
仅在连续命中达到 min_hits(3) 后确认；短时丢失轨迹标记 LOST/visible=false
```

**当前感知主链路**：

```
Sony Image → YOLO ONNX → /perception/detections
→ Kalman + IoU tracking → /perception/tracks
```

### 3.2 servo_control_pkg — 伺服控制

| 模块 | 状态 | 说明 |
|------|------|------|
| IBVS 控制器 | ✅ 完成 | 6x6 交互矩阵，SVD 阻尼伪逆，自适应增益 |
| PBVS 控制器 | ✅ 完成 | 平移 + 旋转解耦（光轴叉积） |
| ControlAllocator | ✅ 完成 | 优先级分配：云台优先旋转，底盘负责平移+剩余旋转 |
| ServoManagerNode | ✅ 完成 | 50Hz 控制循环，pluginlib 加载，VisualServo Action，SetServoMode 服务 |
| MvpFollowControllerNode | ✅ 完成 | 三通道解耦 P 控制 + 单点 IBVS 模式，两级滤波，死区 |
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
| camera_simulator.py | ✅ 完成 | 发布 camera_info（TRANSIENT_LOCAL QoS） |

---

## 4. 接口定义

### Messages（5个）

| Message | 关键字段 |
|---------|---------|
| `Target` | id, class_name, bbox[4], center[2], confidence, position[3], velocity[3], depth_confidence |
| `TargetArray` | Header, Target[] targets, int32 tracking_id |
| `ServoState` | state (IDLE/CONVERGING/TRACKING/LOST/ERROR), feature_error[6], condition_number, camera_velocity[6], gimbal_velocity[2], chassis_velocity[3] |
| `PlatformState` | chassis_pose[3], chassis_velocity[3], gimbal_yaw/pitch/yaw_rate/pitch_rate, angular_velocity[3], emergency_stop, system_mode |
| `GimbalCmd` | yaw_rate, pitch_rate, hold_yaw, hold_pitch |

### Services（3个）

| Service | 用途 |
|---------|------|
| `SetTrackingTarget` | 按 ID 或类别选择跟踪目标 |
| `SetServoMode` | 运行时热切换 IBVS(0)/PBVS(1)/HYBRID(2)/MPC(3)/RL(4) |
| `CalibrateCamera` | 触发相机标定 |

### Actions（1个）

| Action | 用途 |
|--------|------|
| `VisualServo` | 长时间伺服任务，含目标设定、误差容限、超时、实时反馈和取消 |

---

## 5. QoS 策略

| 数据类型 | Reliability | History | Depth | Durability |
|---------|-------------|---------|-------|------------|
| 图像 `/sony/image_raw` | BEST_EFFORT | KEEP_LAST | 1 | VOLATILE |
| 深度图 `/camera/depth/image_raw` | BEST_EFFORT | KEEP_LAST | 1 | VOLATILE |
| 相机内参 `/camera/camera_info` | RELIABLE | KEEP_LAST | 1 | TRANSIENT_LOCAL |
| 检测/跟踪/3D目标 | RELIABLE | KEEP_LAST | 5 | VOLATILE |
| 控制指令 `/cmd_vel`, `/cmd_gimbal` | RELIABLE | KEEP_LAST | 10 | VOLATILE |
| IMU `/imu/data` | BEST_EFFORT | KEEP_LAST | 5 | VOLATILE |
| 平台状态 `/platform/state` | RELIABLE | KEEP_LAST | 10 | TRANSIENT_LOCAL |
| 伺服状态 `/servo/state` | RELIABLE | KEEP_LAST | 5 | VOLATILE |

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

# 完整仿真（Gazebo + 全部节点）
ros2 launch bringup_pkg fcr_sim_bringup.launch.py

# 切换伺服模式
ros2 service call /servo/set_mode vision_servo_msgs/srv/SetServoMode "{mode: 1}"

# 查看控制输出
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_gimbal
```

---

## 8. 待实现

### 高优先级

- **YOLO Jetson 性能验证** — OpenCV DNN 的 ONNX 加载、预热、前后处理和低延迟推理代码已完成；待在 Jetson 上评估 CUDA DNN，并在后续阶段接入 TensorRT engine 后端

### 中优先级

- **真实硬件驱动** — LEKIWI 底盘串口、DJI RS2 云台 CAN 总线、BNO055 IMU I2C 通信均为 TODO
- **架构拆分** — 按 V3 规划将 TF 变换、IBVS/PBVS 控制器拆分为独立节点
- **`velocity_commander_node` 完善** — 当前仅处理 Twist linear 分量，角速度读取为 TODO

### 低优先级（论文扩展方向）

- **MPC 控制器** — 有限时域优化，QP 求解器（OSQP/qpOASES）
- **RL 控制器** — 12 维观测 → 6 维连续动作，ONNX/TorchScript 推理
- **QP 优化型控制分配** — 统一代价函数 + 约束优化

---

## 9. 关键设计决策

1. **底盘不直接追图像水平误差** — 底盘 `wz` 追云台 yaw 偏角，形成串级结构。云台快速修正短时偏移，底盘慢速消除长期偏角，避免画面抖动。
2. **历史深度节点** — 当前仅用于兼容既有仿真，不作为两台独立相机的融合实现。
3. **插件化控制器** — pluginlib + 抽象基类，运行时热切换，方便论文扩展。
4. **Factory Pattern 实/仿共用** — 硬件接口层通过 `use_sim` 参数切换，上层算法代码完全不变。
5. **ComposableNode 零拷贝** — 感知三阶段同进程顺序调用，消除序列化开销。
