# T06：分离云台目标和底盘目标

## 1. 任务背景

当前系统中，`mvp_follow_controller_node` 仅订阅 `/perception/tracks`（`TargetArray`），从中提取目标信息同时用于云台控制和底盘控制。`/perception/aim_target_2d`（`AimTarget2D`）已由 `face_aim_node` 发布，但无任何节点消费。

### 当前数据流

```
/perception/tracks (TargetArray)
  ├── face_aim_node → /perception/aim_target_2d (无消费端)
  └── mvp_follow_controller_node
        ├── 云台 yaw/pitch ← target.center / target.bbox
        └── 底盘距离控制 ← target.position[2] (深度)
```

### 目标数据流

```
/perception/tracks (TargetArray)
  └── mvp_follow_controller_node
        └── 底盘距离控制 ← target.position[2] (深度)

/perception/aim_target_2d (AimTarget2D)
  └── mvp_follow_controller_node
        └── 云台 yaw/pitch ← pixel_x / pixel_y
```

### 完成标准

1. 改变脸部瞄准点不影响底盘距离控制
2. 瞄准消息中断后云台安全停止（250ms 超时）
3. 云台停止时不能保留上一条非零速度指令

## 2. 实施步骤

### 步骤 1：修改 `mvp_follow_controller_node.cpp`

**文件：** `src/servo_control_pkg/src/mvp_follow_controller_node.cpp`

#### 1.1 新增头文件

在现有 include 块中新增：

```cpp
#include <vision_servo_msgs/msg/aim_target2_d.hpp>
```

#### 1.2 新增成员变量

在 `// ── O) ROS2 通信接口` 区域新增：

```cpp
// 瞄准目标（AimTarget2D）— 用于云台 yaw/pitch 控制
rclcpp::Subscription<vision_servo_msgs::msg::AimTarget2D>::SharedPtr aim_target_sub_;
std::string aim_target_topic_;
vision_servo_msgs::msg::AimTarget2D active_aim_target_;
bool has_valid_aim_ = false;
rclcpp::Time last_valid_aim_time_;
double aim_target_timeout_ = 0.25;
```

#### 1.3 新增参数声明

在 `load_parameters()` 中新增：

```cpp
aim_target_topic_ = declare_described_parameter<std::string>(
    "aim_target_topic", "/perception/aim_target_2d",
    "AimTarget2D topic for gimbal yaw/pitch control.");
aim_target_timeout_ = declare_described_parameter<double>(
    "aim_target_timeout", 0.25,
    "Seconds before the aim target is treated as lost.");
```

#### 1.4 新增订阅者

在构造函数中，`target_sub_` 之后新增：

```cpp
aim_target_sub_ = create_subscription<vision_servo_msgs::msg::AimTarget2D>(
    aim_target_topic_, rclcpp::SensorDataQoS(),
    std::bind(&MvpFollowControllerNode::aim_target_callback, this, std::placeholders::_1));
```

#### 1.5 新增回调函数

```cpp
void aim_target_callback(const vision_servo_msgs::msg::AimTarget2D::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_aim_target_ = *msg;
    has_valid_aim_ = msg->valid;
    if (msg->valid) {
        last_valid_aim_time_ = now();
    }
}
```

#### 1.6 修改 `compute_control()` 控制逻辑

- 云台 yaw/pitch 从 `active_aim_target_.pixel_x` / `active_aim_target_.pixel_y` 计算
- 新增 `has_recent_aim()` 检查，超时（`aim_target_timeout_`，默认 0.25s）后重置云台滤波器并输出零速
- 底盘控制逻辑不变，仍从 `active_target_`（`/perception/tracks`）提取深度

#### 1.7 修改 `publish_command()` 云台指令

- 瞄准超时时，`hold_yaw`/`hold_pitch` 设为 `true`（保持当前位置）
- 同时重置云台滤波器，确保不保留上一条非零速度指令

### 步骤 2：修改 `mvp_gimbal_only.yaml`

**文件：** `src/servo_control_pkg/config/mvp_gimbal_only.yaml`

新增参数：

```yaml
aim_target_topic: /perception/aim_target_2d
aim_target_timeout: 0.25
```

### 步骤 3：修改 `mvp_follow_core.launch.py`

**文件：** `src/servo_control_pkg/launch/mvp_follow_core.launch.py`

可选：新增 `aim_target_topic` launch 参数，传递给 controller node。

## 3. 测试方案

### 3.1 新创建的测试文件

为实现自动化验收测试，新增以下文件：

| 文件 | 说明 |
|---|---|
| `src/simulation_pkg/scripts/aim_mock_source.py` | Mock 源节点：同时发布 `/perception/tracks`（TargetArray）和 `/perception/aim_target_2d`（AimTarget2D），按时间阶段切换瞄准点位置和有效性 |
| `src/simulation_pkg/scripts/aim_separation_acceptance.py` | 验收监控节点：验证分离性、超时停车、零速保持三项标准 |
| `src/bringup_pkg/launch/aim_separation_acceptance.launch.py` | 验收测试启动文件 |

### 3.2 自动化验收测试

#### 测试流程

`aim_mock_source.py` 按时间驱动 5 个阶段：

| 阶段 | 时间 | 行为 | 预期 |
|---|---|---|---|
| 0 | 0-1s | 无目标 | 系统空闲 |
| 1 | 1-4s | 目标激活，瞄准点 (320, 240) 中心 | 云台跟踪，底盘建立深度基线 |
| 2 | 4-5s | 瞄准点跳变到右侧 (480, 240) | 云台 yaw 响应，底盘速度不变 |
| 3 | 5-6s | 瞄准点跳变到上部 (320, 120) | 云台 pitch 响应，底盘速度不变 |
| 4 | 6-8s | `aim.valid = false` 瞄准丢失 | 云台 250ms 内停止，速度归零 |
| 5 | 8s+ | 瞄准点恢复有效 | 云台重新跟踪 |

#### 运行命令

```bash
source install/setup.bash
ros2 launch bringup_pkg aim_separation_acceptance.launch.py
```

#### 验证条件（自动检查）

1. **分离性**：瞄准点跳变期间（阶段 2-3），底盘线速度（`/cmd_vel` linear.x）变化量 < 0.01 m/s
2. **超时停止**：阶段 4 进入后，云台角速度（`/cmd_gimbal` yaw_rate/pitch_rate）在 300ms 内归零
3. **零速保持**：停止后 yaw_rate 和 pitch_rate 绝对值均 ≤ 1e-4

测试结束时输出 JSON 结果，退出码 0 表示 PASS：

```json
{
  "result": "PASS",
  "target_seen": true,
  "gimbal_stopped": true,
  "separation_violation": false,
  "gimbal_yaw_final": 0.0,
  "gimbal_pitch_final": 0.0,
  "stop_latency_ms": 180.0
}
```

### 3.3 手动测试

#### 3.3.1 话题监控测试

启动完整系统后，用 ROS 2 CLI 手动验证各通道：

```bash
# 终端 1：启动云台跟随模式
ros2 launch bringup_pkg fcr_mvp_gimbal_follow.launch.py

# 终端 2：监控云台指令
ros2 topic echo /cmd_gimbal --once

# 终端 3：监控底盘指令
ros2 topic echo /cmd_vel --once

# 终端 4：监控瞄准目标
ros2 topic echo /perception/aim_target_2d --once

# 终端 5：监控跟踪目标
ros2 topic echo /perception/tracks --once
```

#### 3.3.2 改变瞄准点不影响底盘

```bash
# 1. 在 face_aim_node 运行中，动态修改瞄准偏移参数：
ros2 param set /face_aim_node aim_offset_ratio 0.50

# 2. 观察 /cmd_gimbal 的 yaw_rate/pitch_rate 应变化
# 3. 观察 /cmd_vel 的 linear.x 应保持不变（云台通道独立）
# 4. 改回默认值：
ros2 param set /face_aim_node aim_offset_ratio 0.20
```

#### 3.3.3 瞄准超时停车测试

```bash
# 1. 确认云台正在跟踪（/cmd_gimbal 非零）
# 2. 手动停止 face_aim_node（模拟感知中断）：
ros2 run rqt_console rqt_console &
kill -STOP $(pgrep -f face_aim_node)

# 3. 观察 /cmd_gimbal — 应在 250ms 内变为零速
# 注意：此处仅暂停进程，实际部署中 face_aim_node 崩溃或网络中断效果相同

# 4. 恢复 face_aim_node：
kill -CONT $(pgrep -f face_aim_node)
# 云台应重新开始跟踪
```

#### 3.3.4 云台零速保持验证

```bash
# 1. 在云台跟踪期间记录非零指令值
# 2. 中断瞄准源后检查滤波器状态（仅通过日志）：
ros2 run rqt_console rqt_console
# 日志应显示 "Aim target timeout (0.250s); resetting gimbal filters"
# 3. 确认 /cmd_gimbal 输出 yaw_rate=0, pitch_rate=0
```

### 3.4 验收标准验证

| 标准 | 自动化测试 | 手动测试 |
|---|---|---|
| 改变脸部瞄准点不影响底盘距离控制 | `aim_separation_acceptance.launch.py` 阶段 2-3 检查 | `ros2 param set` 改变 `aim_offset_ratio`，对比 `/cmd_vel` |
| 瞄准消息中断后云台安全停止 | `aim_separation_acceptance.launch.py` 阶段 4 检查 | `kill -STOP` 暂停 face_aim_node，观察 `/cmd_gimbal` |
| 云台停止时不能保留非零速度 | `aim_separation_acceptance.launch.py` 阶段 4 末检查 | 检查日志中的滤波器重置消息 |

## 4. 涉及文件

### 4.1 修改的文件

| 文件 | 操作 | 说明 |
|---|---|---|
| `src/servo_control_pkg/src/mvp_follow_controller_node.cpp` | 修改 | 新增 aim_target 订阅、回调、超时逻辑、控制流修改 |
| `src/servo_control_pkg/config/mvp_gimbal_only.yaml` | 修改 | 新增 `aim_target_topic` 和 `aim_target_timeout` 参数 |
| `src/servo_control_pkg/config/mvp_follow_controller.yaml` | 修改 | 同上 |
| `src/servo_control_pkg/launch/mvp_follow_core.launch.py` | 修改 | 新增 `aim_target_topic` launch 参数 |
| `src/simulation_pkg/CMakeLists.txt` | 修改 | 安装新脚本 |

### 4.2 新增的文件

| 文件 | 说明 |
|---|---|
| `src/simulation_pkg/scripts/aim_mock_source.py` | Mock 源：同时发布 TargetArray 和 AimTarget2D，按阶段切换 |
| `src/simulation_pkg/scripts/aim_separation_acceptance.py` | 验收监控：验证分离性、超时停车、零速保持 |
| `src/bringup_pkg/launch/aim_separation_acceptance.launch.py` | 验收测试启动文件 |
| `docs/T06_separate_gimbal_and_chassis_target.md` | 本文档 |
