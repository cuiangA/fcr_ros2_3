# DJI RS2 云台实机控制方案与 Bringup 步骤

本文记录当前项目中 DJI RS2 云台的实机接入结论、控制方案、关键参数和最小验证流程。内容基于当前仓库代码和 Jetson Orin Nano Developer Kit 上的实机验证。

## 1. 当前结论

当前云台硬件链路已经打通：

```text
键盘 / 语音 / 手动 topic
  -> command_router_node
  -> /cmd_gimbal
  -> gimbal_driver
  -> SocketCAN can0
  -> DJI RS2
```

已验证能力：

| 项目 | 当前结论 |
| --- | --- |
| CAN 接口 | `can0` 可用，bitrate `1000000`，状态 `ERROR-ACTIVE` |
| CAN ID | host -> gimbal: `0x223`；gimbal -> host: `0x222` |
| 状态读取 | `0E 02` 查询可用，`/gimbal/status` 可稳定返回 yaw/pitch 和连接状态 |
| 绝对位置控制 | `0E 00` + 控制字节 `0x01` 可让 RS2 实际转动 |
| 原生速度控制 | `0E 01` 可发出 CAN 帧，但 RS2 当前不执行，也没有观察到明确 `0E 01` 返回 |
| 原生增量位置控制 | `0E 00` + 控制字节 `0x00` 可发出 CAN 帧，但 RS2 当前不执行 |
| 当前工程方案 | 上层仍发布 `GimbalCmd.yaw_rate/pitch_rate`，驱动内部累加为绝对 yaw/pitch 目标，再用 `0E 00` + `0x01` 执行 |

因此，当前 `control_mode=incremental_position` 的含义不是直接使用 RS2 原生增量模式，而是：

```text
GimbalCmd 角速度输入
  -> gimbal_driver 根据 dt 积分成小角度增量
  -> 累加到内部绝对目标角
  -> 发送 DJI RS2 绝对位置控制帧 0E 00, ctrl=0x01
```

这样做的原因是：项目上层接口仍保持速度语义，后续视觉伺服、键盘、语音都不用改；底层使用已经实测可动的 RS2 绝对位置控制。

## 2. 涉及节点

### 2.1 云台驱动链路

```text
/cmd_gimbal
  -> gimbal_driver
  -> SocketCAN can0
  -> RS2
  -> /gimbal/status
```

关键节点：

| 节点 | 包 | 作用 |
| --- | --- | --- |
| `gimbal_driver` | `robot_platform_pkg` | 订阅 `/cmd_gimbal`，发送 RS2 CAN 控制帧，发布 `/gimbal/status` 和 `/joint_states` |
| `command_router_node` | `external_control_pkg` | 仲裁手动、语音、自主云台命令，统一输出到 `/cmd_gimbal` |
| `keyboard_gimbal_control_node` | `external_control_pkg` | 从键盘读取 `WASD`，发布 `/manual/cmd_gimbal` |
| `voice_gimbal_nudge_node` | `external_control_pkg` | 将 `/external/voice_command` 中的云台意图转换为 `/voice/cmd_gimbal` |

### 2.2 键盘遥控链路

```text
keyboard_gimbal_control_node
  -> /manual/cmd_gimbal
  -> command_router_node
  -> /cmd_gimbal
  -> gimbal_driver
  -> RS2
```

按键含义：

| 按键 | 含义 |
| --- | --- |
| `A` / `D` | yaw 左/右 |
| `W` / `S` | pitch 上/下 |
| 空格 | 停止当前点动 |
| `Q` | 退出键盘节点 |

注意：键盘节点不能通过 `ros2 launch` 直接启动后在同一个 launch 输出里输入按键。若出现：

```text
stdin is not a TTY; keyboard control is disabled
```

说明键盘节点没有拿到真实终端输入。此时应使用 `ros2 run external_control_pkg keyboard_gimbal_control_node` 单独启动。

### 2.3 语音微调链路

```text
/external/voice_command
  -> voice_gimbal_nudge_node
  -> /voice/cmd_gimbal
  -> command_router_node
  -> /cmd_gimbal
  -> gimbal_driver
  -> RS2
```

当前已经验证手动发布 `VoiceCommand` 可以触发 RS2 云台动作。真实麦克风、唤醒和 ASR 前端仍是后续工作。

## 3. 关键参数

云台驱动参数位于：

```text
src/robot_platform_pkg/config/gimbal_params.yaml
```

| 参数 | 当前建议值 | 含义 |
| --- | --- | --- |
| `autostart` | `true` | 节点启动后自动 configure + activate |
| `use_sim` | `false` | `false` 表示连接真实 RS2 |
| `can_interface` | `can0` | SocketCAN 接口名 |
| `control_mode` | `incremental_position` | 上层速度输入，驱动内部累加为绝对位置目标 |
| `max_yaw_rate` | `1.0` | 驱动层 yaw 角速度输入限幅，单位 rad/s |
| `max_pitch_rate` | `1.0` | 驱动层 pitch 角速度输入限幅，单位 rad/s |
| `command_timeout_sec` | `0.5` | 命令超时时间 |
| `status_publish_rate_hz` | `10.0` | `/gimbal/status` 发布频率 |
| `incremental_position_duration_sec` | `0.1` | 每个 RS2 位置目标的执行时间，最小 0.1s |
| `incremental_position_max_step_deg` | `2.0` | 单次积分后的最大角度变化，防止突跳 |
| `incremental_position_default_dt_sec` | `0.05` | 第一帧或时间异常时用于 `rate -> delta` 的默认周期 |
| `incremental_position_max_dt_sec` | `0.1` | 积分时允许使用的最大周期 |
| `debug_position_yaw_deg` | `5.0` | `/gimbal/debug_position` 的绝对 yaw 调试目标 |
| `debug_position_pitch_deg` | `0.0` | `/gimbal/debug_position` 的绝对 pitch 调试目标 |
| `debug_position_duration_sec` | `0.5` | `/gimbal/debug_position` 的执行时间 |

键盘和语音微调常用参数：

| 参数 | 所属节点 | 含义 |
| --- | --- | --- |
| `yaw_step_rate` | `keyboard_gimbal_control_node` / `voice_gimbal_nudge_node` | yaw 点动角速度输入，单位 rad/s |
| `pitch_step_rate` | `keyboard_gimbal_control_node` / `voice_gimbal_nudge_node` | pitch 点动角速度输入，单位 rad/s |
| `step_duration_sec` | `keyboard_gimbal_control_node` / `voice_gimbal_nudge_node` | 单次点动持续时间 |
| `right_yaw_sign` | `keyboard_gimbal_control_node` / `voice_gimbal_nudge_node` | 右转方向符号 |
| `up_pitch_sign` | `keyboard_gimbal_control_node` / `voice_gimbal_nudge_node` | 上抬方向符号 |

实机点动不明显时，可以先临时提高键盘参数：

```bash
ros2 run external_control_pkg keyboard_gimbal_control_node \
  --ros-args -p step_duration_sec:=0.8 -p yaw_step_rate:=0.5
```

## 4. 实机 Bringup 步骤

以下步骤假设工作区为：

```text
~/ros2_ws
```

### 4.1 配置 CAN

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

期望看到：

```text
state UP
can state ERROR-ACTIVE
bitrate 1000000
```

### 4.2 编译相关包

```bash
cd ~/ros2_ws
colcon build --packages-select vision_servo_msgs robot_platform_pkg external_control_pkg \
  --allow-overriding vision_servo_msgs robot_platform_pkg external_control_pkg
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

确认 ROS 找到的是当前工作区安装结果：

```bash
ros2 pkg prefix robot_platform_pkg
```

期望类似：

```text
/home/nvidia/ros2_ws/install/robot_platform_pkg
```

### 4.3 启动云台驱动

终端 1：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch robot_platform_pkg gimbal_bringup.launch.py \
  use_sim:=false \
  can_interface:=can0 \
  control_mode:=incremental_position
```

### 4.4 验证云台连接状态

终端 2：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 param get /gimbal_driver control_mode
ros2 topic echo /gimbal/status --once
```

期望：

```text
String value is: incremental_position
connected: true
crc_error_count: 0
can_error_count: 0
parse_error_count: 0
```

### 4.5 直接测试 `/cmd_gimbal`

```bash
timeout 2s ros2 topic pub -r 20 /cmd_gimbal vision_servo_msgs/msg/GimbalCmd \
"{yaw_rate: -0.35, pitch_rate: 0.0, hold_yaw: false, hold_pitch: true}"
```

如果云台动作，说明：

```text
/cmd_gimbal -> gimbal_driver -> CAN -> RS2
```

已经可用。

### 4.6 启动命令路由

终端 3：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch external_control_pkg voice_control.launch.py \
  start_wake_up_node:=false \
  start_command_router:=true \
  start_keyboard_node:=false
```

这里不要通过 launch 启动键盘节点，否则可能拿不到 TTY。

### 4.7 启动 WASD 键盘遥控

终端 4：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run external_control_pkg keyboard_gimbal_control_node \
  --ros-args -p step_duration_sec:=0.8 -p yaw_step_rate:=0.5
```

在终端 4 里按 `A/D/W/S`，云台应产生对应动作。

### 4.8 验证键盘和路由 Topic

终端 5：

```bash
ros2 topic echo /manual/cmd_gimbal vision_servo_msgs/msg/GimbalCmd
```

终端 6：

```bash
ros2 topic echo /cmd_gimbal vision_servo_msgs/msg/GimbalCmd
```

按键时两个 topic 都应有输出。

### 4.9 测试语音意图链路

先保证 `voice_gimbal_nudge_node` 和 `command_router_node` 已通过上面的 `voice_control.launch.py` 启动。

发布“向右一点”：

```bash
ros2 topic pub --once /external/voice_command external_control_pkg/msg/VoiceCommand \
"{intents: ['gimbal_nudge_right'], confidences: [1.0], raw_text: '向右一点', distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"
```

发布“向左一点”：

```bash
ros2 topic pub --once /external/voice_command external_control_pkg/msg/VoiceCommand \
"{intents: ['gimbal_nudge_left'], confidences: [1.0], raw_text: '向左一点', distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"
```

可观察：

```bash
ros2 topic echo /voice/cmd_gimbal vision_servo_msgs/msg/GimbalCmd
ros2 topic echo /cmd_gimbal vision_servo_msgs/msg/GimbalCmd
```

## 5. CAN 过滤命令

### 5.1 观察发往 RS2 的控制帧

```bash
candump -tz can0,223:7FF | awk '/AA 1A 00 03/{n=4} n>0{print;n--}'
```

当前可用控制应表现为：

```text
0E 00 ... 01 ...
```

其中：

| 字段 | 含义 |
| --- | --- |
| `0E 00` | 位置控制 |
| yaw/pitch 字段 | 目标绝对角度，单位 0.1 deg |
| `01` | 绝对位置控制模式 |
| time 字节 | 执行时间，单位 100ms |

### 5.2 观察 RS2 返回状态

```bash
candump -tz can0,222:7FF | awk '/AA/ {n=4} n>0 {print; n--}'
```

当前稳定返回主要是：

```text
0E 02
```

表示云台状态/当前位置查询返回。

### 5.3 过滤速度控制响应

```bash
candump -tz can0,222:7FF | grep --line-buffered -B1 -A2 '0E 01'
```

当前实测没有观察到有效 `0E 01` 响应。

## 6. 常见问题

### 6.1 `gimbal_bringup.launch.py` not found

通常是没有从正确工作区重新编译或 source。

处理：

```bash
cd ~/ros2_ws
colcon build --packages-select robot_platform_pkg --allow-overriding robot_platform_pkg
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix robot_platform_pkg
```

### 6.2 参数未声明

如果出现：

```text
parameter 'debug_position_yaw_deg' cannot be set because it was not declared
```

说明运行的还是旧版 `gimbal_driver`。重新编译、source 并重启节点。

### 6.3 键盘节点提示 stdin is not a TTY

不要通过 launch 启动键盘节点。使用：

```bash
ros2 run external_control_pkg keyboard_gimbal_control_node
```

并在该终端内按 `WASD`。

### 6.4 `/cmd_gimbal` 直接发布能动，WASD 不动

按顺序检查：

```bash
ros2 topic echo /manual/cmd_gimbal vision_servo_msgs/msg/GimbalCmd
ros2 topic echo /cmd_gimbal vision_servo_msgs/msg/GimbalCmd
ros2 topic info /manual/cmd_gimbal -v
ros2 topic info /cmd_gimbal -v
```

正常链路应为：

```text
/manual/cmd_gimbal:
  publisher: keyboard_gimbal_control_node
  subscriber: command_router_node

/cmd_gimbal:
  publisher: command_router_node
  subscriber: gimbal_driver
```

### 6.5 CAN 有 `0E 00` 但云台不动

检查控制字节：

| 控制字节 | 结果 |
| --- | --- |
| `00` | RS2 原生增量模式，当前实测不动作 |
| `01` | RS2 绝对位置模式，当前实测可动作 |

当前驱动应使用 `01`。

## 7. 后续工作

建议后续按以下顺序推进：

1. 将 yaw/pitch 软件限位做成参数，而不是固定 `[-pi, pi]` 和 `[-pi/2, pi/2]`。
2. 继续排查 `0E 01` 速度控制，重点验证 DJI SDK 原始 `set_speed()` demo、控制字节和 ACK 返回。
3. 将真实麦克风、唤醒和 ASR 前端接入 `/external/voice_command`。
4. 为云台 bringup 录制 rosbag 或保存固定测试脚本，便于队友复现。
