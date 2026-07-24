# 语音底盘控制测试

## 1. 链路

```text
双层意图模型
  -> /external/voice_command
  -> voice_command_dispatcher_node
  -> /voice/chassis_command
  -> voice_chassis_nudge_node
  -> /voice/cmd_vel
  -> chassis_command_router_node
  -> /cmd_vel
  -> chassis_driver
```

底盘仲裁优先级：

```text
e_stop > manual > voice > autonomy
```

默认语音点动参数：

```text
线速度：0.05 m/s
角速度：0.20 rad/s
持续时间：0.4 s
```

## 2. 构建

```bash
cd ~/ros2_ws/src/fcr_ros2_3
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select external_control_pkg voice_intent_pkg robot_platform_pkg \
  --allow-overriding external_control_pkg voice_intent_pkg robot_platform_pkg \
  --cmake-args -DBUILD_WAKE_UP_NODE=OFF

colcon test --packages-select external_control_pkg voice_intent_pkg
colcon test-result --verbose
source install/setup.bash
```

## 3. 第一级：只验证目标分流

终端 1：

```bash
ros2 launch external_control_pkg voice_control.launch.py \
  start_wake_up_node:=false \
  start_intent_classifier:=false \
  start_command_router:=false \
  start_chassis_control:=true
```

终端 2：

```bash
ros2 topic echo /voice/dispatch_status
```

终端 3：

```bash
ros2 topic echo /voice/chassis_command
```

发布测试意图：

```bash
ros2 topic pub --once /external/voice_command \
  external_control_pkg/msg/VoiceCommand \
  "{header: {frame_id: test}, intents: [chassis_move_forward], confidences: [1.0], raw_text: '底盘向前移动一点', distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"
```

预期：

```text
/voice/dispatch_status: target=chassis, accepted=true, reason=routed
/voice/chassis_command: intents=[chassis_move_forward]
```

## 4. 第二级：仿真底盘完整链路

终端 1启动仿真底盘：

```bash
ros2 launch robot_platform_pkg platform.launch.py \
  use_sim:=true \
  enable_chassis:=true \
  enable_imu:=false
```

终端 2启动语音控制层：

```bash
ros2 launch external_control_pkg voice_control.launch.py \
  start_wake_up_node:=false \
  start_intent_classifier:=false \
  start_command_router:=false \
  start_chassis_control:=true
```

终端 3观察最终速度：

```bash
ros2 topic echo /cmd_vel geometry_msgs/msg/TwistStamped
```

终端 4观察仿真反馈：

```bash
ros2 topic echo /chassis/odom_raw nav_msgs/msg/Odometry
```

依次发布：

```bash
ros2 topic pub --once /external/voice_command external_control_pkg/msg/VoiceCommand \
  "{header: {frame_id: test}, intents: [chassis_move_forward], confidences: [1.0], raw_text: '底盘向前一点', distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"

ros2 topic pub --once /external/voice_command external_control_pkg/msg/VoiceCommand \
  "{header: {frame_id: test}, intents: [chassis_move_backward], confidences: [1.0], raw_text: '底盘向后一点', distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"

ros2 topic pub --once /external/voice_command external_control_pkg/msg/VoiceCommand \
  "{header: {frame_id: test}, intents: [chassis_move_left], confidences: [1.0], raw_text: '底盘向左一点', distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"

ros2 topic pub --once /external/voice_command external_control_pkg/msg/VoiceCommand \
  "{header: {frame_id: test}, intents: [chassis_move_right], confidences: [1.0], raw_text: '底盘向右一点', distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"
```

期望值：

| 意图 | `/cmd_vel` 非零分量 |
|---|---|
| `chassis_move_forward` | `linear.x = +0.05` |
| `chassis_move_backward` | `linear.x = -0.05` |
| `chassis_move_left` | `linear.y = +0.05` |
| `chassis_move_right` | `linear.y = -0.05` |

每条命令持续约 0.4 秒，随后必须出现全零速度。

## 5. 第三级：双层模型到仿真底盘

保持第 4 节两个 launch 运行，再启动模型控制台：

```bash
source ~/venvs/fcr_bert/bin/activate
VENV_SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
source /opt/ros/humble/setup.bash
source ~/ros2_ws/src/fcr_ros2_3/install/setup.bash
export PYTHONPATH="$VENV_SITE${PYTHONPATH:+:$PYTHONPATH}"
export CUDA_VISIBLE_DEVICES=""

ros2 run voice_intent_pkg double_layer_console_voice_node --ros-args \
  -p model_root:=$HOME/ros2_ws/models/classifier_current \
  -p embedding_model_dir:=$HOME/ros2_ws/models/bge-base-zh-v1.5 \
  -p device:=cpu
```

输入：

```text
底盘向前移动一点
底盘向后移动一点
底盘向左移动一点
底盘向右移动一点
```

同时检查：

```bash
ros2 topic echo /external/intent_result
ros2 topic echo /voice/dispatch_status
ros2 topic echo /cmd_vel
ros2 topic echo /chassis/odom_raw
```

## 6. 急停验证

运动期间发布：

```bash
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: true}"
```

`/cmd_vel` 必须立即变为全零。解除急停：

```bash
ros2 topic pub --once /e_stop std_msgs/msg/Bool "{data: false}"
```

解除后必须等待下一条新命令，底盘不能自行恢复运动。

## 7. 实机前置条件

只有以上测试全部通过后才进行实机测试：

1. 三个轮子离地；
2. 确认串口设备存在；
3. 确认电机 ID 为 `7/8/9`；
4. 保持 `0.05 m/s` 和 `0.20 rad/s` 低速；
5. 操作员准备断电和急停；
6. 先直接测试 `/cmd_vel`，再测试模型链路。

检查串口：

```bash
ls -l /dev/serial/by-id/
```

启动真实底盘：

```bash
CONFIG=$(ros2 pkg prefix --share robot_platform_pkg)/config/chassis_params.yaml
ros2 run robot_platform_pkg chassis_driver_node --ros-args \
  --params-file "$CONFIG" \
  -p use_sim:=false
```

如果实际串口与配置不同：

```bash
ros2 run robot_platform_pkg chassis_driver_node --ros-args \
  --params-file "$CONFIG" \
  -p use_sim:=false \
  -p serial_device:=/dev/serial/by-id/实际设备名
```

实机第一次只测试 `chassis_move_forward`。若轮子方向错误，立即停止，不继续测试
其他方向，先修正电机方向或运动学参数。
