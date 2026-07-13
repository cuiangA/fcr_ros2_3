# BERT 控制台云台链路验证

本文只验证下面这条链路，不包含麦克风、唤醒词和 ASR：

```text
终端输入中文
  -> bert_console_voice_node
  -> BERT 原始分类输出
  -> 云台控制意图解析
  -> /external/voice_command
  -> voice_gimbal_nudge_node
  -> /voice/cmd_gimbal
  -> command_router_node
  -> /cmd_gimbal
  -> gimbal_driver
  -> DJI RS2
```

## 1. 检查 Python 推理环境

在 Jetson 上执行：

```bash
python3 -c "import torch, transformers, safetensors; print(torch.__version__); print(transformers.__version__)"
```

如果只有 `transformers` 或 `safetensors` 缺失：

```bash
python3 -m pip install --user transformers==4.57.6 safetensors
```

Jetson 上的 `torch` 应使用与 JetPack 匹配的 NVIDIA 版本，不建议直接用普通
`pip install torch` 覆盖现有安装。

## 2. 编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  external_control_pkg voice_intent_pkg robot_platform_pkg
source ~/ros2_ws/install/setup.bash
```

## 3. 启动真实云台驱动

终端 1：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch robot_platform_pkg gimbal_bringup.launch.py \
  use_sim:=false \
  can_interface:=can0 \
  control_mode:=incremental_position
```

先确认驱动已连接：

```bash
ros2 topic echo /gimbal/status --once
```

应看到 `connected: true`。

## 4. 启动已经验证过的语音意图控制链

终端 2：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch external_control_pkg voice_control.launch.py \
  start_wake_up_node:=false \
  start_command_router:=true \
  start_keyboard_node:=false
```

## 5. 启动 BERT 控制台节点

终端 3必须直接使用 `ros2 run`，以便节点获得可交互终端：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run voice_intent_pkg bert_console_voice_node --ros-args \
  -p model_dir:=$HOME/ros2_ws/src/fcr_ros2_3/classifier/bert_intent_output \
  -p device:=cpu
```

模型加载完成后会出现：

```text
BERT>
```

依次输入：

```text
向右一点
向左一点
抬高一点
降低一点
快一点
向右一点
慢一点
向左一点
云台停止
```

每次输入后会看到两行关键输出，例如：

```text
[BERT] text="向右一点" | intent=chat | confidence=... | margin=...
[CONTROL] intent=gimbal_nudge_right | source=gimbal_rule | published=true
```

这表示 BERT 已完成推理，但当前模型没有方向类别，所以由明确标记的云台规则层
生成 `gimbal_nudge_right`，之后进入原有控制链路。

输入 `:q` 退出节点。

## 6. 观察中间话题

终端 4可以观察 BERT 结果：

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /external/bert_result std_msgs/msg/String
```

输出是 JSON，包含：

```text
bert_intent       BERT 原始分类结果
bert_confidence   BERT 第一名置信度
bert_margin       第一名和第二名置信度差
control_intent    最终发布的控制意图
control_source    bert / gimbal_rule / none
published         是否发布 VoiceCommand
```

继续向下检查链路：

```bash
ros2 topic echo /external/voice_command external_control_pkg/msg/VoiceCommand
ros2 topic echo /voice/cmd_gimbal vision_servo_msgs/msg/GimbalCmd
ros2 topic echo /cmd_gimbal vision_servo_msgs/msg/GimbalCmd
```

## 7. 当前验证边界

- `gimbal_nudge_left/right/up/down`、`gimbal_stop`、速度调整可以进入实机链路；
- 当前 BERT 训练类别中的 `gimbal_home` 可以被识别和发布，但下游尚未实现回中动作；
- 跟随、距离调整和目标切换可以验证分类结果，但尚没有对应执行节点；
- 如果希望方向控制完全由 BERT 决策，下一步需要给模型增加云台方向类别并重新训练。
