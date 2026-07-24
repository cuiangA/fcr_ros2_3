# 语音控制架构与执行边界

## 1. 当前正式数据流

```text
ASR 或控制台文本
  -> 唯一意图分类节点
  -> /external/voice_command
  -> voice_command_dispatcher_node
       |- /voice/gimbal_command
       |- /voice/chassis_command
       |- /voice/camera_command
       |- /voice/autonomy_command
       `- /voice/system_command
```

分发器只读取 `VoiceCommand.intents`。`raw_text` 仅用于日志和诊断，禁止执行
节点根据原始文本猜测硬件动作。

## 2. 云台执行链路

```text
/voice/gimbal_command
  -> voice_gimbal_nudge_node
  -> /voice/cmd_gimbal
  -> command_router_node
  -> /cmd_gimbal
  -> gimbal_driver
  -> DJI RS2
```

`voice_gimbal_nudge_node.enable_raw_text_fallback` 默认是 `false`。只有专门
验证旧模型兼容性时才允许临时开启，正式实机运行不得开启。

## 3. 分发规则

| 目标 | 输出话题 | 当前执行状态 |
|---|---|---|
| `system` | `/voice/system_command` | 等待安全与状态管理节点 |
| `gimbal` | `/voice/gimbal_command` | 方向、停止、回中已接入 |
| `chassis` | `/voice/chassis_command` | 点动桥接与仲裁已实现，等待实机验收 |
| `camera` | `/voice/camera_command` | 等待相机控制节点 |
| `autonomy` | `/voice/autonomy_command` | 等待自主任务管理节点 |

`/voice/dispatch_status` 表示意图是否通过验证并被路由，不代表硬件执行成功。

## 4. 安全约束

1. `底盘向右` 只能路由到 `chassis`，不能到达云台执行节点。
2. `stop` 和 `stop_current_action` 因缺少目标而拒绝。
3. 必须使用 `gimbal_stop`、`chassis_stop`、`camera_stop_recording`、
   `stop_following`、`stop_all` 或 `emergency_stop`。
4. 低于分发器 `min_confidence` 的意图不路由。
5. 默认 0.5 秒内完全相同的来源、文本和意图只执行一次。
6. `/external/voice_command` 出现多个发布者时，分发器会告警。正式运行只允许
   一个意图来源。

## 5. 运行模式

### 新版模型验证

```text
关闭 wake_up_node
启动 double_layer_console_voice_node
启动 dispatcher、云台桥接、router 和 driver
```

### 真实语音与本地双层模型

```text
wake_up_node -> /voice/text -> intent_classifier_node
```

启动参数：

```bash
ros2 launch external_control_pkg voice_control.launch.py \
  start_wake_up_node:=true \
  start_intent_classifier:=true \
  publish_cloud_intents:=false \
  classifier_model_root:=$HOME/ros2_ws/models/classifier_current \
  embedding_model_dir:=$HOME/ros2_ws/models/bge-base-zh-v1.5
```

`publish_cloud_intents:=false` 是关键约束：云端只提供 `raw_text`，意图统一由
本地双层模型生成。兼容旧云端意图模式时，应保持
`start_intent_classifier:=false` 和 `publish_cloud_intents:=true`。

## 6. 诊断

```bash
ros2 topic echo /external/voice_command
ros2 topic echo /voice/dispatch_status
ros2 topic echo /voice/gimbal_command
ros2 topic info /external/voice_command -v
```

典型拒绝原因：

| `reason` | 含义 |
|---|---|
| `unknown_intent` | 不在正式意图白名单 |
| `empty_intents` | 消息没有提供任何意图 |
| `ambiguous_stop_requires_target` | 停止指令缺少控制对象 |
| `confidence_below_threshold` | 置信度不足 |
| `duplicate_suppressed` | 短时间重复指令被抑制 |
| `routed` | 已路由到对应目标话题 |

## 7. 最小安全验收

构建后启动控制层，不启动真实云台也可以先检查分流：

```bash
ros2 launch external_control_pkg voice_control.launch.py \
  start_wake_up_node:=false \
  start_command_router:=false
```

终端 A：

```bash
ros2 topic echo /voice/dispatch_status
```

终端 B：

```bash
ros2 topic echo /voice/gimbal_command
```

发布底盘右移：

```bash
ros2 topic pub --once /external/voice_command \
  external_control_pkg/msg/VoiceCommand \
  "{header: {frame_id: test}, intents: [chassis_move_right], confidences: [1.0], raw_text: '底盘向右移动一点', distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"
```

验收结果：

- `/voice/dispatch_status` 显示 `target: chassis` 和 `reason: routed`；
- `/voice/gimbal_command` 必须没有消息。

发布云台右移：

```bash
ros2 topic pub --once /external/voice_command \
  external_control_pkg/msg/VoiceCommand \
  "{header: {frame_id: test}, intents: [gimbal_nudge_right], confidences: [1.0], raw_text: '云台向右一点', distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"
```

此时 `/voice/gimbal_command` 必须收到 `gimbal_nudge_right`。

发布模糊停止：

```bash
ros2 topic pub --once /external/voice_command \
  external_control_pkg/msg/VoiceCommand \
  "{header: {frame_id: test}, intents: [stop_current_action], confidences: [1.0], raw_text: '停一下', distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"
```

验收结果必须是：

```text
accepted: false
reason: ambiguous_stop_requires_target
```
