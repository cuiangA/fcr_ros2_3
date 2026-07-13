# 语音指令解析清单

## 1. 文档目的

本文档定义 FCR ROS 2 项目当前及后续需要支持的自然语言控制指令，包括：

- 控制对象识别；
- 标准意图名称；
- 参数提取；
- ROS 2 路由目标；
- 歧义和安全处理；
- 当前实现状态。

指令解析不使用 `setMode`。用户直接说出任务，例如“跟上我”“开始运镜”“拍一张照片”，系统根据控制对象和动作意图启动对应功能。

## 2. 统一解析结构

自然语言不应只映射为一个扁平标签。建议统一解析为：

```text
控制对象 target + 动作 action + 参数 slots
```

推荐输出结构：

```json
{
  "raw_text": "云台向右移动一点",
  "target": "gimbal",
  "action": "nudge",
  "direction": "right",
  "amount": "small",
  "distance": -1.0,
  "unit": "",
  "speed": "",
  "target_id": -1,
  "target_desc": "",
  "confidence": 0.98,
  "source": "bert_or_rule"
}
```

### 2.1 控制对象

| `target` | 含义 | 路由目标 |
|---|---|---|
| `system` | 系统、安全和状态 | safety/status manager |
| `gimbal` | DJI RS2 云台 | `/voice/cmd_gimbal` |
| `chassis` | 移动底盘 | `/voice/cmd_vel` |
| `camera` | 拍照和录像 | camera control node |
| `autonomy` | 跟随、运镜等自主任务 | autonomy manager |

### 2.2 通用参数

| 参数 | 可选值或格式 | 说明 |
|---|---|---|
| `direction` | `left/right/up/down/forward/backward` | 运动方向 |
| `amount` | `small/medium/large` | 模糊动作幅度 |
| `distance` | 浮点数，默认 `-1.0` | 指定移动或跟随距离 |
| `unit` | `m/cm` | 距离单位 |
| `speed` | `slow/mid/fast` | 速度等级 |
| `target_id` | 整数，默认 `-1` | 追踪目标 ID |
| `target_desc` | 字符串 | 目标外观描述 |

## 3. 安全与系统指令

| 标准意图 | 示例语句 | 参数 | 状态 |
|---|---|---|---|
| `emergency_stop` | 急停、紧急停止、立即停止所有动作 | 无 | 待接入统一安全节点 |
| `stop_all` | 停止所有动作、全部停下 | 无 | 规划中 |
| `cancel_action` | 取消当前任务、停止当前动作 | 无 | 规划中 |
| `status_query` | 当前状态怎么样、云台连接正常吗 | `target` 可选 | BERT 可识别，执行端待补充 |

安全要求：

1. `emergency_stop` 的优先级最高。
2. 只有明确出现“急停”或“紧急停止”时才触发硬件急停。
3. “停止录像”“停止跟随”“停止云台”不能误判为全局急停。

## 4. 云台控制指令

| 当前路由意图 | 结构化结果 | 示例语句 | 状态 |
|---|---|---|---|
| `gimbal_nudge_left` | `target=gimbal, action=nudge, direction=left` | 云台向左一点、镜头往左转一点 | 已解析、已实机执行 |
| `gimbal_nudge_right` | `target=gimbal, action=nudge, direction=right` | 云台向右一点、镜头往右移动一点 | 已解析、已实机执行 |
| `gimbal_nudge_up` | `target=gimbal, action=nudge, direction=up` | 云台向上一点、把镜头抬高一点 | 已解析、已实机执行 |
| `gimbal_nudge_down` | `target=gimbal, action=nudge, direction=down` | 云台向下一点、镜头降低一点 | 已解析、已实机执行 |
| `gimbal_stop` | `target=gimbal, action=stop` | 停止云台、云台别动 | 已解析、已执行 |
| `gimbal_home` | `target=gimbal, action=home` | 云台回中、镜头回正、云台归位 | 已解析、已实机执行 |
| `gimbal_speed_up` | `target=gimbal, action=speed_adjust, speed=fast` | 云台快一点、镜头转快一些 | 已解析、已执行 |
| `gimbal_speed_down` | `target=gimbal, action=speed_adjust, speed=slow` | 云台慢一点、镜头转慢一些 | 已解析、已执行 |

当前云台命令通过以下链路执行：

```text
/external/voice_command
  -> voice_gimbal_nudge_node
  -> /voice/cmd_gimbal
  -> command_router_node
  -> /cmd_gimbal
  -> gimbal_driver
  -> DJI RS2
```

## 5. 底盘控制指令

| 标准意图 | 结构化结果 | 示例语句 | 状态 |
|---|---|---|---|
| `chassis_move_forward` | `action=translate, direction=forward` | 底盘前进、向前走一点 | 待底盘型号确定后接入 |
| `chassis_move_backward` | `action=translate, direction=backward` | 底盘后退、往后一点 | 待接入 |
| `chassis_move_left` | `action=translate, direction=left` | 底盘向左平移 | 待接入 |
| `chassis_move_right` | `action=translate, direction=right` | 底盘向右平移 | 待接入 |
| `chassis_turn_left` | `action=rotate, direction=left` | 底盘左转、向左转身 | 待接入 |
| `chassis_turn_right` | `action=rotate, direction=right` | 底盘右转、向右转身 | 待接入 |
| `chassis_stop` | `action=stop` | 停止底盘、底盘别动 | 待接入 |
| `chassis_speed_up` | `action=speed_adjust, speed=fast` | 底盘快一点、走快一些 | 待接入 |
| `chassis_speed_down` | `action=speed_adjust, speed=slow` | 底盘慢一点、走慢一些 | 待接入 |

底盘语义必须区分平移和旋转：

```text
“底盘向右移动” -> chassis_move_right
“底盘向右转”   -> chassis_turn_right
“云台向右转”   -> gimbal_nudge_right
```

## 6. 相机控制指令

| 标准意图 | 结构化结果 | 示例语句 | 状态 |
|---|---|---|---|
| `camera_take_photo` | `target=camera, action=take_photo` | 拍照、拍一张照片、相机拍照 | 已解析，执行节点待实现 |
| `camera_start_recording` | `target=camera, action=start_recording` | 开始录像、开始录制、录一段视频 | 已解析，执行节点待实现 |
| `camera_stop_recording` | `target=camera, action=stop_recording` | 停止录像、结束录制 | 已解析，执行节点待实现 |

规划中的扩展指令：

| 标准意图 | 示例语句 | 前置条件 |
|---|---|---|
| `camera_zoom_in` | 放大画面、拉近一点 | 相机支持变焦接口 |
| `camera_zoom_out` | 缩小画面、拉远一点 | 相机支持变焦接口 |
| `camera_focus` | 对焦、重新对焦 | 相机支持对焦接口 |

建议的相机执行链路：

```text
camera_take_photo
  -> camera_control_node
  -> 读取 /camera/image_raw
  -> 保存单帧图片

camera_start_recording
  -> camera_control_node
  -> 连续读取 /camera/image_raw
  -> 写入视频文件

camera_stop_recording
  -> camera_control_node
  -> 结束写入并关闭视频文件
```

## 7. 自主跟随指令

| 标准意图 | 示例语句 | 参数 | 状态 |
|---|---|---|---|
| `start_following` | 跟上我、开始跟随目标 | `target_id/target_desc` | BERT 已支持，任务执行端待完善 |
| `stop_following` | 停止跟随、别再跟了 | 无 | BERT 已支持，执行端待完善 |
| `pause_following` | 暂停跟随、先停一下 | 无 | 规划中 |
| `resume_following` | 继续跟随、接着跟 | 无 | 规划中 |
| `distance_adjust` | 靠近一点、离远一些 | `direction/distance/unit` | BERT 已支持，参数提取待完善 |
| `speed_adjust` | 跟快一点、跟慢一点 | `speed` | 规划中 |
| `switch_target` | 切换目标、跟踪二号目标 | `target_id/target_desc` | BERT 已支持，执行端待完善 |

参数示例：

```json
{
  "distance_direction": "closer",
  "distance": 0.5,
  "unit": "m",
  "speed": "fast",
  "target_id": 2,
  "target_desc": "穿红衣服的人"
}
```

## 8. 自主运镜指令

| 标准意图 | 示例语句 | 状态 |
|---|---|---|
| `start_cinematic` | 开始运镜、开始自动拍摄 | 规划中 |
| `stop_cinematic` | 停止运镜、结束自动拍摄 | 规划中 |
| `orbit_target` | 环绕目标拍摄、绕着目标转 | 规划中 |
| `push_in` | 镜头推进、慢慢靠近目标 | 规划中 |
| `pull_out` | 镜头拉远、慢慢后退 | 规划中 |
| `track_target` | 镜头跟住目标、保持目标居中 | 规划中 |

运镜任务可能同时控制：

```text
底盘运动 + 云台姿态 + 相机录像
```

因此应由 `autonomy_manager_node` 统一编排，而不是由语音节点直接操作多个硬件节点。

## 9. 指令优先级与路由

建议总优先级：

```text
emergency_stop
  > manual
  > voice
  > autonomy
```

对象路由：

```text
target=system   -> safety/status manager
target=gimbal  -> command_router_node -> gimbal_driver
target=chassis -> command_router_node -> chassis_driver
target=camera  -> camera_control_node
target=autonomy -> autonomy_manager_node
```

## 10. 歧义处理规则

1. “向右一点”未说明对象时，不默认控制底盘。
2. “镜头向右”表示云台运动，“拍照、录像”表示相机采集。
3. “停止录像”优先匹配 `camera_stop_recording`，不能匹配全局停止。
4. “停止跟随”优先匹配 `stop_following`。
5. `gimbal_home` 必须明确包含“回中、回正、归位”等词。
6. “底盘右转”和“底盘向右平移”必须分开。
7. 文本夹杂少量字母或标点时，可使用仅中文字符副本进行规则匹配。
8. BERT 没有对应类别时，不能因为置信度高就强制执行不相关动作。
9. 涉及硬件运动的未知或歧义指令默认不执行。
10. 复合指令应拆分并按顺序执行，例如“开始跟随并开始录像”。

## 11. 当前 BERT 模型边界

当前 BERT 是七分类模型：

```text
start_following
stop_following
switch_target
distance_adjust
gimbal_home
status_query
chat
```

它尚不能直接区分：

```text
云台向右
底盘向右
相机拍照
相机开始录像
```

当前云台方向和相机采集意图由确定性规则补充。后续重新训练时，建议优先加入：

- 云台方向、停止、回中；
- 底盘平移、旋转、停止；
- 相机拍照、开始录像、停止录像；
- 跟随和运镜任务；
- 安全停止。

长期可采用“控制对象分类 + 动作分类 + 参数抽取”的分层模型，避免为每一种对象、动作、方向组合建立大量扁平标签。

## 12. 实施顺序

| 优先级 | 工作内容 |
|---|---|
| P0 | 固化云台方向、停止、回中和安全保护 |
| P0 | 增加控制对象字段，区分云台、底盘、相机 |
| P1 | 实现相机拍照和录像执行节点 |
| P1 | 底盘型号确定后接入底盘语音控制 |
| P1 | 接通跟随任务管理和目标切换 |
| P2 | 实现自主运镜任务编排 |
| P2 | 扩展训练集并重新训练分层意图模型 |
