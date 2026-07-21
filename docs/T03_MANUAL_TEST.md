# T03 目标锁定状态机 — 手动测试手册

## 前置

```bash
cd ~/programs/fcr_ros2_3
source /opt/ros/humble/setup.bash
source install/setup.bash
```

> **ByteTrack 轨迹恢复行为**：ByteTrack 通过目标的 position/center/bbox 进行轨迹匹配和恢复。手动测试中所有命令的默认 `center=[0,0]`、`bbox=[0,0,0,0]`，会导致 ByteTrack 把新检测匹配回丢失轨迹，让 selector 永远收到 LOCKED_VISIBLE 而非超时。因此**测试丢失超时需在同一消息中同时发布丢失轨迹和新目标**（场景 G），或者给新目标指定不同的 `center` + `bbox`。

---

## 步骤 1：运行单元测试

```bash
colcon test --packages-select perception_pkg --ctest-args -R test_target_selector --output-on-failure
```

预期输出：`Summary: 1 package finished`，无 failure。

---

## 步骤 2：启动跟踪节点

打开终端 1：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run perception_pkg tracking_node --ros-args \
  -p tracker_type:=bytetrack \
  -p enable_camera_motion_compensation:=false \
  -p auto_select:=true \
  -p allow_auto_switch:=false \
  -p min_confirm_hits:=1 \
  -p new_track_delay_frames:=1 \
  -r detections:=/detections
```

> `min_confirm_hits:=1` + `new_track_delay_frames:=1`：ByteTrack 首帧即创建并确认轨迹。默认 `new_track_delay_frames=2`、`min_confirm_hits=3`，测试时需同时覆盖，否则至少需连续 2 帧才能创建轨迹。真实场景恢复默认值。

保留运行。

---

## 步骤 3：观察状态话题

打开终端 2：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /tracking_node/selector_status
```

初始输出：`state=NO_TARGET id=-1 reason=No target locked`

---

## 步骤 4：模拟测试场景

打开终端 3，按顺序执行每条命令并观察终端 2 的输出变化。

### 场景 A：自动锁定可见目标

```bash
ros2 topic pub --once /detections vision_servo_msgs/msg/TargetArray \
  "{header: {stamp: {sec: 1, nanosec: 0}}, targets: [{id: 1, class_name: 'person', visible: true, tracking_state: 2, confidence: 0.9, width: 100, height: 200}]}"
```

预期：`state=LOCKED_VISIBLE id=0 reason=Auto-selected best visible target`（ByteTrack 从 0 开始分配 ID）

---

### 场景 B：目标丢失，2.5s 内保持锁定

```bash
ros2 topic pub --once /detections vision_servo_msgs/msg/TargetArray \
  "{header: {stamp: {sec: 2, nanosec: 0}}, targets: [{id: 1, class_name: 'person', visible: false, tracking_state: 3, confidence: 0.0, width: 100, height: 200}]}"
```

预期：`state=LOCKED_PERSON_LOST id=0 reason=Person lost, within recovery timeout`

---

### 场景 C：短时恢复（在 2.5s 内）

```bash
ros2 topic pub --once /detections vision_servo_msgs/msg/TargetArray \
  "{header: {stamp: {sec: 3, nanosec: 0}}, targets: [{id: 1, class_name: 'person', visible: true, tracking_state: 2, confidence: 0.9, width: 100, height: 200}]}"
```

预期：`state=LOCKED_VISIBLE id=0 reason=Target visible and confirmed`

---

### 场景 D：手动锁定（通过服务）

```bash
ros2 service call /tracking_node/set_tracking_target \
  vision_servo_msgs/srv/SetTrackingTarget \
  "{target_id: 0, class_name: 'person', enable: true}"
```

> 使用 ByteTrack 分配的 ID `0`（非输入消息的 `id=1`）。如果失败，先用 `ros2 topic echo /tracking_node/tracks --once` 查看实际 ID。

预期返回：`success: true message: "Target locked (manual)"`

查看 `/diagnostics` 确认手动锁定标志：

```bash
ros2 topic echo /diagnostics --once
```

应包含 `selector_manual_lock: "true"`。

---

### 场景 E：手动锁定后禁止自动切换

先切到 LOST，再发布另一个位置不同的可见人物（防止 ByteTrack 恢复旧轨迹），验证不会自动换人。

```bash
# 让手动锁定的目标 0 丢失（confidence=0.0 → ByteTrack 跳过此检测，标记轨迹为 Lost）
ros2 topic pub --once /detections vision_servo_msgs/msg/TargetArray \
  "{header: {stamp: {sec: 4, nanosec: 0}}, targets: [{id: 0, class_name: 'person', visible: false, tracking_state: 3, confidence: 0.0, width: 100, height: 200}]}"

# 发布另一个位置不同的可见目标（center/bbox 避免 ByteTrack 恢复旧轨迹）
ros2 topic pub --once /detections vision_servo_msgs/msg/TargetArray \
  "{header: {stamp: {sec: 4, nanosec: 1000000}}, targets: [{id: 0, class_name: 'person', visible: false, tracking_state: 3, confidence: 0.0, width: 100, height: 200}, {id: 2, class_name: 'person', visible: true, tracking_state: 2, confidence: 0.9, width: 80, height: 180, center: [200.0, 300.0], bbox: [160.0, 210.0, 240.0, 390.0]}]}"
```

预期：`state=LOCKED_PERSON_LOST id=0 reason=Person lost, within recovery timeout`，不会切换到新目标。

---

### 场景 F：手动锁定后 ID 删除 → WAIT_REACQUIRE

```bash
# 先恢复目标 1 可见
ros2 topic pub --once /detections vision_servo_msgs/msg/TargetArray \
  "{header: {stamp: {sec: 20, nanosec: 0}}, targets: [{id: 1, class_name: 'person', visible: true, tracking_state: 2, confidence: 0.9, width: 100, height: 200}]}"

# 手动锁定（注意 ByteTrack 的 ID 可能是 0，如果 set_tracking_target 失败，先用 ros2 topic echo /tracking_node/tracks --once 查看实际 id）
ros2 service call /tracking_node/set_tracking_target \
  vision_servo_msgs/srv/SetTrackingTarget \
  "{target_id: 0, class_name: 'person', enable: true}"

# 发布空数组 + 时间戳大幅前进（模拟 ID 被跟踪器超时删除）
# 必须超过 lost_timeout_seconds=2.5s 后 ByteTrack 才会删除轨迹
ros2 topic pub --once /detections vision_servo_msgs/msg/TargetArray \
  "{header: {stamp: {sec: 30, nanosec: 0}}, targets: []}"
```

预期：`state=WAIT_REACQUIRE id=-1 reason=Locked ID deleted, waiting for reacquisition`

---

### 场景 G：启用 allow_auto_switch，丢失超时后自动切换

**需要重启节点**。在终端 1 按 Ctrl+C 停止节点，重新启动：

```bash
ros2 run perception_pkg tracking_node --ros-args \
  -p tracker_type:=bytetrack \
  -p enable_camera_motion_compensation:=false \
  -p auto_select:=true \
  -p allow_auto_switch:=true \
  -p switch_delay_seconds:=0.0 \
  -p min_confirm_hits:=1 \
  -p new_track_delay_frames:=1 \
  -r detections:=/detections
```

```bash
# 锁定目标（ByteTrack 分配 id=0）
ros2 topic pub --once /detections vision_servo_msgs/msg/TargetArray \
  "{header: {stamp: {sec: 1, nanosec: 0}}, targets: [{id: 1, class_name: 'person', visible: true, tracking_state: 2, confidence: 0.9, width: 100, height: 200}]}"

# 目标丢失 + 新目标可见（时间戳大幅前进，超过 lost_timeout=2.5s）
# 新目标用不同 center，防止 ByteTrack 恢复旧轨迹
ros2 topic pub --once /detections vision_servo_msgs/msg/TargetArray \
  "{header: {stamp: {sec: 100, nanosec: 0}}, targets: [{id: 0, class_name: 'person', visible: false, tracking_state: 3, confidence: 0.0, width: 100, height: 200}, {id: 2, class_name: 'person', visible: true, tracking_state: 2, confidence: 0.9, width: 80, height: 180, center: [200.0, 300.0], bbox: [160.0, 210.0, 240.0, 390.0]}]}"
```

预期：`state=SWITCH_ALLOWED id=* reason=Switched to new target after timeout`（新轨迹 ID 取决于 ByteTrack 内部计数器，上一轮的 `id=0` 在丢失 97s 后被 ByteTrack 删除，新轨迹从新 ID 开始）

---

## 步骤 5：检查诊断信息

```bash
ros2 topic echo /diagnostics --once
```

应包含：

| 字段 | 含义 |
|------|------|
| `selector_state` | 当前状态名 |
| `selector_active_id` | 当前锁定 ID |
| `selector_reason` | 切换/锁定原因 |
| `selector_manual_lock` | 是否手动锁定 |

---

## 验证清单

| # | 检查项 | 操作 | 预期 |
|---|--------|------|------|
| # | 检查项 | 操作 | 预期 |
|---|--------|------|------|
| 1 | 自动锁定 | 发布可见 person | `state=LOCKED_VISIBLE id=0` |
| 2 | 丢失保持 | 同一 id 变 LOST | `state=LOCKED_PERSON_LOST id=0` |
| 3 | 短时恢复 | 变回可见 | `state=LOCKED_VISIBLE id=0` |
| 4 | 手动锁定 | `set_tracking_target` 服务 | `message="Target locked (manual)"` |
| 5 | 手动锁定禁止切换 | manual_lock 后 LOST + 远处新目标 | 不会切到新 id（仍为 `LOCKED_PERSON_LOST id=0`） |
| 6 | WAIT_REACQUIRE | 手动锁定后 ID 被删除（空目标数组） | `state=WAIT_REACQUIRE id=-1` |
| 7 | 自动切换 | `allow_auto_switch:=true`，丢失 + 新目标同帧 | `state=SWITCH_ALLOWED id=新id` |
| 8 | 状态话题 | `ros2 topic echo /tracking_node/selector_status` | 格式 `state=XX id=XX reason=XX` |
| 9 | 诊断上报 | `ros2 topic echo /diagnostics` | 含所有 selector_* 字段 |

---

## 清理

测试完成后，在终端 1 按 `Ctrl+C` 停止跟踪节点。

> ⚠️ 测试中使用了 `min_confirm_hits:=1`、`new_track_delay_frames:=1` 等非默认参数，部署到真实场景时应恢复默认值（`min_confirm_hits=3`、`new_track_delay_frames=2`、`publish_tentative_tracks=false`），否则跟踪器会对短时消失的目标立即创建新轨迹，导致 ID 频繁切换。

---

## 已知限制

### ByteTrack 轨迹恢复影响 Selector 超时测试

ByteTrack 的**丢失轨迹恢复匹配**（lost track recovery）机制会在新检测到达时尝试匹配丢失轨迹。手动测试中所有检测的 `center=[0,0]`、`bbox=[0,0,0,0]`，中心距离为 0，ByteTrack 总会把新检测恢复回旧轨迹，导致 Selector 永远收不到丢失超时信号。

**表现**：即使新检测的 `id` 和旧轨迹不同，ByteTrack 仍会匹配并更新旧轨迹，输出只有旧 ID。Selector 看到可见目标 → `LOCKED_VISIBLE`，等不到超时。

**解决方案**：
- 测试超时切换时，给新目标指定不同的 `center` + `bbox`，使 ByteTrack 无法恢复旧轨迹
- 或使用场景 G 的方式：在同一消息中同时发布丢失目标和新目标，时间戳大幅前进以触发 Selector 超时
- 单元测试已覆盖 Selector 状态机逻辑，不受 ByteTrack 影响
