# T04 face_aim_node 框架

## 任务说明

创建一个 C++ ROS 2 Humble 节点 `face_aim_node`，桥接感知管线（YOLO 检测 + ByteTrack 跟踪）与视觉伺服控制。

### 输入

| 话题 | 类型 | QoS | 说明 |
|------|------|-----|------|
| `/sony/image_raw` | `sensor_msgs/Image` | `SensorDataQoS` (BEST_EFFORT + KEEP_LAST(1)) | Sony 相机原始图像 |
| `/perception/tracks` | `vision_servo_msgs/TargetArray` | RELIABLE + KEEP_LAST(1) | ByteTrack 多目标跟踪输出 |

### 输出

| 话题 | 类型 | 说明 |
|------|------|------|
| `/perception/aim_target_2d` | `vision_servo_msgs/AimTarget2D` | 当前跟踪人物的瞄准点（像素坐标） |

### 核心逻辑

1. 只处理 `tracking_id` 对应的人物（`class_name == "person"`）
2. 图像和轨迹按源时间戳（`header.stamp`）匹配
3. 不使用人脸模型时，回退使用人物 bbox 中心作为瞄准点
4. 缓存必须有界——只保留最新一帧待处理图像和轨迹
5. 没有订阅者时跳过调试图生成
6. 输入超时后发布 `valid=false`

### 完成标准

- 不接入人脸模型时，节点也能稳定运行
- 不影响人体检测和 ByteTrack 帧率

---

## 实施步骤

### Step 1: 创建头文件 `face_aim_node.hpp`

**路径**：`src/perception_pkg/include/perception_pkg/face_aim_node.hpp`

#### 类设计

```
FaceAimNode : rclcpp::Node
├── 订阅
│   ├── image_sub_    → /sony/image_raw      (SensorDataQoS)
│   └── tracks_sub_   → /perception/tracks   (qos::perception)
├── 发布
│   ├── aim_pub_      → /perception/aim_target_2d
│   └── debug_pub_    → /perception/face_aim_debug (image_transport)
├── 定时器
│   └── timeout_timer_ → 定期检查输入超时
├── 有界缓存（各 1 帧）
│   ├── pending_image_
│   └── pending_tracks_
├── 同步机制
│   └── try_match_and_process() → 当 image 和 tracks 的 stamp 匹配时处理
└── 配置参数
    ├── input_timeout_seconds_  (默认 2.0s)
    └── max_tolerance_ns_       (时间戳匹配容忍窗口，默认 1ms)
```

#### 关键方法

| 方法 | 触发时机 | 行为 |
|------|----------|------|
| `image_callback()` | 收到图像 | 替换 `pending_image_`，更新最后输入时间，尝试匹配 |
| `tracks_callback()` | 收到轨迹 | 替换 `pending_tracks_`，更新最后输入时间，尝试匹配 |
| `try_match_and_process()` | 任一回调 | 若两缓存均非空且 stamp 在容忍窗口内匹配，则处理并清空缓存 |
| `check_timeout()` | 定时器（10Hz） | 若距最后输入超过阈值，发布 `valid=false` |
| `publish_debug_image()` | 处理后 | 仅在 `count_subscribers() > 0` 时渲染标注图 |

#### 瞄准点决策（无人脸模型时）

```
tracking_id == -1 → valid=false
target.class_name != "person" → 跳过
否则 → pixel_x = target.center[0], pixel_y = target.center[1]
        source = UPPER_BODY (3)
        valid = true
```

### Step 2: 创建实现文件 `face_aim_node.cpp`

**路径**：`src/perception_pkg/src/face_aim_node.cpp`

#### 时间戳匹配逻辑

```cpp
bool stamps_match(const builtin_interfaces::msg::Time& a,
                  const builtin_interfaces::msg::Time& b) {
  const int64_t diff_ns =
      (static_cast<int64_t>(a.sec) - static_cast<int64_t>(b.sec)) * 1'000'000'000LL +
      (static_cast<int64_t>(a.nanosec) - static_cast<int64_t>(b.nanosec));
  return std::abs(diff_ns) <= max_tolerance_ns_;
}
```

#### 处理流水线

```
[image_callback]    [tracks_callback]
       |                    |
       v                    v
  pending_image_ =   pending_tracks_ =
    msg                 msg
       |                    |
       +--------+-----------+
                |
                v
       try_match_and_process()
                |
                v
       stamps_match()?
          /        \
        YES        NO
         |          |
         v          v
   找 tracking_id   等待另一帧
   对应 target
         |
         v
   填充 AimTarget2D
   pixel_x = target.center[0]
   pixel_y = target.center[1]
   source = UPPER_BODY
   valid = true
         |
         v
   publish()
   clear 缓存
```

#### 超时检测

```cpp
void check_timeout() {
  const auto now = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration<double>(
      now - last_input_time_).count();
  if (elapsed > input_timeout_seconds_) {
    AimTarget2D msg;
    msg.header.stamp = now();
    msg.valid = false;
    msg.tracking_id = -1;
    aim_pub_->publish(msg);
  }
}
```

### Step 3: 修改 `CMakeLists.txt`

在 `tracking_node` 之后添加：

```cmake
add_executable(face_aim_node src/face_aim_node.cpp)
target_include_directories(face_aim_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(face_aim_node
  rclcpp sensor_msgs vision_servo_msgs cv_bridge image_transport)
target_link_libraries(face_aim_node perception_qos ${OpenCV_LIBS})
```

安装目标添加 `face_aim_node`：

```cmake
install(TARGETS
  yolo_processing
  detection_node
  tracking_node
  face_aim_node           # ← 新增
  ...
```

### Step 4: （可选）集成到 `perception.launch.py`

在 launch 文件中增加新节点：

```python
face_aim_node = Node(
    package="perception_pkg",
    executable="face_aim_node",
    name="face_aim_node",
    output="screen",
    remappings=[
        ("image", LaunchConfiguration("sony_image_topic")),
        ("tracks", LaunchConfiguration("tracks_topic")),
        ("aim_target", "/perception/aim_target_2d"),
    ],
)
```

---

## 架构图

```
感知管线                          face_aim_node                   伺服控制
┌─────────────┐   /sony/image_raw   ┌────────────────┐   /perception/   ┌────────────────────┐
│ detection   │────────────────────→│                │   aim_target_2d  │ mvp_follow_        │
│ _node       │                     │  face_aim_node  │────────────────→│ controller_node    │
│ (YOLO)      │                     │                │                 │ (IBVS/PBVS)        │
└──────┬──────┘                     │  ┌──────────┐  │                 └────────────────────┘
       │  /perception/detections    │  │pending   │  │
       v                            │  │_image_   │  │
┌─────────────┐                     │  │          │  │
│ tracking    │  /perception/tracks │  │pending   │  │
│ _node       │────────────────────→│  │_tracks_  │  │
│ (ByteTrack) │                     │  └──────────┘  │
└─────────────┘                     └────────────────┘
```

---

## 测试方案

### 1. 编译验证

```bash
cd ~/programs/fcr_ros2_3
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select perception_pkg --symlink-install
```

确认无编译错误。

### 2. 单独启动测试（验证节点稳定性）

```bash
ros2 run perception_pkg face_aim_node
```

| 验证项 | 预期 |
|--------|------|
| 节点正常启动 | 日志输出 `face_aim_node started` |
| 无输入超时 | 几秒后 `/perception/aim_target_2d` 收到 `valid=false` |

```bash
# 另一终端查看输出
ros2 topic echo /perception/aim_target_2d
```

### 3. 模拟数据测试（验证功能逻辑）

手动发布匹配的图像和轨迹数据：

**终端 1：启动节点**
```bash
ros2 run perception_pkg face_aim_node
```

**终端 2：发布匹配的图像（注意 sec/nanosec 要与 tracks 匹配）**
```bash
ros2 topic pub --once /sony/image_raw sensor_msgs/msg/Image \
  '{header: {stamp: {sec: 100, nanosec: 0}, frame_id: "sony_optical_frame"},
    height: 480, width: 640, encoding: "bgr8", is_bigendian: false, step: 1920, data: []}'
```

**终端 3：发布匹配的 tracks（相同的 stamp）**
```bash
ros2 topic pub --once /perception/tracks vision_servo_msgs/msg/TargetArray \
  '{header: {stamp: {sec: 100, nanosec: 0}, frame_id: "sony_optical_frame"},
    targets: [{id: 1, class_name: "person", confidence: 0.9,
               center: [320.0, 240.0], bbox: [200.0, 150.0, 440.0, 330.0],
               tracking_state: 2, visible: true, height: 180, width: 240}],
    tracking_id: 1}'
```

**终端 4：观察输出**
```bash
ros2 topic echo /perception/aim_target_2d
```

预期输出：
```
header:
  stamp: {sec: 100, nanosec: 0}
  frame_id: "sony_optical_frame"
tracking_id: 1
pixel_x: 320.0
pixel_y: 240.0
confidence: 0.9
source: 3    # UPPER_BODY
valid: true
```

### 4. 测试场景矩阵

| # | 场景 | 操作 | 预期 |
|---|------|------|------|
| 1 | 正常匹配 | image 和 tracks 的 stamp 一致 | `valid=true`, pixel 为 bbox center |
| 2 | 无跟踪目标 | `tracking_id=-1` | `valid=false` |
| 3 | tracking_id 不在 targets 中 | 发布不存在的 ID | `valid=false` |
| 4 | 非 person 目标 | `class_name != "person"` | `valid=false` |
| 5 | 输入超时 | 停止发布数据 >2s | `valid=false` |
| 6 | 时间戳不匹配 | image 和 tracks stamp 不一致 | 等待，不输出 |
| 7 | 恢复匹配 | 不匹配后发送正确配对 | 恢复正常 `valid=true` |
| 8 | 调试图无订阅者 | 不启动 debug topic echo | 无 debug 图发布（日志可确认） |
| 9 | 调试图有订阅者 | `ros2 topic echo /perception/face_aim_debug` | debug 图正常发布 |

### 5. 集成测试（结合实际管线）

启动完整感知管线并加入 face_aim_node：

```bash
ros2 launch perception_pkg perception.launch.py
# 另一终端
ros2 run perception_pkg face_aim_node
# 观察瞄准目标
ros2 topic echo /perception/aim_target_2d
```

### 6. 性能影响测试

```bash
# 基准：不加 face_aim_node，测 tracks 频率
ros2 topic hz /perception/tracks

# 加 face_aim_node 后再次测量
ros2 topic hz /perception/tracks
```

预期：帧率差异 ≤ 1-2 FPS（节点仅做坐标传递和 stamp 匹配，无重计算）

### 7. 超时重连测试

```bash
# 发送一组匹配数据 → 看到 valid=true
# 停止发送 3 秒 → 看到 valid=false
# 重新发送匹配数据 → 恢复 valid=true
```

---

## 验收清单

| # | 项 | 标准 |
|---|-----|------|
| 1 | 编译 | `colcon build` 无警告/错误 |
| 2 | 空载运行 | 节点启动后无崩溃，超时发 `valid=false` |
| 3 | 正常匹配 | 匹配的 image+tracks → 正确的 AimTarget2D |
| 4 | 仅处理 person | 非 person 目标不产生输出 |
| 5 | 有界缓存 | 内存不随时间增长（`ps aux` 监控 RSS） |
| 6 | 调试图开关 | 无订阅者时不发布 debug 图 |
| 7 | 超时检测 | 超时后发送 `valid=false` |
| 8 | 性能影响 | `ros2 topic hz /perception/tracks` 下降 < 2 FPS |
| 9 | 不依赖人脸模型 | 用 bbox center 回退稳定运行 |

---

## 已知限制

- 当前使用 `UPPER_BODY` 作为 source 类型（bbox center 近似上半身中心），未来接入人脸模型后改为 `FACE`
- 时间戳匹配采用 1ms 容忍窗口，若管线存在大于 1ms 的时间戳抖动，需调整 `max_tolerance_ns_` 参数
- 节点不缓存历史帧，仅依赖最新待处理帧
