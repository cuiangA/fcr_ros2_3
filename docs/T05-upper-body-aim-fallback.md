# T05：人体上部瞄准回退

## 目标

在接入人脸模型前，先打通完整控制链。将云台瞄准点从人体检测框几何中心改为上身区域（头部附近），并补齐 LOST 预测、低通滤波、图像边界限制等能力。

## 改动文件

| 文件 | 改动类型 |
|------|----------|
| `src/perception_pkg/include/perception_pkg/face_aim_node.hpp` | 新增成员变量 |
| `src/perception_pkg/src/face_aim_node.cpp` | 核心逻辑修改 |

## 实现步骤

### 步骤 1：新增 ROS2 参数

在构造函数中新增两个参数声明和读取：

```cpp
// face_aim_node.cpp:18-19, 25-26
declare_parameter("aim_offset_ratio", 0.20);  // 瞄准点 Y = bbox_y_min + ratio * height
declare_parameter("lpf_alpha", 0.40);          // 一阶低通滤波系数

aim_offset_ratio_ = get_parameter("aim_offset_ratio").as_double();
lpf_alpha_ = get_parameter("lpf_alpha").as_double();
```

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `aim_offset_ratio` | double | 0.20 | 瞄准点 Y 偏移比率：`y = bbox[1] + ratio × height` |
| `lpf_alpha` | double | 0.40 | 低通滤波系数，0~1。越小越平滑（响应越慢） |

### 步骤 2：新增成员变量

```cpp
// face_aim_node.hpp:54-60
double aim_offset_ratio_ = 0.20;
double lpf_alpha_ = 0.40;
float filtered_x_ = 0.0f;          // 滤波后 X
float filtered_y_ = 0.0f;          // 滤波后 Y
bool filter_initialized_ = false;  // 滤波器是否已初始化
int last_tracking_id_ = -1;        // 上一帧 tracking_id，用于 ID 变化时重置滤波
```

### 步骤 3：上身瞄准点计算 + LOST 预测 + 低通滤波 + 图像边界限制

修改 `try_match_and_process()` 的核心逻辑：

```cpp
// face_aim_node.cpp:107-161
// 获取图像尺寸用于边界限制
const float image_width = static_cast<float>(pending_image_->width);
const float image_height = static_cast<float>(pending_image_->height);

if (tracks.tracking_id >= 0) {
    const auto it = std::find_if(...);

    if (it != tracks.targets.end()) {
        // 目标存在（无论 visible 与否）
        aim.tracking_id = it->id;

        // 根据可见性设置来源类型
        if (it->visible) {
            aim.source = UPPER_BODY;        // 绿色，可见目标
        } else {
            aim.source = LOST_PREDICTION;   // 橙色，ByteTrack 卡尔曼预测
        }

        // 上身瞄准点：X 同中心，Y 偏移到上身
        const float raw_x = it->center[0];
        const float raw_y = it->bbox[1] + aim_offset_ratio_ * it->height;

        // 一阶低通滤波
        if (!filter_initialized_ || it->id != last_tracking_id_) {
            filtered_x_ = raw_x;
            filtered_y_ = raw_y;
            filter_initialized_ = true;
        } else {
            filtered_x_ = lpf_alpha * raw_x + (1 - lpf_alpha) * filtered_x_;
            filtered_y_ = lpf_alpha * raw_y + (1 - lpf_alpha) * filtered_y_;
        }
        last_tracking_id_ = it->id;

        // 限制在图像范围内
        aim.pixel_x = clamp(filtered_x_, 0, image_width);
        aim.pixel_y = clamp(filtered_y_, 0, image_height);
        aim.valid = true;
    } else {
        // 轨迹已删除（标志为 Removed），重置滤波
        filter_initialized_ = false;
        last_tracking_id_ = -1;
    }
}
```

### 步骤 4：超时进入 LOST 时重置滤波

```cpp
// face_aim_node.cpp:169-170
filter_initialized_ = false;
last_tracking_id_ = -1;
```

### 步骤 5：可视化区分 UPPER_BODY / LOST_PREDICTION

调试图像中：
- **UPPER_BODY（可见）**：绿色圆圈 + 绿色文字
- **LOST_PREDICTION（丢失预测）**：橙色圆圈 + 橙色文字

## 控制链数据流变更

```
                  face_aim_node (改前)
TrackingNode ──→  bbox 几何中心 ──→ 下游（未接入）
                  aim = center_x, center_y

                  face_aim_node (改后)
TrackingNode ──→  上身瞄准点 + LPF + 边界限制 ──→ 下游（待接入）
                  aim_x = center_x
                  aim_y = bbox[1] + 0.20 × height
                  source = UPPER_BODY / LOST_PREDICTION
```

## 完成标准

| # | 标准 | 验证方式 |
|---|------|----------|
| 1 | 云台瞄准点从人体中心变为头部附近 | 调试可视化看到绿点在人头区域（比中心上移约 30% 身高） |
| 2 | 瞄准点被限制在图像边界内 | `pixel_x ∈ [0, width]`, `pixel_y ∈ [0, height]` |
| 3 | LOST 时输出 ByteTrack 预测位置 | `topic echo` 看到 `valid=true, source=4(LOST_PREDICTION)` |
| 4 | 轨迹删除后（>2.5s 超时）输出无效点 | `topic echo` 看到 `valid=false` |
| 5 | 低通滤波生效，像素值变化平滑 | 连续观察 `pixel_x/pixel_y` 无跳变 |
| 6 | 全程保持原人体 ID | `tracking_id` 在 UPPER_BODY → LOST_PREDICTION 切换前后不变 |

## 测试方案

### 前置准备

已录好视频 bag（171 帧，9.6s，1254×720）：

```
/tmp/opencode/v2_bag              # 含 /sony/image_raw 话题
```

`video_publisher.py` 脚本已安装，也可直接发布视频文件为图像话题：

```bash
ros2 run perception_pkg video_publisher \
    --path src/perception_pkg/video/v2.mp4
```

### 测试 1：完整感知管线测试（推荐）

face_aim_node 需要同时接收 `/sony/image_raw` 和 `/perception/tracks`，所以必须启动完整的检测+跟踪流水线。

**方式 A：使用 video_publisher + 实时感知**

```bash
# 终端 1：发布视频帧作为图像源
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run perception_pkg video_publisher \
    --path src/perception_pkg/video/v2.mp4

# 终端 2：启动完整感知管线（检测 + 跟踪 + face_aim）
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch perception_pkg perception.launch.py \
    sony_image_topic:=/sony/image_raw

# 终端 3：可视化观察
ros2 run rqt_image_view rqt_image_view /face_aim_debug
```

**方式 B：使用 rosbag 回放**

```bash
# 终端 1：回放视频 bag
source /opt/ros/humble/setup.bash
ros2 bag play /tmp/opencode/v2_bag --loop

# 终端 2：启动感知管线
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch perception_pkg perception.launch.py
```

### 测试 2：可视化观察

```bash
ros2 run rqt_image_view rqt_image_view /face_aim_debug
```

观察绿色瞄准点是否在人体头部附近而非几何中心（相比人体中心上移约 30% 身高）。
LOST 时点变为橙色。

### 测试 3：CLI 消息检查

```bash
# 实时查看瞄准点消息
ros2 topic echo /perception/aim_target_2d

# 预期输出（可见时）：
#   valid: true
#   source: 3  (UPPER_BODY)
#   pixel_x: ...  (中心 X)
#   pixel_y: ...  (比 center_y 偏上 ~30% 身高)

# 预期输出（LOST 时，轨迹未超时）：
#   valid: true
#   source: 4  (LOST_PREDICTION)
#   tracking_id: 保持不变
#   pixel_x/pixel_y 为 ByteTrack 卡尔曼预测位置

# 预期输出（轨迹已删除）：
#   valid: false
```

### 测试 4：人体走出画面

1. 人体停留在画面中央 → 确认绿点在头部附近（UPPER_BODY，source=3）
2. 人体快速走出画面 → 确认切换到橙色点（LOST_PREDICTION，source=4），`tracking_id` 不变
3. 等待 2.5 秒（ByteTrack lost_timeout）→ 确认 `valid=false`
4. 人体回到画面 → 确认重新锁定，绿点回到头部附近（`tracking_id` 可能变化，ByteTrack 重新分配 ID）
