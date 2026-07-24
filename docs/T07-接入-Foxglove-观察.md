# T07：接入 Foxglove 观察

## 目标

在 Foxglove 远程监控的标注图 `/perception/tracking_image/compressed` 上绘制完整的瞄准和锁定视觉信息，使操作员能直观回答：

| 问题 | 对应可视化 |
|------|-----------|
| 机器人跟踪的是谁？ | 人体框 + 加粗 ID 标签 + 锁定状态 |
| 镜头当前瞄准哪里？ | 图像中心十字 + 实际瞄准点十字 + 中心→瞄准点连线 |
| 为什么使用这个瞄准点？ | 瞄准来源标签 + 置信度 + 延迟 + 状态机状态 |

## 当前状态

`perception_visualizer_node.cpp` 现有能力：

| 功能 | 状态 |
|------|------|
| 检测框（绿色细线） | ✅ |
| 跟踪框（颜色按 state 编码，TARGET 洋红粗线，LOST 虚线） | ✅ |
| 跟踪框标签 `TARGET ID:X class STATE conf` | ✅ |
| 顶部状态面板（FPS、延迟、组件健康） | ✅ |
| 瞄准点十字 | ❌ |
| 图像中心十字 | ❌ |
| 瞄准来源/置信度/延迟显示 | ❌ |
| 锁定状态机状态独立显示 | ❌ |
| 人脸/关键点/上身瞄准点可视化 | ❌ |

## 涉及文件

| 文件 | 操作 |
|------|------|
| `src/remote_monitor_pkg/src/perception_visualizer_node.cpp` | **主要修改** |
| `src/remote_monitor_pkg/launch/remote_monitor.launch.py` | 添加参数传递 |
| `src/bringup_pkg/launch/fcr_perception_observe.launch.py` | 如需要继承参数 |
| `src/bringup_pkg/launch/fcr_bringup.launch.py` | 如需要继承参数 |
| `src/bringup_pkg/launch/fcr_mvp_mode.launch.py` | 如需要继承参数 |

## 数据流

```
face_aim_node
  └─ /perception/aim_target_2d  (AimTarget2D)
       ├─ tracking_id, pixel_x, pixel_y
       ├─ source: FACE=0 / KEYPOINT=1 / PREDICTED_FACE=2 / UPPER_BODY=3 / LOST_PREDICTION=4
       ├─ confidence [0,1]
       └─ valid: bool

servo_manager_node (可选)
  └─ /servo/state  (ServoState)
       ├─ state: IDLE=0 / CONVERGING=1 / TRACKING=2 / LOST=3 / ERROR=4
       └─ norm_error, camera_velocity, etc.

perception_visualizer_node  ← 订阅以上话题
  └─ /perception/tracking_image/compressed  → Foxglove
```

## 实现步骤

### Step 1：添加 AimTarget2D 话题订阅

在 `perception_visualizer_node.cpp`：

- 包含 `vision_servo_msgs/msg/aim_target2_d.hpp`
- 添加成员变量 `latest_aim_target_`（`std::optional<AimTarget2D>`）和 `aim_target_last_update_`
- 在构造函数中创建订阅器 `aim_target_sub_`
- 实现回调 `aim_target_callback()`：缓存最新消息和时间戳

### Step 2：添加 ServoState 可选订阅

- 包含 `vision_servo_msgs/msg/servo_state.hpp`
- 在 `enable_future_inputs_` 条件块下添加订阅 `/servo/state`
- 缓存 `servo_state_`, `servo_state_last_update_`

### Step 3：实现绘制函数

| 函数 | 功能 |
|------|------|
| `draw_center_crosshair()` | 图像几何中心处绘制小十字（浅蓝/青色），半透明 |
| `draw_aim_crosshair()` | 瞄准点处绘制大十字（颜色按来源编码），中心填充 |
| `draw_center_to_aim_line()` | 从图像中心到瞄准点的连线（白色虚线或半透明实线） |
| `draw_aim_info()` | 瞄准点旁显示 `<来源名> ID:<id> conf:<置信度>` 标签 |
| `draw_lock_state_badge()` | 跟踪目标框上方绘制大号 `TARGET ID:X` 及状态徽标 |

瞄准点十字颜色约定：

| 来源 | 颜色 |
|------|------|
| FACE (0) | 绿色 `(0,255,0)` |
| KEYPOINT (1) | 蓝色 `(255,0,0)` |
| PREDICTED_FACE (2) | 青黄 `(0,255,255)` |
| UPPER_BODY (3) | 橙色 `(0,165,255)` |
| LOST_PREDICTION (4) | 红色 `(0,0,200)` |

### Step 4：在 render_and_publish 中集成

在 `draw_tracks_` 绘制块之后、`draw_status_overlay_` 之前（或之后）插入：

```
draw_center_crosshair(canvas);
if (latest_aim_target_.valid) {
    draw_center_to_aim_line(canvas, center, aim_pt);
    draw_aim_crosshair(canvas, aim);
    draw_aim_info(canvas, aim, aim_latency_ms);
}
```

### Step 5：更新 launch 文件参数

在 `remote_monitor.launch.py` 中添加 `aim_target_topic` 参数，默认值 `/perception/aim_target_2d`，传递给 `perception_visualizer_node`。

### Step 6：添加参数开关

在 `declare_parameters()` 中添加：

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `draw_center_crosshair` | bool | true | 绘制图像中心十字 |
| `draw_aim_crosshair` | bool | true | 绘制瞄准点十字 |
| `draw_aim_info` | bool | true | 绘制瞄准信息标签 |

## 渲染效果示意

```
┌─────────────────────────────────────────────┐
│ [状态面板: Camera OK YOLO OK Tracker OK ...] │
│                                               │
│      ┌─────────────────┐       ·←中心十字      │
│      │ TARGET ID:3     │    ╲ │ ╱              │
│      │ CONFIRMED   ←锁定状态徽标                │
│      │ [ 人体跟踪框 ]  │    ╲ │ ╱  ←中心→瞄准点│
│      │   (洋红粗线)    │     ╲│╱    连线        │
│      │                │      ○ ←瞄准点十字     │
│      └─────────────────┘     UPPER_BODY 0.85   │
│                                     ↑来源/置信度 │
│                                   延迟:12.3ms   │
└─────────────────────────────────────────────┘
```

## 测试方法

### 方案 A：无硬件独立测试（推荐开发环境使用）

不依赖任何真实硬件、YOLO 模型或索尼相机。使用一个自包含的 mock 发布器 `t07_mock_test.py`，它会生成：
- 合成测试图像（640×480，含模拟人体图案）
- 模拟跟踪数据（TargetArray，person CONFIRMED，位置左右摆动）
- 模拟瞄准数据（AimTarget2D，来源自动轮换：UPPER_BODY → FACE → LOST_PREDICTION）
- CameraInfo

```bash
# 编译（确保新脚本已安装）
colcon build --packages-select simulation_pkg remote_monitor_pkg

# 终端 1：启动 visualizer 节点（单独运行，不依赖 perception 管线）
source install/setup.bash
ros2 run remote_monitor_pkg perception_visualizer_node --ros-args \
  -r image:=/test/image_raw \
  -r tracks:=/test/tracks \
  -r aim_target:=/test/aim_target_2d \
  -p remote_max_width:=640 \
  -p draw_detections:=false \
  -p draw_status_overlay:=true

# 终端 2：启动 mock 数据发布器
source install/setup.bash
ros2 run simulation_pkg t07_mock_test.py

# 终端 3：查看输出话题
source install/setup.bash
ros2 topic echo /perception/tracking_image/compressed --once --no-arr | head -5

# （可选）启动 Foxglove bridge 查看实时画面
ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p address:=0.0.0.0 -p port:=8765
# Foxglove Desktop → 连接 ws://localhost:8765 → 打开 /perception/tracking_image/compressed
```

输出标注图每隔几秒自动切换瞄准阶段，可直观验证全部可视化元素。

### 方案 B：rosbag + 完整感知管线（需 YOLO 模型）

```bash
# 终端 1：回放已有 rosbag（含索尼图像）
ros2 bag play bags/v2_bag --clock --loop

# 终端 2：启动感知观察模式（检测+跟踪+face_aim+visualizer+foxglove）
ros2 launch bringup_pkg fcr_perception_observe.launch.py use_sim_time:=true
```

### 方案 C：rosbag + 仅 visualizer（不需 YOLO 模型）

```bash
# 终端 1：回放 rosbag 提供图像
ros2 bag play bags/v2_bag --clock --loop

# 终端 2：启动 mock 跟踪和瞄准数据（时间戳与 bag 不匹配，不渲染标注图）
# 此方案仅验证 visualizer 节点启动正常，不验证绘制效果
ros2 run remote_monitor_pkg perception_visualizer_node --ros-args \
  -r image:=/sony/image_raw \
  -p draw_detections:=false
```

### Foxglove 验证

1. 打开 Foxglove Desktop/Web
2. 连接 `ws://localhost:8765`
3. 打开面板布局（导入 `fcr_remote_monitor_layout.json`）
4. 观察 `/perception/tracking_image/compressed` 面板，验证：
   - ✅ 图像正中有不遮挡内容的细十字线
   - ✅ 跟踪目标框为粗洋红色，上方有大号 TARGET ID 和状态标签
   - ✅ 瞄准点十字颜色随来源变化
   - ✅ 瞄准点与中心之间有连线
   - ✅ 瞄准点旁显示来源名 + 置信度 + 延迟
   - ✅ 状态面板中有瞄准延迟数据
   - ✅ 当 aim_target.valid=false 时不显示瞄准十字

### 验收清单

| # | 检查项 | 预期 |
|---|--------|------|
| 1 | 有人体目标时 | 显示洋红色粗框，上方有 TARGET ID:X + CONFIRMED |
| 2 | 无人目标时 | 显示其他跟踪框（颜色按状态），无 TARGET 标签 |
| 3 | 目标跟踪丢失时 | 框变灰色虚线，显示 LOST |
| 4 | 有瞄准数据时 | 显示彩色瞄准十字 + 来源标签 + 置信度 + 延迟 |
| 5 | 瞄准数据无效时 | 不显示瞄准十字 |
| 6 | 图像中心始终 | 显示青色小十字 |
| 7 | 瞄准来源轮换 | mock 脚本自动切换 UPPER_BODY / FACE / LOST_PRED，十字颜色对应变化 |
