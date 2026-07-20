# 第二阶段：真实2D跟踪安全接入视觉伺服

本阶段直接复用 `perception_pkg` 发布的 `TargetArray`：

`/sony/image_raw → detection_node → tracking_node → /perception/tracks`

随后进入安全控制链：

`/perception/tracks → servo_manager(IBVS) → /auto/* → command_mux → 执行器`

## 安全边界

- 只接受 `visible=true` 且状态为 `CONFIRMED` 或 `UNTRACKED` 的目标；
- `LOST`、不可见和空目标不刷新超时，300 ms内触发停车；
- 无可靠深度时 `servo_allow_chassis_translation=false`，底盘线速度强制为零；
- 云台运动和底盘原地转向允许通过；
- 主启动的 `enable_servo`、`servo_auto_start` 默认仍为 `false`；
- `command_mux` 默认仍处于 `manual`，自动模式必须由操作者单独授权。

## Jetson自动验收

该测试使用纯2D目标（`position=0`、`depth_confidence=0`），并在目标消失后
持续发布 `LOST` 轨迹，以验证 LOST 数据不会让控制器继续运动。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select servo_control_pkg simulation_pkg bringup_pkg \
  --executor sequential
source install/setup.bash

ros2 launch bringup_pkg servo_2d_acceptance.launch.py
```

成功输出：

```text
PHASE2_ACCEPTANCE {"result":"PASS", ...}
```

验收器同时检查云台运动、底盘转向、三处线速度均为零、停车延迟小于
300 ms，以及 `/cmd_vel` 只有 `command_mux` 一个发布者。

## 真人检测观察（不启动控制）

终端1只启动相机，不启动任何执行器：

```bash
ros2 launch sony_camera_pkg sony_camera.launch.py \
  camera_info_url:=file:///home/nvidia/.ros/camera_info/sony_zv_e10_ii.yaml
```

终端2只启动检测与跟踪：

```bash
ros2 launch perception_pkg perception.launch.py \
  sony_image_topic:=/sony/image_raw \
  device:=cpu
```

确认：

```bash
ros2 topic hz /perception/tracks
ros2 topic echo /perception/tracks --once
```

只有检测频率、目标ID和边界框稳定后，才进入悬空低速控制验收。

## 实机控制分级

首次只允许云台，底盘保持静止：

```bash
ros2 launch bringup_pkg fcr_bringup.launch.py \
  use_sim:=false \
  enable_imu:=false \
  enable_servo:=true \
  servo_auto_start:=true \
  servo_allocation_ratio:=0.0 \
  servo_allow_chassis_translation:=false \
  sony_camera_info_url:=file:///home/nvidia/.ros/camera_info/sony_zv_e10_ii.yaml
```

确认云台方向正确后，悬空测试底盘转向时把
`servo_allocation_ratio` 调为 `0.5`。生产 `command_mux` 初始仍为 manual，
切换 auto 前应确认急停可用、底盘悬空且周围无人。
