# 第一阶段：伺服到底盘控制链路收口

本阶段验证以下真实软件链路，底盘后端使用 `use_sim=true`，不会驱动实车：

`/perception/targets_3d → servo_manager → /auto/cmd_vel → command_mux → /cmd_vel → chassis_driver`

## Jetson 验收

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to bringup_pkg simulation_pkg servo_control_pkg \
  teleop_control_pkg robot_platform_pkg
source install/setup.bash

ros2 launch bringup_pkg servo_chain_acceptance.launch.py
```

启动文件会自动发布 Sony 相机格式的 `CameraInfo` 和一个持续 3 秒的 3D
目标，并检查：

- `servo_manager` 产生非零 `/auto/cmd_vel`；
- `command_mux` 将其送到最终 `/cmd_vel`；
- 仿真 `chassis_driver` 的 `/chassis/odom_raw` 出现运动反馈；
- 目标停止发布后，最终 `/cmd_vel` 在 300 ms 内归零；
- `/cmd_vel` 只有一个发布者，且发布节点是 `command_mux`。

成功时终端会出现：

```text
PHASE1_ACCEPTANCE {"result": "PASS", ...}
```

该启动文件在验收完成后自动退出。若出现 `FAIL`，同一行 JSON 会给出缺失的
链路证据、停车延迟和 `/cmd_vel` 发布者信息。

## 生产启动参数

生产主启动仍保持安全默认值：`enable_servo:=false`、
`servo_auto_start:=false`。在 3D 目标源完成验收后，才显式启用：

```bash
ros2 launch bringup_pkg fcr_bringup.launch.py \
  use_sim:=false \
  enable_servo:=true \
  servo_auto_start:=true \
  servo_camera_info_topic:=/sony/camera_info \
  servo_target_timeout:=0.25
```

注意：生产 `command_mux` 默认仍为 `manual`。切换到自动模式属于独立的人工
授权动作，避免系统仅因收到目标就直接驱动底盘。
