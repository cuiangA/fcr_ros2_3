# MVP 分级真机测试模式

本方案使用同一个 `mvp_follow_controller_node`，通过独立参数文件逐级开放执行器：

| 模式 | 云台 | 底盘偏航 | 底盘平移 | 深度要求 |
|---|---:|---:|---:|---:|
| 感知观察 | 关闭 | 关闭 | 关闭 | 无 |
| 手动云台 | 键盘 | 关闭 | 关闭 | 无 |
| MVP纯云台 | 自动 | 关闭 | 关闭 | 无 |
| MVP协同偏航 | 自动 | 自动 | 强制关闭 | 无 |
| MVP完整跟随 | 自动 | 自动 | 深度门控 | 必须 |

所有自动模式只发布 `/auto/cmd_vel` 和 `/auto/cmd_gimbal`。最终执行话题
`/cmd_vel`、`/cmd_gimbal` 必须只有 `command_mux` 一个发布者。

## 构建与自动验收

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select servo_control_pkg simulation_pkg bringup_pkg \
  --executor sequential
source install/setup.bash

ros2 launch bringup_pkg mvp_gimbal_acceptance.launch.py
ros2 launch bringup_pkg mvp_cooperative_yaw_acceptance.launch.py
ros2 launch bringup_pkg mvp_full_follow_acceptance.launch.py
```

每条测试都必须输出：

```text
MVP_MODE_ACCEPTANCE {"result":"PASS", ...}
```

## 模式0：只读感知观察

该模式不启动平台驱动、控制器或安全仲裁，不会产生运动指令：

```bash
ros2 launch bringup_pkg fcr_perception_observe.launch.py
```

Foxglove连接 `ws://ROBOT_IP:8765`，图像选择
`/perception/tracking_image/compressed`。

## 模式1：手动云台

```bash
sudo ip link set can1 up
ros2 launch bringup_pkg fcr_gimbal_manual_test.launch.py can_interface:=can1
```

另一个交互终端：

```bash
ros2 run teleop_control_pkg keyboard_platform_teleop
```

使用方向键确认云台方向、失联停车和急停。不要启动第二个键盘节点。

## 模式2：纯云台自动跟踪

底盘必须悬空，物理断电必须可用：

```bash
ros2 launch bringup_pkg fcr_mvp_gimbal_follow.launch.py \
  can_interface:=can1
```

该launch不会创建底盘驱动或里程计节点。启动键盘节点，确认检测框稳定后按
`O`。本模式 `/cmd_vel` 的线速度和角速度必须始终为零。目标丢失后应在
300ms内保持云台。

## 模式3：云台与底盘协同偏航

只有模式2通过后才能运行，并且首次必须保持底盘悬空：

启动前先用手动模式把云台物理朝向调整为底盘正前方，然后停止手动模式。协同控制器
会把收到的第一帧有效 RS2 yaw 记录为相对零位；这是必须步骤，因为 RS2 编码器的
绝对零点不等于底盘正前方，且角度可能在 `-pi/pi` 附近回绕。

```bash
ros2 launch bringup_pkg fcr_mvp_cooperative_yaw.launch.py \
  can_interface:=can1
```

允许 `angular.z`，但所有 `linear.*` 必须为零。
启动日志必须出现 `Captured gimbal forward reference`。云台向右偏离相对零位时，
底盘应输出负 `angular.z` 右转；云台回到相对零位死区后，底盘角速度应回到零。

## 模式4：完整深度跟随

```bash
ros2 launch bringup_pkg fcr_mvp_full_follow.launch.py \
  can_interface:=can1
```

该模式默认订阅 `/perception/targets_3d`。只有当目标深度位于0.3到10米之间且
`depth_confidence >= 0.6` 时才允许底盘平移。当前 `/perception/tracks` 的
`depth_confidence=0`，不能用于本模式的真实平移验收。

## 通用安全检查

进入AUTO前执行：

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_gimbal -v
ros2 topic echo /remote_control/status --once
```

两条最终指令话题都必须只有 `command_mux` 一个发布者。按 `P` 进入安全停止，
按 `X` 锁存软件急停；软件急停不能替代物理断电。

## 二代改进观察

本轮分级测试只记录优化诉求，不在测试中途连续修改算法。人脸构图、云台响应速度
以及后续测试发现的问题统一记录在
[MVP 二代改进观察日志](mvp_v2_improvement_log.md)，待模式2到模式4完成后集中设计、
实现和回归。
