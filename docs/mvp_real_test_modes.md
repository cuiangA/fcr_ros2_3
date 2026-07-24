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

## 固定云台控制策略

当前真机策略固定为两条彼此独立的控制链路，禁止混用：

| 使用场景 | 输入话题 | 最终话题 | RS2执行方式 |
|---|---|---|---|
| 人工方向键 | `/teleop/gimbal_nudge` | `/cmd_gimbal_nudge` | 单次增量位置 |
| MVP AUTO | `/auto/cmd_gimbal` | `/cmd_gimbal` | 连续速度控制 |

- 方向键只产生 `GimbalNudge`，每次按键执行一个有界角度增量。
- MVP AUTO只产生 `GimbalCmd`，由`command_mux`以连续速度指令输出。
- `fcr_mvp_*`真机模式默认使用`gimbal_control_mode:=speed`和
  `gimbal_speed_control_byte:=128`。
- 普通平台bringup仍保留运行时参数覆盖，便于独立硬件调试；不得因为人工
  nudge可动或单次隔离测试结果，擅自把AUTO链路改为nudge。
- AUTO验收期间禁止按方向键，否则人工nudge会污染自动跟踪结论。

启动MVP后必须确认实际参数：

```bash
ros2 param get /gimbal_driver control_mode
ros2 param get /gimbal_driver speed_control_byte
```

当前预期为：

```text
String value is: speed
Integer value is: 128
```

## 每次USB-CAN插拔后的强制初始化

USB-CAN每次拔插、Jetson重启、CAN出现`BUS-OFF`，或者内核出现`echo id`
异常后，都必须重新执行本节。Linux分配的`can0/can1`编号不固定，禁止沿用上次
接口编号。

### 1. 停止所有控制节点

先在对应终端用`Ctrl-C`停止launch和键盘节点，然后检查：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 node list | grep -E \
  "gimbal_driver|command_mux|mvp_follow_controller_node|keyboard_platform_teleop"
```

初始化CAN时不应存在上述节点。不要在云台驱动发送数据时重配接口。

### 2. 识别USB-CAN，不依赖接口编号

```bash
ip -br link | grep can || true

for IFACE_PATH in /sys/class/net/can*; do
  test -e "$IFACE_PATH" || continue
  NAME=$(basename "$IFACE_PATH")
  echo "=== $NAME ==="
  ethtool -i "$NAME" 2>/dev/null | grep -E "driver|bus-info"
done
```

云台USB-CAN必须显示：

```text
driver: gs_usb
```

Jetson板载CAN通常显示`driver: mttcan`，除非云台物理接入板载CAN收发器，否则
不得拿它代替`gs_usb`。

### 3. 自动配置实际的gs_usb接口

推荐不指定编号，让脚本自动识别：

```bash
cd ~/ros2_ws/src/fcr_ros2_3
chmod +x tools/can/setup_gimbal_can.sh
./tools/can/setup_gimbal_can.sh
```

成功输出示例：

```text
GIMBAL_CAN_INTERFACE=can0
```

立即按实际结果设置本终端变量：

```bash
export CAN_IF=can0
```

如果有多个`gs_usb`适配器，才显式指定：

```bash
./tools/can/setup_gimbal_can.sh --interface can0
export CAN_IF=can0
```

### 4. 初始化后的硬门禁

```bash
test -n "${CAN_IF:-}" || { echo "CAN_IF未设置"; false; }
ethtool -i "$CAN_IF" | grep -E "driver|bus-info"
ip -details -statistics link show "$CAN_IF"
```

必须同时满足：

```text
driver: gs_usb
接口状态: UP
can state ERROR-ACTIVE
bitrate 1000000
restart-ms 100
qlen 100
```

记录测试前的`bus-errors`、`bus-off`、TX/RX和dropped计数，测试后再次比较。

### 5. 掉线或echo异常恢复

以下任一现象出现时禁止启动AUTO：

```text
RTNETLINK answers: No such device
Network is down
No buffer space available
can_put_echo_skb ... occupied
Unexpected unused echo id
Unexpected out of range echo id
BUS-OFF
```

恢复流程：

1. 停止全部ROS控制节点。
2. 拔下USB-CAN，等待5秒，再重新插入；优先直连Jetson或使用独立供电Hub。
3. 执行`sudo udevadm settle && sleep 2`。
4. 重复本节第2～4步，接口编号可能已经变化。
5. 若仍无`gs_usb`接口，再执行：

```bash
sudo modprobe -r gs_usb
sudo modprobe gs_usb
sudo udevadm settle
sleep 2
```

重新插拔后不能只执行`ip link set canX up`，必须重新设置bitrate、restart-ms和
txqueuelen。

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
ros2 launch bringup_pkg fcr_gimbal_manual_test.launch.py \
  can_interface:="$CAN_IF"
```

另一个交互终端：

```bash
ros2 run teleop_control_pkg keyboard_platform_teleop
```

使用方向键确认云台方向、失联停车和急停。不要启动第二个键盘节点。
方向键验收观察的是`/cmd_gimbal_nudge`，不能用它证明或否定AUTO速度链路。

## 模式2：纯云台自动跟踪

底盘必须悬空，物理断电必须可用，并且已经完成“每次USB-CAN插拔后的强制
初始化”。关闭所有旧launch后启动：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bringup_pkg fcr_mvp_gimbal_follow.launch.py \
  can_interface:="$CAN_IF" \
  enable_camera_motion_compensation:=false
```

该launch不会创建底盘驱动或里程计节点。启动键盘节点，确认检测框稳定后按
`O`。本模式 `/cmd_vel` 的线速度和角速度必须始终为零。目标丢失后应在
300ms内保持云台。

本模式的正式链路必须为：

```text
mvp_follow_controller_node
  -> /auto/cmd_gimbal
  -> command_mux
  -> /cmd_gimbal
  -> gimbal_driver(speed)
```

进入AUTO后不要按方向键。

### 模式2启动检查

```bash
ros2 lifecycle get /gimbal_driver
ros2 param get /gimbal_driver can_interface
ros2 param get /gimbal_driver control_mode
ros2 param get /gimbal_driver speed_control_byte
ros2 topic echo /perception/tracks --once
ros2 topic info /cmd_gimbal -v
ros2 topic info /cmd_vel -v
```

必须满足：驱动为`active`、接口等于`$CAN_IF`、模式为`speed`、控制字为`128`；
真人目标必须包含`class_name: person`和`visible: true`；最终指令话题都只能由
`command_mux`发布。如果日志持续显示`target=0`，保持MANUAL并先修复目标输入。

### 模式2进入AUTO与监控

另开交互终端：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_control_pkg keyboard_platform_teleop
```

按`O`后确认：

```bash
ros2 topic echo /remote_control/status --once
```

必须包含`"mode":"auto"`、`"active_source":"auto"`和`"estop":false`。
分别在终端持续观察：

```bash
ros2 topic echo /auto/cmd_gimbal
ros2 topic echo /cmd_gimbal
ros2 topic echo /gimbal/status
```

### 模式2动作序列

1. 人在中心停2秒，云台应基本静止。
2. 人移到右侧停3秒，人物框应向画面中心收敛。
3. 缓慢穿过中心到左侧停3秒，`yaw_rate`应变号。
4. 左右往返至少5次，不得变向卡住。
5. 人脸分别移动到画面上方和下方，各重复3次，`pitch_rate`应变号。
6. 人快速离开或遮挡镜头；约300ms内最终指令必须变为零且`hold=true`。
7. 按`P`安全停车；异常时按`X`并准备物理断电。

通过条件：方向正确、框趋向中心、左右切换不卡住、目标丢失及时停车、底盘无
动作，并且CAN错误计数不升级。

## 模式3：云台与底盘协同偏航

只有模式2通过后才能运行，并且首次必须保持底盘悬空：

启动前先用手动模式把云台物理朝向调整为底盘正前方，然后停止手动模式。协同控制器
会把收到的第一帧有效 RS2 yaw 记录为相对零位；这是必须步骤，因为 RS2 编码器的
绝对零点不等于底盘正前方，且角度可能在 `-pi/pi` 附近回绕。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch bringup_pkg fcr_mvp_cooperative_yaw.launch.py \
  can_interface:="$CAN_IF" \
  enable_camera_motion_compensation:=false
```

允许 `angular.z`，但所有 `linear.*` 必须为零。
启动日志必须出现 `Captured gimbal forward reference`。云台向右偏离相对零位时，
底盘应输出负 `angular.z` 右转；云台回到相对零位死区后，底盘角速度应回到零。

### 模式3启动门禁

- 底盘轮子必须悬空。
- 模式2必须全部通过。
- 启动前用手动nudge把云台对准底盘正前方，然后关闭手动launch。
- 当前进程日志必须出现`Captured gimbal forward reference`。
- `/cmd_vel`和`/cmd_gimbal`都必须只有`command_mux`一个发布者。
- `/perception/tracks`必须有可见真人目标。

启动键盘后按`O`，AUTO期间不要按方向键。分别监控：

```bash
ros2 topic echo /auto/cmd_gimbal
ros2 topic echo /cmd_gimbal
ros2 topic echo /auto/cmd_vel
ros2 topic echo /cmd_vel
```

所有`/cmd_vel`必须始终满足`linear.x/y/z=0`、`angular.x/y=0`，只允许
`angular.z`非零。发现任何平移立即按`P`并判定失败。

### 模式3动作序列

1. 人在中心停2秒，云台和底盘应基本静止。
2. 人到右侧停4秒：云台先跟踪，产生相对偏航后底盘开始原地旋转。
3. 人穿过中心到左侧停4秒：云台和底盘指令应在需要时变号。
4. 再回右侧，左右往返3～5次；底盘不得沿旧方向持续旋转。
5. 人离开画面：约300ms内云台指令和`angular.z`都必须回零。
6. 按`P`停车，确认轮子和云台停止。

正式判断使用同步录包：

```bash
rm -rf /tmp/mvp_cooperative_test
ros2 bag record -o /tmp/mvp_cooperative_test \
  /perception/tracks \
  /auto/cmd_gimbal /cmd_gimbal \
  /gimbal/status /platform/state \
  /auto/cmd_vel /cmd_vel \
  /remote_control/status
```

录制中心、右、左、右、目标离开和按`P`的完整序列后，用`Ctrl-C`停止并执行：

```bash
ros2 bag info /tmp/mvp_cooperative_test
ip -details -statistics link show "$CAN_IF"
```

模式3通过条件：云台和底盘方向正确、及时变向、底盘始终零平移、目标丢失及时
停车、发布者唯一，且CAN没有`Network is down`、队列耗尽或`BUS-OFF`。

## 模式4：完整深度跟随

```bash
ros2 launch bringup_pkg fcr_mvp_full_follow.launch.py \
  can_interface:="$CAN_IF"
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
