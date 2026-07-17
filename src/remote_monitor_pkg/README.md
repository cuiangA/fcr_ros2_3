# remote_monitor_pkg

面向 ROS 2 Humble 实机的只读远程观察层。该包不修改检测、跟踪或控制状态，
只订阅现有数据并发布带标注图像与结构化监控消息。

## 数据流

```text
/sony/image_raw -----------+
/perception/detections ----+--> perception_visualizer_node
/perception/tracks --------+        |--> /perception/tracking_image/compressed
/diagnostics --------------+        `--> /perception/monitor_status

ROS graph --> foxglove_bridge:8765 --> Foxglove Desktop / Web
```

可视化节点使用原始图像时间戳对图像、检测和轨迹做精确匹配，并在轨迹消息到达后
绘制，因此输出继续保留 Sony 图像的 `stamp` 和 `frame_id`。无线网络应在 Foxglove
Image panel 选择 `/perception/tracking_image/compressed`。节点直接发布 JPEG
`sensor_msgs/CompressedImage`，不再发布远程原始图，也不经过
`compressed_image_transport` 的二次队列。

轨迹回调只覆盖一个容量为 1 的“最新帧邮箱”，10 Hz 定时器只取当时最新帧；旧帧
不会排队等待绘制。默认输出宽度 960、JPEG 质量 65，源时间戳年龄超过 300 ms 的帧
在编码前后都会直接丢弃，发布端 QoS 采用 best-effort、keep-last(1)、300 ms
lifespan。该限制只作用于观察图；Sony、YOLO、Tracker 仍按各自完整帧率运行。

相关参数为 `remote_publish_rate_hz`、`remote_max_width`、`jpeg_quality` 和
`max_frame_age_ms`。如果无线网络仍不足，优先把输出降到 3 Hz / 480 px；不要提高
队列深度。

`inference_time_ms_estimate` 是图像采集时间到检测结果到达观察节点的链路测量，
不是 TensorRT 内核 profiler 数据。这样可以保持观察层与 YOLO 实现完全解耦。

## 安装与构建

```bash
sudo apt update
sudo apt install \
  ros-humble-foxglove-bridge

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-up-to remote_monitor_pkg \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DOpenCV_DIR=/usr/lib/cmake/opencv4
source install/setup.bash
```

Jetson 使用工作空间内基于 OpenCV 4.8 重编的 `cv_bridge` 时，必须在构建本包前
先 `source ~/ros2_ws/install/setup.bash`。

## 启动

已有 Sony + YOLO + Tracker 链运行时：

```bash
ros2 launch remote_monitor_pkg remote_monitor.launch.py \
  model_path:=$HOME/fcr_models/yolov8n_fp16.engine \
  inference_backend:=tensorrt
```

服务器默认监听 `0.0.0.0:8765`，但只开放监控白名单和 Connection Graph；远程
客户端不能通过该桥发布控制话题、调用服务、修改参数或读取任意文件。

开发电脑与机器人处于同一局域网时，在 Foxglove 选择 **Foxglove WebSocket**，
连接：

```text
ws://ROBOT_IP:8765
```

本项目 Jetson 示例为 `ws://192.168.50.35:8765`。

## 默认布局

在 Foxglove 的 Layouts 菜单选择 **Import from file**，导入安装目录中的：

```bash
ros2 pkg prefix --share remote_monitor_pkg
```

对应文件：

```text
<share>/foxglove/fcr_remote_monitor_layout.json
```

布局包含 Camera Image、ROS Topic Tree、Robot Status 和 Perception Performance。
Foxglove 的布局 JSON 属于应用侧非稳定格式；如果未来版本拒绝旧布局，按上述四个
panel 名称重新添加，并使用同一组 topic/message path 即可重新导出。

## 后续输入

设置 `enable_future_inputs:=true` 后，节点只读观察：

- `/perception/targets_3d` (`vision_servo_msgs/TargetArray`)
- `/cmd_vel` (`geometry_msgs/TwistStamped`)
- `/gimbal/status` (`vision_servo_msgs/GimbalStatus`，对应需求中的 gimbal_state)

这些订阅不会产生任何运动指令。
