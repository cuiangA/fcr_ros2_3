#!/usr/bin/env python3
"""
@file mock_detector.py
@brief 合成目标检测器 — 生成模拟的 2D 检测结果，替代真实 YOLO 推理。

在仿真/测试环境中替代真实检测节点，以固定频率生成合成检测框。
发布的 TargetArray 直接对接跟踪节点和深度估计节点，使感知管线能
端到端运行而无需真实相机和 YOLO 模型。

生成的检测框在图像中做水平正弦摆动，模拟一个运动目标的检测输出。

@note 发布话题：
  - /perception/detections (vision_servo_msgs/TargetArray)  合成 2D 检测框
"""

import rclpy
from rclpy.node import Node
from vision_servo_msgs.msg import Target, TargetArray
import math


class MockDetector(Node):
    """
    @class MockDetector
    @brief 以固定频率生成合成检测框的 ROS2 仿真节点。

    不订阅任何话题 — 纯定时器驱动。每个周期生成一个在图像中
    水平摆动的方形边界框，模拟真实检测器的输出。

    参数：
      - image_width   (int,   默认 640):  虚拟图像宽度 (px)
      - image_height  (int,   默认 480):  虚拟图像高度 (px)
      - rate          (float, 默认 30.0): 发布频率 (Hz)
      - bbox_size     (float, 默认 80.0): 边界框边长 (px)
      - camera_frame  (string, 默认 "camera_link"): 输出消息的坐标系
      - class_name    (string, 默认 "target"): 目标类别名称
    """

    def __init__(self):
        super().__init__("mock_detector")

        # ── 1. 声明参数 ────────────────────────────────────────────
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("bbox_size", 80.0)
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("class_name", "target")

        self.width = self.get_parameter("image_width").value
        self.height = self.get_parameter("image_height").value
        rate = self.get_parameter("rate").value
        self.bbox_size = self.get_parameter("bbox_size").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.class_name = self.get_parameter("class_name").value

        # ── 2. 创建发布者 ──────────────────────────────────────────
        self.det_pub = self.create_publisher(
            TargetArray, "/perception/detections", 5
        )

        # ── 3. 创建定时器 ──────────────────────────────────────────
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.t = 0.0
        self.frame_count = 0

        self.get_logger().info(
            f"Mock 检测器已启动 ({self.width}x{self.height}, "
            f"{rate}Hz, class={self.class_name})"
        )

    def publish(self):
        """
        定时器回调 — 生成一个水平摆动的检测框并发布。
        """
        # 水平正弦摆动：边界框中心沿 X 轴在图像 20%~80% 范围摆动
        phase = self.t * 0.5  # 缓慢摆动
        cx = self.width * 0.2 + self.width * 0.6 * (math.sin(phase) + 1.0) / 2.0
        # 垂直方向略微上下浮动
        cy = self.height * 0.5 + self.height * 0.1 * math.cos(phase * 1.3)

        half = self.bbox_size / 2.0

        # ── 组装 Target 消息 ───────────────────────────────────────
        target = Target()
        target.id = -1  # 由跟踪器分配 ID
        target.class_name = self.class_name
        target.confidence = 0.9
        target.bbox = [cx - half, cy - half, cx + half, cy + half]
        target.center = [cx, cy]
        # position/velocity/depth_confidence 由深度估计节点填充

        # ── 组装 TargetArray 消息 ──────────────────────────────────
        msg = TargetArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.targets = [target]
        msg.tracking_id = -1

        self.det_pub.publish(msg)
        self.t += (1.0 / self.get_parameter("rate").value)
        self.frame_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
