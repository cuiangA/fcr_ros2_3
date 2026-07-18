#!/usr/bin/env python3
"""Publish deterministic CameraInfo and a temporary 3D servo target."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from vision_servo_msgs.msg import Target, TargetArray


class ServoChainMockSource(Node):
    def __init__(self):
        super().__init__("servo_chain_mock_source")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("start_delay_sec", 1.0)
        self.declare_parameter("active_duration_sec", 3.0)
        self.declare_parameter("camera_info_topic", "/sony/camera_info")
        self.declare_parameter("target_topic", "/perception/targets_3d")

        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        control_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.camera_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), sensor_qos)
        self.target_pub = self.create_publisher(
            TargetArray, str(self.get_parameter("target_topic").value), control_qos)
        self.active_pub = self.create_publisher(Bool, "/mock/target_active", control_qos)

        self.start_delay = float(self.get_parameter("start_delay_sec").value)
        self.active_duration = float(self.get_parameter("active_duration_sec").value)
        self.started_at = time.monotonic()
        self.was_active = False
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        self.get_logger().info(
            "Mock source ready: target starts in %.2fs and lasts %.2fs" %
            (self.start_delay, self.active_duration))

    def publish_frame(self):
        now = self.get_clock().now().to_msg()
        camera = CameraInfo()
        camera.header.stamp = now
        camera.header.frame_id = "camera_optical_link"
        camera.width = 640
        camera.height = 480
        camera.distortion_model = "plumb_bob"
        camera.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera.k = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
        camera.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera.p = [400.0, 0.0, 320.0, 0.0,
                    0.0, 400.0, 240.0, 0.0,
                    0.0, 0.0, 1.0, 0.0]
        self.camera_pub.publish(camera)

        elapsed = time.monotonic() - self.started_at
        active = self.start_delay <= elapsed < self.start_delay + self.active_duration
        self.active_pub.publish(Bool(data=active))
        if not active:
            if self.was_active:
                self.get_logger().info("Target publishing stopped; timeout test begins")
            self.was_active = False
            return

        targets = TargetArray()
        targets.header.stamp = now
        targets.header.frame_id = "camera_optical_link"
        targets.tracking_id = 1
        target = Target()
        target.header = targets.header
        target.id = 1
        target.class_name = "person"
        target.tracking_state = Target.TRACKING_STATE_CONFIRMED
        target.visible = True
        target.bbox = [260.0, 140.0, 420.0, 400.0]
        target.center = [340.0, 270.0]
        target.confidence = 0.95
        target.height = 260.0
        target.width = 160.0
        target.position = [0.35, 0.0, 3.0]
        target.velocity = [0.0, 0.0, 0.0]
        target.depth_confidence = 1.0
        targets.targets = [target]
        self.target_pub.publish(targets)
        self.was_active = True


def main(args=None):
    rclpy.init(args=args)
    node = ServoChainMockSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
