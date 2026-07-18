#!/usr/bin/env python3
"""Publish deterministic CameraInfo and a temporary 2D or 3D servo target."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from vision_servo_msgs.msg import PlatformState, Target, TargetArray


class ServoChainMockSource(Node):
    def __init__(self):
        super().__init__("servo_chain_mock_source")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("start_delay_sec", 1.0)
        self.declare_parameter("active_duration_sec", 3.0)
        self.declare_parameter("camera_info_topic", "/sony/camera_info")
        self.declare_parameter("target_topic", "/perception/targets_3d")
        self.declare_parameter("target_mode", "3d")
        self.declare_parameter("platform_yaw", 0.0)

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
        platform_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.camera_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), sensor_qos)
        self.target_pub = self.create_publisher(
            TargetArray, str(self.get_parameter("target_topic").value), control_qos)
        self.active_pub = self.create_publisher(Bool, "/mock/target_active", control_qos)
        self.platform_pub = self.create_publisher(
            PlatformState, "/platform/state", platform_qos)

        self.start_delay = float(self.get_parameter("start_delay_sec").value)
        self.active_duration = float(self.get_parameter("active_duration_sec").value)
        self.target_mode = str(self.get_parameter("target_mode").value).lower()
        self.platform_yaw = float(self.get_parameter("platform_yaw").value)
        if self.target_mode not in ("2d", "3d"):
            raise ValueError("target_mode must be '2d' or '3d'")
        self.started_at = time.monotonic()
        self.was_active = False
        self.target_has_ended = False
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

        platform = PlatformState()
        platform.header.stamp = now
        platform.header.frame_id = "base_link"
        platform.gimbal_yaw = self.platform_yaw
        platform.chassis_connected = True
        platform.gimbal_connected = True
        self.platform_pub.publish(platform)

        elapsed = time.monotonic() - self.started_at
        active = self.start_delay <= elapsed < self.start_delay + self.active_duration
        self.active_pub.publish(Bool(data=active))
        if not active:
            if self.was_active:
                self.get_logger().info("Target publishing stopped; timeout test begins")
                self.target_has_ended = True
            self.was_active = False
            if self.target_mode == "2d" and self.target_has_ended:
                self.publish_lost_track(now)
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
        if self.target_mode == "2d":
            target.bbox = [60.0, 120.0, 220.0, 380.0]
            target.center = [140.0, 250.0]
        else:
            target.bbox = [260.0, 140.0, 420.0, 400.0]
            target.center = [340.0, 270.0]
        target.confidence = 0.95
        target.height = 260.0
        target.width = 160.0
        target.position = (
            [0.0, 0.0, 0.0] if self.target_mode == "2d"
            else [0.35, 0.0, 3.0])
        target.velocity = [0.0, 0.0, 0.0]
        target.depth_confidence = 0.0 if self.target_mode == "2d" else 1.0
        targets.targets = [target]
        self.target_pub.publish(targets)
        self.was_active = True

    def publish_lost_track(self, stamp):
        """Keep publishing a LOST track to prove it cannot refresh servo timeout."""
        targets = TargetArray()
        targets.header.stamp = stamp
        targets.header.frame_id = "camera_optical_link"
        targets.tracking_id = 1
        target = Target()
        target.header = targets.header
        target.id = 1
        target.class_name = "person"
        target.tracking_state = Target.TRACKING_STATE_LOST
        target.visible = False
        target.bbox = [60.0, 120.0, 220.0, 380.0]
        target.center = [140.0, 250.0]
        target.confidence = 0.5
        target.height = 260.0
        target.width = 160.0
        target.position = [0.0, 0.0, 0.0]
        target.velocity = [0.0, 0.0, 0.0]
        target.depth_confidence = 0.0
        targets.targets = [target]
        self.target_pub.publish(targets)


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
