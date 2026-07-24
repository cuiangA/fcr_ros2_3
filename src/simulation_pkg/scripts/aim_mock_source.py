#!/usr/bin/env python3
"""Publish deterministic TargetArray + AimTarget2D for separation acceptance test.

Scenarios (driven by aim_phase parameter):
  0: No target (initial idle)
  1: Target active + center aim point, 3s
  2: Aim point jumps right (pixel_x=480) — gimbal must follow, chassis must not change
  3: Aim point jumps to upper body (pixel_y=120) — same expectation
  4: Aim target lost (valid=false) — gimbal must stop within 250ms
  5: Aim target resumes — gimbal must resume tracking

Throughout all phases, TargetArray depth stays constant at 3.0m.
Chassis translation must NOT change when aim point changes.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from vision_servo_msgs.msg import AimTarget2D, PlatformState, Target, TargetArray


class AimMockSource(Node):
    def __init__(self):
        super().__init__("aim_mock_source")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("tracks_topic", "/perception/tracks")
        self.declare_parameter("aim_topic", "/perception/aim_target_2d")
        self.declare_parameter("camera_info_topic", "/sony/camera_info")

        rate = float(self.get_parameter("publish_rate_hz").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        control_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        platform_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.tracks_pub = self.create_publisher(
            TargetArray, str(self.get_parameter("tracks_topic").value), control_qos)
        self.aim_pub = self.create_publisher(
            AimTarget2D, str(self.get_parameter("aim_topic").value), control_qos)
        self.camera_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), sensor_qos)
        self.active_pub = self.create_publisher(
            Bool, "/mock/target_active", control_qos)
        self.platform_pub = self.create_publisher(
            PlatformState, "/platform/state", platform_qos)

        self.started_at = time.monotonic()
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        self.get_logger().info(
            "Aim mock source ready: 5 phases, ~9s total")

    def _phase(self, elapsed):
        if elapsed < 1.0:
            return 0  # idle
        elif elapsed < 4.0:
            return 1  # active + center aim
        elif elapsed < 5.0:
            return 2  # aim point jumps right
        elif elapsed < 6.0:
            return 3  # aim point jumps upper body
        elif elapsed < 8.0:
            return 4  # aim target lost
        else:
            return 5  # aim target resumes

    def publish_frame(self):
        now = self.get_clock().now().to_msg()
        elapsed = time.monotonic() - self.started_at
        phase = self._phase(elapsed)

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
        platform.gimbal_yaw = 0.0
        platform.chassis_connected = True
        platform.gimbal_connected = True
        self.platform_pub.publish(platform)

        active = phase >= 1
        self.active_pub.publish(Bool(data=active))
        if not active:
            if phase == 0:
                self.get_logger().info("Phase 0: idle, no target", throttle_duration_sec=1.0)
            return

        target = Target()
        target.header.stamp = now
        target.header.frame_id = "camera_optical_link"
        target.id = 1
        target.class_name = "person"
        target.tracking_state = Target.TRACKING_STATE_CONFIRMED
        target.visible = True
        target.bbox = [60.0, 120.0, 220.0, 380.0]
        target.center = [140.0, 250.0]
        target.confidence = 0.95
        target.height = 260.0
        target.width = 160.0
        target.position = [0.0, 0.0, 3.0]
        target.velocity = [0.0, 0.0, 0.0]
        target.depth_confidence = 1.0

        tracks = TargetArray()
        tracks.header.stamp = now
        tracks.header.frame_id = "camera_optical_link"
        tracks.tracking_id = 1
        tracks.targets = [target]
        self.tracks_pub.publish(tracks)

        aim = AimTarget2D()
        aim.header.stamp = now
        aim.header.frame_id = "camera_optical_link"
        aim.tracking_id = 1
        aim.confidence = 0.95

        if phase == 1:
            aim.pixel_x = 320.0
            aim.pixel_y = 240.0
            aim.source = AimTarget2D.UPPER_BODY
            aim.valid = True
            self.get_logger().info("Phase 1: center aim (320, 240)", throttle_duration_sec=1.0)
        elif phase == 2:
            aim.pixel_x = 480.0
            aim.pixel_y = 240.0
            aim.source = AimTarget2D.UPPER_BODY
            aim.valid = True
            self.get_logger().info("Phase 2: aim jumps RIGHT (480, 240)", throttle_duration_sec=0.5)
        elif phase == 3:
            aim.pixel_x = 320.0
            aim.pixel_y = 120.0
            aim.source = AimTarget2D.UPPER_BODY
            aim.valid = True
            self.get_logger().info("Phase 3: aim jumps UPPER (320, 120)", throttle_duration_sec=0.5)
        elif phase == 4:
            aim.pixel_x = 320.0
            aim.pixel_y = 240.0
            aim.source = AimTarget2D.LOST_PREDICTION
            aim.valid = False
            self.get_logger().info("Phase 4: aim LOST (valid=false) — gimbal must stop", throttle_duration_sec=0.5)
        else:
            aim.pixel_x = 320.0
            aim.pixel_y = 240.0
            aim.source = AimTarget2D.UPPER_BODY
            aim.valid = True
            self.get_logger().info("Phase 5: aim RESUMED — gimbal must track", throttle_duration_sec=0.5)

        self.aim_pub.publish(aim)


def main(args=None):
    rclpy.init(args=args)
    node = AimMockSource()
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
