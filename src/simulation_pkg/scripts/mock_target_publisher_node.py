#!/usr/bin/env python3
"""Publish simple TargetArray scenarios for the MVP follow controller."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from vision_servo_msgs.msg import Target, TargetArray


class MockTargetPublisher(Node):
    def __init__(self):
        super().__init__("mock_target_publisher_node")

        self.declare_parameter("target_topic", "/target/current")
        self.declare_parameter("scenario", "center")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("desired_distance", 2.0)
        self.declare_parameter("confidence", 1.0)
        self.declare_parameter("target_id", 0)
        self.declare_parameter("sinusoidal_amplitude_px", 160.0)
        self.declare_parameter("sinusoidal_omega", 1.0)

        target_topic = self.get_parameter("target_topic").value
        publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(TargetArray, target_topic, qos)
        self.start_time = self.get_clock().now()
        self.last_unknown_scenario = None

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_target)
        self.get_logger().info(
            f"Mock target publisher started on {target_topic}, scenario="
            f"{self.get_parameter('scenario').value}"
        )

    def publish_target(self):
        msg = TargetArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"

        scenario = str(self.get_parameter("scenario").value).lower()
        values = self.scenario_values(scenario)
        if values is None:
            msg.tracking_id = -1
            self.publisher.publish(msg)
            return

        cx, cy, depth = values
        target = self.make_target(cx, cy, depth)
        msg.targets = [target]
        msg.tracking_id = target.id
        self.publisher.publish(msg)

    def scenario_values(self, scenario):
        width = float(self.get_parameter("image_width").value)
        height = float(self.get_parameter("image_height").value)
        desired_distance = float(self.get_parameter("desired_distance").value)

        cx_mid = 0.5 * width
        cy_mid = 0.5 * height

        if scenario == "center":
            return cx_mid, cy_mid, desired_distance
        if scenario == "left":
            return 0.25 * width, cy_mid, desired_distance
        if scenario == "right":
            return 0.75 * width, cy_mid, desired_distance
        if scenario == "up":
            return cx_mid, 0.25 * height, desired_distance
        if scenario == "down":
            return cx_mid, 0.75 * height, desired_distance
        if scenario == "far":
            return cx_mid, cy_mid, desired_distance + 1.0
        if scenario == "near":
            return cx_mid, cy_mid, max(0.1, desired_distance - 1.0)
        if scenario == "lost":
            return None
        if scenario == "sinusoidal":
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            amplitude = float(self.get_parameter("sinusoidal_amplitude_px").value)
            omega = float(self.get_parameter("sinusoidal_omega").value)
            cx = cx_mid + amplitude * math.sin(omega * elapsed)
            return cx, cy_mid, desired_distance

        if scenario != self.last_unknown_scenario:
            self.get_logger().warning(
                f"Unknown scenario '{scenario}', falling back to center"
            )
            self.last_unknown_scenario = scenario
        return cx_mid, cy_mid, desired_distance

    def make_target(self, cx, cy, depth):
        width = float(self.get_parameter("image_width").value)
        height = float(self.get_parameter("image_height").value)
        target_id = int(self.get_parameter("target_id").value)
        confidence = float(self.get_parameter("confidence").value)

        bbox_w = 0.125 * width
        bbox_h = 0.25 * height
        x_min = max(0.0, cx - 0.5 * bbox_w)
        y_min = max(0.0, cy - 0.5 * bbox_h)
        x_max = min(width, cx + 0.5 * bbox_w)
        y_max = min(height, cy + 0.5 * bbox_h)

        target = Target()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "camera_link"
        target.id = target_id
        target.class_name = "mock_target"
        target.bbox = [float(x_min), float(y_min), float(x_max), float(y_max)]
        target.center = [float(cx), float(cy)]
        target.confidence = float(confidence)
        target.height = float(y_max - y_min)
        target.width = float(x_max - x_min)
        target.position = [0.0, 0.0, float(depth)]
        target.velocity = [0.0, 0.0, 0.0]
        target.depth_confidence = 1.0
        return target


def main(args=None):
    rclpy.init(args=args)
    node = MockTargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
