#!/usr/bin/env python3
"""Acceptance monitor for safe MVP gimbal, cooperative-yaw, and full modes."""

import json
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from vision_servo_msgs.msg import GimbalCmd


EPSILON = 1.0e-4


class MvpModeAcceptance(Node):
    def __init__(self):
        super().__init__("mvp_mode_acceptance")
        self.declare_parameter("mode", "gimbal")
        self.declare_parameter("test_timeout_sec", 10.0)
        self.declare_parameter("max_stop_latency_sec", 0.30)
        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in ("gimbal", "cooperative_yaw", "full"):
            raise ValueError("mode must be gimbal, cooperative_yaw, or full")

        self.timeout = float(self.get_parameter("test_timeout_sec").value)
        self.stop_limit = float(self.get_parameter("max_stop_latency_sec").value)
        self.started = time.monotonic()
        self.target_seen = False
        self.target_stopped_at = None
        self.auto_gimbal = False
        self.final_gimbal = False
        self.auto_yaw = False
        self.final_yaw = False
        self.auto_linear = False
        self.final_linear = False
        self.max_forbidden_linear = 0.0
        self.final_zero = True
        self.final_hold = True
        self.done = False
        self.exit_code = 1

        self.create_subscription(TwistStamped, "/auto/cmd_vel", self.on_auto_vel, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_final_vel, 10)
        self.create_subscription(GimbalCmd, "/auto/cmd_gimbal", self.on_auto_gimbal, 10)
        self.create_subscription(GimbalCmd, "/cmd_gimbal", self.on_final_gimbal, 10)
        self.create_subscription(Bool, "/mock/target_active", self.on_active, 10)
        self.timer = self.create_timer(0.02, self.evaluate)

    def on_auto_vel(self, msg):
        linear = max(abs(msg.twist.linear.x), abs(msg.twist.linear.y))
        self.auto_linear |= linear > EPSILON
        self.auto_yaw |= abs(msg.twist.angular.z) > EPSILON
        if self.mode != "full":
            self.max_forbidden_linear = max(self.max_forbidden_linear, linear)

    def on_final_vel(self, msg):
        linear = max(abs(msg.twist.linear.x), abs(msg.twist.linear.y))
        yaw = abs(msg.twist.angular.z)
        self.final_linear |= linear > EPSILON
        self.final_yaw |= yaw > EPSILON
        self.final_zero = linear <= EPSILON and yaw <= EPSILON
        if self.mode != "full":
            self.max_forbidden_linear = max(self.max_forbidden_linear, linear)

    def on_auto_gimbal(self, msg):
        self.auto_gimbal |= abs(msg.yaw_rate) > EPSILON or abs(msg.pitch_rate) > EPSILON

    def on_final_gimbal(self, msg):
        moving = abs(msg.yaw_rate) > EPSILON or abs(msg.pitch_rate) > EPSILON
        self.final_gimbal |= moving
        self.final_hold = (
            not moving and bool(msg.hold_yaw) and bool(msg.hold_pitch))

    def on_active(self, msg):
        if msg.data:
            self.target_seen = True
        elif self.target_seen and self.target_stopped_at is None:
            self.target_stopped_at = time.monotonic()

    def publisher_check(self):
        velocity = self.get_publishers_info_by_topic("/cmd_vel")
        gimbal = self.get_publishers_info_by_topic("/cmd_gimbal")
        return (
            len(velocity) == 1 and velocity[0].node_name == "command_mux" and
            len(gimbal) == 1 and gimbal[0].node_name == "command_mux")

    def payload(self, latency):
        expected_yaw = self.mode in ("cooperative_yaw", "full")
        expected_linear = self.mode == "full"
        motion_ok = (
            self.auto_gimbal and self.final_gimbal and
            self.auto_yaw == expected_yaw and
            self.final_yaw == expected_yaw and
            self.auto_linear == expected_linear and
            self.final_linear == expected_linear)
        safe_linear = self.max_forbidden_linear <= EPSILON
        passed = (
            motion_ok and safe_linear and self.final_zero and self.final_hold and
            self.publisher_check() and latency <= self.stop_limit)
        return {
            "result": "PASS" if passed else "FAIL",
            "mode": self.mode,
            "auto_gimbal": self.auto_gimbal,
            "final_gimbal": self.final_gimbal,
            "auto_yaw": self.auto_yaw,
            "final_yaw": self.final_yaw,
            "auto_linear": self.auto_linear,
            "final_linear": self.final_linear,
            "max_forbidden_linear": round(self.max_forbidden_linear, 6),
            "final_zero": self.final_zero,
            "final_hold": self.final_hold,
            "unique_final_publishers": self.publisher_check(),
            "stop_latency_ms": round(latency * 1000.0, 1),
        }

    def evaluate(self):
        now = time.monotonic()
        if self.target_stopped_at is not None:
            latency = now - self.target_stopped_at
            if self.final_zero and self.final_hold and latency > 0.05:
                result = self.payload(latency)
                self.finish(result, 0 if result["result"] == "PASS" else 1)
                return
            if latency > self.stop_limit:
                result = self.payload(latency)
                result["reason"] = "stop latency exceeded"
                self.finish(result, 1)
                return
        if now - self.started > self.timeout:
            result = self.payload(self.timeout)
            result["reason"] = "test timeout"
            self.finish(result, 1)

    def finish(self, result, code):
        if self.done:
            return
        self.done = True
        self.exit_code = code
        message = "MVP_MODE_ACCEPTANCE " + json.dumps(result, sort_keys=True)
        (self.get_logger().info if code == 0 else self.get_logger().error)(message)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MvpModeAcceptance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.exit_code = 130
    finally:
        code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
