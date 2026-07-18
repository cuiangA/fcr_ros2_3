#!/usr/bin/env python3
"""Verify safe 2D IBVS: yaw/gimbal enabled, chassis translation forbidden."""

import json
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from vision_servo_msgs.msg import GimbalCmd


def linear_magnitude(twist):
    return max(abs(twist.linear.x), abs(twist.linear.y))


def moving(twist):
    return linear_magnitude(twist) > 1.0e-4 or abs(twist.angular.z) > 1.0e-4


class Servo2DAcceptance(Node):
    def __init__(self):
        super().__init__("servo_2d_acceptance")
        self.declare_parameter("test_timeout_sec", 12.0)
        self.declare_parameter("max_stop_latency_sec", 0.30)
        self.declare_parameter("max_linear_leak", 1.0e-4)
        self.test_timeout = float(self.get_parameter("test_timeout_sec").value)
        self.max_stop_latency = float(
            self.get_parameter("max_stop_latency_sec").value)
        self.max_linear_leak = float(self.get_parameter("max_linear_leak").value)

        self.started_at = time.monotonic()
        self.target_was_active = False
        self.target_stopped_at = None
        self.saw_auto_yaw = False
        self.saw_final_yaw = False
        self.saw_chassis_yaw = False
        self.saw_auto_gimbal_motion = False
        self.saw_final_gimbal_motion = False
        self.final_gimbal_holding = True
        self.max_auto_linear = 0.0
        self.max_final_linear = 0.0
        self.max_chassis_linear = 0.0
        self.final_is_zero = True
        self.done = False
        self.exit_code = 1

        self.create_subscription(
            TwistStamped, "/auto/cmd_vel", self.on_auto_cmd, 10)
        self.create_subscription(
            TwistStamped, "/cmd_vel", self.on_final_cmd, 10)
        self.create_subscription(
            Odometry, "/chassis/odom_raw", self.on_chassis_odom, 10)
        self.create_subscription(
            GimbalCmd, "/auto/cmd_gimbal", self.on_auto_gimbal_cmd, 10)
        self.create_subscription(
            GimbalCmd, "/cmd_gimbal", self.on_final_gimbal_cmd, 10)
        self.create_subscription(
            Bool, "/mock/target_active", self.on_target_active, 10)
        self.timer = self.create_timer(0.02, self.evaluate)
        self.get_logger().info("Phase-2 safe 2D IBVS acceptance monitor started")

    def on_auto_cmd(self, msg):
        self.max_auto_linear = max(
            self.max_auto_linear, linear_magnitude(msg.twist))
        self.saw_auto_yaw |= abs(msg.twist.angular.z) > 1.0e-4

    def on_final_cmd(self, msg):
        self.max_final_linear = max(
            self.max_final_linear, linear_magnitude(msg.twist))
        self.saw_final_yaw |= abs(msg.twist.angular.z) > 1.0e-4
        self.final_is_zero = not moving(msg.twist)

    def on_chassis_odom(self, msg):
        self.max_chassis_linear = max(
            self.max_chassis_linear, linear_magnitude(msg.twist.twist))
        self.saw_chassis_yaw |= abs(msg.twist.twist.angular.z) > 1.0e-4

    def on_auto_gimbal_cmd(self, msg):
        self.saw_auto_gimbal_motion |= (
            abs(msg.yaw_rate) > 1.0e-4 or abs(msg.pitch_rate) > 1.0e-4)

    def on_final_gimbal_cmd(self, msg):
        self.saw_final_gimbal_motion |= (
            abs(msg.yaw_rate) > 1.0e-4 or abs(msg.pitch_rate) > 1.0e-4)
        self.final_gimbal_holding = (
            msg.hold_yaw and msg.hold_pitch and
            abs(msg.yaw_rate) <= 1.0e-4 and abs(msg.pitch_rate) <= 1.0e-4)

    def on_target_active(self, msg):
        if msg.data:
            self.target_was_active = True
        elif self.target_was_active and self.target_stopped_at is None:
            self.target_stopped_at = time.monotonic()

    def publisher_check(self):
        velocity_infos = self.get_publishers_info_by_topic("/cmd_vel")
        velocity_names = sorted({info.node_name for info in velocity_infos})
        gimbal_infos = self.get_publishers_info_by_topic("/cmd_gimbal")
        gimbal_names = sorted({info.node_name for info in gimbal_infos})
        publishers_ok = (
            len(velocity_infos) == 1 and velocity_names == ["command_mux"] and
            len(gimbal_infos) == 1 and gimbal_names == ["command_mux"])
        return {
            "ok": publishers_ok,
            "cmd_vel_count": len(velocity_infos),
            "cmd_vel_names": velocity_names,
            "cmd_gimbal_count": len(gimbal_infos),
            "cmd_gimbal_names": gimbal_names,
        }

    def result_payload(self, stop_latency):
        publishers = self.publisher_check()
        motion_ok = (
            self.saw_auto_yaw and self.saw_final_yaw and
            self.saw_chassis_yaw and self.saw_auto_gimbal_motion and
            self.saw_final_gimbal_motion)
        translation_ok = max(
            self.max_auto_linear,
            self.max_final_linear,
            self.max_chassis_linear) <= self.max_linear_leak
        passed = (
            motion_ok and translation_ok and self.final_is_zero and
            self.final_gimbal_holding and publishers["ok"] and
            stop_latency <= self.max_stop_latency)
        return {
            "result": "PASS" if passed else "FAIL",
            "auto_yaw": self.saw_auto_yaw,
            "final_yaw": self.saw_final_yaw,
            "chassis_yaw": self.saw_chassis_yaw,
            "auto_gimbal_motion": self.saw_auto_gimbal_motion,
            "final_gimbal_motion": self.saw_final_gimbal_motion,
            "final_gimbal_holding": self.final_gimbal_holding,
            "max_auto_linear": round(self.max_auto_linear, 6),
            "max_final_linear": round(self.max_final_linear, 6),
            "max_chassis_linear": round(self.max_chassis_linear, 6),
            "stop_latency_ms": round(stop_latency * 1000.0, 1),
            "cmd_vel_publisher_count": publishers["cmd_vel_count"],
            "cmd_vel_publishers": publishers["cmd_vel_names"],
            "cmd_gimbal_publisher_count": publishers["cmd_gimbal_count"],
            "cmd_gimbal_publishers": publishers["cmd_gimbal_names"],
        }

    def evaluate(self):
        now = time.monotonic()
        if self.target_stopped_at is not None:
            stop_latency = now - self.target_stopped_at
            if self.final_is_zero and stop_latency > 0.05:
                result = self.result_payload(stop_latency)
                self.finish(result, 0 if result["result"] == "PASS" else 1)
                return
            if stop_latency > self.max_stop_latency:
                result = self.result_payload(stop_latency)
                result["reason"] = "final /cmd_vel did not stop within limit"
                self.finish(result, 1)
                return

        if now - self.started_at > self.test_timeout:
            result = self.result_payload(self.test_timeout)
            result["reason"] = "test timeout"
            self.finish(result, 1)

    def finish(self, result, exit_code):
        if self.done:
            return
        self.done = True
        self.exit_code = exit_code
        message = "PHASE2_ACCEPTANCE " + json.dumps(result, sort_keys=True)
        (self.get_logger().info if exit_code == 0 else self.get_logger().error)(
            message)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Servo2DAcceptance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.exit_code = 130
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
