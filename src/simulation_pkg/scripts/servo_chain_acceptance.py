#!/usr/bin/env python3
"""Automatically verify the phase-1 servo-to-chassis command chain."""

import json
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def moving(twist):
    return any(abs(value) > 1.0e-4 for value in (
        twist.linear.x, twist.linear.y, twist.angular.z))


class ServoChainAcceptance(Node):
    def __init__(self):
        super().__init__("servo_chain_acceptance")
        self.declare_parameter("test_timeout_sec", 12.0)
        self.declare_parameter("max_stop_latency_sec", 0.30)
        self.test_timeout = float(self.get_parameter("test_timeout_sec").value)
        self.max_stop_latency = float(
            self.get_parameter("max_stop_latency_sec").value)

        self.started_at = time.monotonic()
        self.target_was_active = False
        self.target_stopped_at = None
        self.saw_auto_motion = False
        self.saw_final_motion = False
        self.saw_chassis_motion = False
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
            Bool, "/mock/target_active", self.on_target_active, 10)
        self.timer = self.create_timer(0.02, self.evaluate)
        self.get_logger().info("Phase-1 full-chain acceptance monitor started")

    def on_auto_cmd(self, msg):
        self.saw_auto_motion |= moving(msg.twist)

    def on_final_cmd(self, msg):
        self.final_is_zero = not moving(msg.twist)
        self.saw_final_motion |= not self.final_is_zero

    def on_chassis_odom(self, msg):
        self.saw_chassis_motion |= moving(msg.twist.twist)

    def on_target_active(self, msg):
        if msg.data:
            self.target_was_active = True
        elif self.target_was_active and self.target_stopped_at is None:
            self.target_stopped_at = time.monotonic()

    def publisher_check(self):
        infos = self.get_publishers_info_by_topic("/cmd_vel")
        names = sorted({info.node_name for info in infos})
        return len(infos) == 1 and names == ["command_mux"], names, len(infos)

    def evaluate(self):
        now = time.monotonic()
        if self.target_stopped_at is not None:
            stop_latency = now - self.target_stopped_at
            ready = (
                self.saw_auto_motion and self.saw_final_motion and
                self.saw_chassis_motion and self.final_is_zero)
            if ready:
                publishers_ok, names, count = self.publisher_check()
                result = {
                    "result": "PASS" if (
                        publishers_ok and stop_latency <= self.max_stop_latency
                    ) else "FAIL",
                    "auto_motion": self.saw_auto_motion,
                    "final_motion": self.saw_final_motion,
                    "chassis_motion": self.saw_chassis_motion,
                    "stop_latency_ms": round(stop_latency * 1000.0, 1),
                    "cmd_vel_publisher_count": count,
                    "cmd_vel_publishers": names,
                }
                self.finish(result, 0 if result["result"] == "PASS" else 1)
                return
            if stop_latency > self.max_stop_latency:
                self.finish({
                    "result": "FAIL",
                    "reason": (
                        "full-chain evidence incomplete or final /cmd_vel "
                        "stop was late"),
                    "auto_motion": self.saw_auto_motion,
                    "final_motion": self.saw_final_motion,
                    "chassis_motion": self.saw_chassis_motion,
                    "final_is_zero": self.final_is_zero,
                    "stop_latency_ms": round(stop_latency * 1000.0, 1),
                }, 1)
                return

        if now - self.started_at > self.test_timeout:
            publishers_ok, names, count = self.publisher_check()
            self.finish({
                "result": "FAIL",
                "reason": "test timeout",
                "auto_motion": self.saw_auto_motion,
                "final_motion": self.saw_final_motion,
                "chassis_motion": self.saw_chassis_motion,
                "target_cycle_seen": self.target_stopped_at is not None,
                "cmd_vel_publisher_ok": publishers_ok,
                "cmd_vel_publisher_count": count,
                "cmd_vel_publishers": names,
            }, 1)

    def finish(self, result, exit_code):
        if self.done:
            return
        self.done = True
        self.exit_code = exit_code
        message = "PHASE1_ACCEPTANCE " + json.dumps(result, sort_keys=True)
        (self.get_logger().info if exit_code == 0 else self.get_logger().error)(message)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ServoChainAcceptance()
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
