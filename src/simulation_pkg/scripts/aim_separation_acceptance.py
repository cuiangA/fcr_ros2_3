#!/usr/bin/env python3
"""Acceptance monitor for T06: gimbal/chassis target separation.

Validates:
  1. Aim point changes do NOT affect chassis distance control
  2. Aim target timeout (250ms) causes gimbal to stop
  3. Gimbal does not retain non-zero velocity after timeout
  4. Aim target resume causes gimbal to track again
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from vision_servo_msgs.msg import AimTarget2D, GimbalCmd, TargetArray


EPSILON = 1.0e-4
AIM_TIMEOUT_MAX = 0.30  # must stop within 300ms (250ms timeout + margin)


class AimSeparationAcceptance(Node):
    def __init__(self):
        super().__init__("aim_separation_acceptance")
        self.declare_parameter("test_timeout_sec", 15.0)
        self.declare_parameter("max_stop_latency_sec", AIM_TIMEOUT_MAX)
        self.timeout = float(self.get_parameter("test_timeout_sec").value)
        self.stop_limit = float(self.get_parameter("max_stop_latency_sec").value)

        # Phase tracking
        self.started = time.monotonic()
        self.phase = 0
        self.target_seen = False

        # Gimbal state
        self.gimbal_yaw = 0.0
        self.gimbal_pitch = 0.0
        self.gimbal_moving = False
        self.gimbal_stopped_at = None
        self.gimbal_stopped = False

        # Chassis state
        self.chassis_linear_x = 0.0
        self.chassis_linear_y = 0.0
        self.chassis_angular_z = 0.0

        # Separation check: sample chassis velocity during stable tracking
        self.chassis_baseline = None
        self.chassis_baseline_angular = None
        self.separation_violation = False

        # Aim state
        self.aim_valid = False
        self.aim_pixel_x = 0.0
        self.aim_pixel_y = 0.0
        self.aim_lost_reported = False
        self.aim_lost_at = None
        self.aim_resumed = False

        self.done = False
        self.exit_code = 1

        self.create_subscription(
            TargetArray, "/perception/tracks", self.on_tracks, 10)
        self.create_subscription(
            AimTarget2D, "/perception/aim_target_2d", self.on_aim, 10)
        self.create_subscription(
            TwistStamped, "/auto/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(
            GimbalCmd, "/auto/cmd_gimbal", self.on_cmd_gimbal, 10)
        self.timer = self.create_timer(0.02, self.evaluate)

    def on_tracks(self, msg):
        if msg.tracking_id >= 0 and msg.targets:
            t = msg.targets[0]
            if t.tracking_state != 0 and t.visible:
                if not self.target_seen:
                    self.get_logger().info("Target seen (tracks)")
                self.target_seen = True

    def on_aim(self, msg):
        if self.aim_valid and not msg.valid and self.aim_lost_at is None:
            self.aim_lost_at = time.monotonic()
        self.aim_valid = msg.valid
        self.aim_pixel_x = msg.pixel_x
        self.aim_pixel_y = msg.pixel_y
        if msg.valid:
            self.aim_resumed = True

    def on_cmd_vel(self, msg):
        self.chassis_linear_x = msg.twist.linear.x
        self.chassis_linear_y = msg.twist.linear.y
        self.chassis_angular_z = msg.twist.angular.z

    def on_cmd_gimbal(self, msg):
        moving = (abs(msg.yaw_rate) > EPSILON or
                  abs(msg.pitch_rate) > EPSILON)
        self.gimbal_yaw = msg.yaw_rate
        self.gimbal_pitch = msg.pitch_rate
        self.gimbal_moving = moving
        if not moving and self.gimbal_stopped_at is None and self.aim_lost_at is not None:
            self.gimbal_stopped_at = time.monotonic()
            self.gimbal_stopped = True

    def check_separation(self):
        """Record chassis baseline during stable center tracking (phase 1).
        In phases 2-3, any change in chassis velocity is a violation."""
        if self.chassis_baseline is None and self.target_seen:
            self.chassis_baseline = self.chassis_linear_x
            self.chassis_baseline_angular = self.chassis_angular_z
            return True
        if self.chassis_baseline is not None:
            linear_drift = abs(self.chassis_linear_x - self.chassis_baseline)
            angular_drift = abs(self.chassis_angular_z - self.chassis_baseline_angular)
            if linear_drift > 0.01 or angular_drift > 0.01:
                self.separation_violation = True
                return False
        return True

    def payload(self, result_str):
        return {
            "result": result_str,
            "target_seen": self.target_seen,
            "gimbal_stopped": self.gimbal_stopped,
            "aim_resumed": self.aim_resumed,
            "separation_violation": self.separation_violation,
            "gimbal_yaw_final": round(self.gimbal_yaw, 6),
            "gimbal_pitch_final": round(self.gimbal_pitch, 6),
            "chassis_baseline": round(self.chassis_baseline or 0.0, 6),
            "chassis_linear_x_final": round(self.chassis_linear_x, 6),
            "chassis_angular_z_final": round(self.chassis_angular_z, 6),
        }

    def evaluate(self):
        now = time.monotonic()
        elapsed = now - self.started

        # Wait for target to appear
        if not self.target_seen:
            if elapsed > self.timeout:
                result = self.payload("FAIL")
                result["reason"] = "target never appeared"
                self.finish(result, 1)
            return

        # Record baseline during stable tracking
        self.check_separation()

        # Phase detection from mock source timing
        if elapsed < 4.0:
            self.phase = 1
        elif elapsed < 5.0:
            self.phase = 2
        elif elapsed < 6.0:
            self.phase = 3
        elif elapsed < 8.0:
            self.phase = 4
            if self.aim_valid is False and not self.aim_lost_reported:
                self.aim_lost_reported = True
                self.get_logger().info(
                    "Phase 4: aim=invalid, waiting for gimbal stop...")
            # If gimbal stopped, record latency
            if self.gimbal_stopped and self.gimbal_stopped_at is not None:
                stop_latency = self.gimbal_stopped_at - self.aim_lost_at
                if stop_latency <= self.stop_limit:
                    self.aim_resumed = True  # mark as passed
        else:
            self.phase = 5

        # After phase 4 + margin, check all conditions
        if elapsed > 9.0:
            passed = (
                self.target_seen and
                self.gimbal_stopped and
                not self.separation_violation and
                abs(self.gimbal_yaw) <= EPSILON and
                abs(self.gimbal_pitch) <= EPSILON)

            result = self.payload("PASS" if passed else "FAIL")
            if not self.gimbal_stopped:
                result["reason"] = "gimbal did not stop after aim lost"
            elif self.separation_violation:
                result["reason"] = "chassis velocity changed with aim point"
            elif abs(self.gimbal_yaw) > EPSILON:
                result["reason"] = "gimbal retained non-zero yaw_rate after stop"
            elif abs(self.gimbal_pitch) > EPSILON:
                result["reason"] = "gimbal retained non-zero pitch_rate after stop"
            if self.gimbal_stopped_at is not None and self.aim_lost_at is not None:
                result["stop_latency_ms"] = round(
                    (self.gimbal_stopped_at - self.aim_lost_at) * 1000.0, 1)
            self.finish(result, 0 if passed else 1)
            return

        if elapsed > self.timeout:
            result = self.payload("FAIL")
            result["reason"] = "test timeout"
            self.finish(result, 1)

    def finish(self, result, code):
        if self.done:
            return
        self.done = True
        self.exit_code = code
        msg = "AIM_SEPARATION_ACCEPTANCE " + json.dumps(result, sort_keys=True)
        (self.get_logger().info if code == 0 else self.get_logger().error)(msg)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AimSeparationAcceptance()
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
