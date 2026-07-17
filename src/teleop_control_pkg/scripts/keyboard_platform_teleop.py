#!/usr/bin/env python3
"""Fail-safe terminal keyboard control for the FCR platform.

Terminal input has no key-release events. Motion therefore uses a short lease:
repeat a motion key to keep the deadman active; releasing all keys stops the
robot after key_timeout_sec. A gamepad should be used when a true held button
deadman is required.
"""

from __future__ import annotations

import select
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from vision_servo_msgs.msg import GimbalCmd


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class Motion:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    gimbal_yaw: float = 0.0
    gimbal_pitch: float = 0.0


class PosixKeyboardReader:
    def __init__(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard teleop requires an interactive terminal")
        import termios
        import tty

        self._termios = termios
        self._old_settings = termios.tcgetattr(sys.stdin)
        self._buffer = ""
        self._escape_started_at: Optional[float] = None
        tty.setcbreak(sys.stdin.fileno())

    def read_key(self) -> Optional[str]:
        # SSH/terminal may split an arrow escape sequence across timer ticks.
        # Accumulate available bytes without blocking the ROS executor.
        while select.select([sys.stdin], [], [], 0.0)[0]:
            self._buffer += sys.stdin.read(1)

        if not self._buffer:
            return None
        if not self._buffer.startswith("\x1b"):
            key, self._buffer = self._buffer[0], self._buffer[1:]
            return key

        if self._escape_started_at is None:
            self._escape_started_at = time.monotonic()
        if len(self._buffer) < 3:
            # A standalone/incomplete ESC is deliberately ignored instead of
            # becoming an ESTOP. X is the unambiguous software ESTOP key.
            if time.monotonic() - self._escape_started_at > 0.15:
                self._buffer = self._buffer[1:]
                self._escape_started_at = None
            return None

        sequence, self._buffer = self._buffer[:3], self._buffer[3:]
        self._escape_started_at = None
        return {
            "\x1b[A": "UP",
            "\x1b[B": "DOWN",
            "\x1b[C": "RIGHT",
            "\x1b[D": "LEFT",
        }.get(sequence)

    def close(self) -> None:
        self._termios.tcsetattr(
            sys.stdin, self._termios.TCSADRAIN, self._old_settings
        )


class KeyboardPlatformTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_platform_teleop")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("key_timeout_sec", 0.25)
        self.declare_parameter("linear_x", 0.03)
        self.declare_parameter("linear_y", 0.03)
        self.declare_parameter("angular_z", 0.15)
        self.declare_parameter("gimbal_yaw_rate", 0.15)
        self.declare_parameter("gimbal_pitch_rate", 0.12)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("speed_step", 0.25)
        self.declare_parameter("min_speed_scale", 0.25)
        self.declare_parameter("max_speed_scale", 1.5)
        self.declare_parameter("frame_id", "base_link")

        self.rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.key_timeout = max(float(self.get_parameter("key_timeout_sec").value), 0.05)
        self.linear_x = abs(float(self.get_parameter("linear_x").value))
        self.linear_y = abs(float(self.get_parameter("linear_y").value))
        self.angular_z = abs(float(self.get_parameter("angular_z").value))
        self.gimbal_yaw = abs(float(self.get_parameter("gimbal_yaw_rate").value))
        self.gimbal_pitch = abs(float(self.get_parameter("gimbal_pitch_rate").value))
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.speed_step = abs(float(self.get_parameter("speed_step").value))
        self.min_scale = max(float(self.get_parameter("min_speed_scale").value), 0.01)
        self.max_scale = max(float(self.get_parameter("max_speed_scale").value), self.min_scale)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.speed_scale = clamp(self.speed_scale, self.min_scale, self.max_scale)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.velocity_pub = self.create_publisher(TwistStamped, "teleop/cmd_vel", qos)
        self.gimbal_pub = self.create_publisher(GimbalCmd, "teleop/cmd_gimbal", qos)
        self.heartbeat_pub = self.create_publisher(Empty, "teleop/heartbeat", qos)
        self.deadman_pub = self.create_publisher(Bool, "teleop/deadman", qos)
        self.estop_pub = self.create_publisher(Bool, "teleop/estop", qos)
        self.clear_estop_pub = self.create_publisher(Empty, "teleop/clear_estop", qos)
        self.mode_pub = self.create_publisher(String, "teleop/mode", qos)

        self.motion = Motion()
        self.last_motion_time = self.get_clock().now()
        self.deadman = False
        self.shutdown_requested = False
        self.keyboard: Optional[PosixKeyboardReader] = None
        try:
            self.keyboard = PosixKeyboardReader()
        except RuntimeError as error:
            self.get_logger().error(str(error))
            self.shutdown_requested = True

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self._publish_mode("manual")
        self._print_help()

    def close(self) -> None:
        self._stop(repeat=3)
        if self.keyboard is not None:
            self.keyboard.close()

    def _tick(self) -> None:
        if self.shutdown_requested:
            self._stop(repeat=3)
            rclpy.shutdown()
            return
        key = self.keyboard.read_key() if self.keyboard is not None else None
        if key is not None:
            self._handle_key(key)
        elapsed = (self.get_clock().now() - self.last_motion_time).nanoseconds * 1e-9
        if elapsed > self.key_timeout:
            self.motion = Motion()
            self.deadman = False
        self.heartbeat_pub.publish(Empty())
        self._publish_motion()

    def _handle_key(self, key: str) -> None:
        scale = self.speed_scale
        motion: Optional[Motion] = None
        if key == "w":
            motion = Motion(x=self.linear_x * scale)
        elif key == "s":
            motion = Motion(x=-self.linear_x * scale)
        elif key == "a":
            motion = Motion(y=self.linear_y * scale)
        elif key == "d":
            motion = Motion(y=-self.linear_y * scale)
        elif key == "q":
            motion = Motion(yaw=self.angular_z * scale)
        elif key == "e":
            motion = Motion(yaw=-self.angular_z * scale)
        elif key == "UP":
            motion = Motion(gimbal_pitch=self.gimbal_pitch * scale)
        elif key == "DOWN":
            motion = Motion(gimbal_pitch=-self.gimbal_pitch * scale)
        elif key == "LEFT":
            motion = Motion(gimbal_yaw=self.gimbal_yaw * scale)
        elif key == "RIGHT":
            motion = Motion(gimbal_yaw=-self.gimbal_yaw * scale)
        elif key == " ":
            self._stop(repeat=2)
            return
        elif key == "x":
            self._stop(repeat=2)
            self.estop_pub.publish(Bool(data=True))
            self.get_logger().error("software emergency stop requested")
            return
        elif key == "c":
            self._stop(repeat=2)
            self.clear_estop_pub.publish(Empty())
            return
        elif key == "m":
            self._stop(repeat=2)
            self._publish_mode("manual")
            return
        elif key == "o":
            self._stop(repeat=2)
            self._publish_mode("auto")
            return
        elif key == "p":
            self._stop(repeat=2)
            self._publish_mode("stop")
            return
        elif key in ("+", "="):
            self._change_speed(self.speed_step)
            return
        elif key in ("-", "_"):
            self._change_speed(-self.speed_step)
            return
        elif key in ("h", "?"):
            self._print_help()
            return
        elif key == "\x03":
            self.shutdown_requested = True
            return
        if motion is not None:
            self.motion = motion
            self.deadman = True
            self.last_motion_time = self.get_clock().now()

    def _publish_motion(self) -> None:
        stamp = self.get_clock().now().to_msg()
        velocity = TwistStamped()
        velocity.header.stamp = stamp
        velocity.header.frame_id = self.frame_id
        velocity.twist.linear.x = self.motion.x
        velocity.twist.linear.y = self.motion.y
        velocity.twist.angular.z = self.motion.yaw
        self.velocity_pub.publish(velocity)

        gimbal = GimbalCmd()
        gimbal.header.stamp = stamp
        gimbal.yaw_rate = float(self.motion.gimbal_yaw)
        gimbal.pitch_rate = float(self.motion.gimbal_pitch)
        gimbal.hold_yaw = self.motion.gimbal_yaw == 0.0
        gimbal.hold_pitch = self.motion.gimbal_pitch == 0.0
        self.gimbal_pub.publish(gimbal)
        self.deadman_pub.publish(Bool(data=self.deadman))

    def _stop(self, repeat: int = 1) -> None:
        self.motion = Motion()
        self.deadman = False
        for _ in range(max(repeat, 1)):
            self.heartbeat_pub.publish(Empty())
            self._publish_motion()

    def _publish_mode(self, mode: str) -> None:
        self.mode_pub.publish(String(data=mode))
        self.get_logger().warning(f"requested control mode: {mode}")

    def _change_speed(self, delta: float) -> None:
        self.speed_scale = clamp(self.speed_scale + delta, self.min_scale, self.max_scale)
        self.get_logger().info(f"speed scale: {self.speed_scale:.2f}")

    def _print_help(self) -> None:
        self.get_logger().info(
            "W/S forward/back, A/D strafe, Q/E rotate, arrows gimbal; "
            "repeat motion key to keep enabled; Space stop, X ESTOP, "
            "C clear ESTOP, M manual, O auto, P safe stop, +/- speed, Ctrl-C quit"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardPlatformTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
