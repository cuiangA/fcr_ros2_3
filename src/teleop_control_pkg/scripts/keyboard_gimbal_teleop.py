#!/usr/bin/env python3
"""Keyboard teleoperation for the gimbal velocity command topic.

Keys:
  w/s: pitch up/down
  a/d: yaw left/right
  space: stop and hold
  +/-: change speed scale
  q: publish stop and quit

TODO: Add a home/recenter command once the platform layer exposes a gimbal home
service or action. GimbalCmd currently only carries velocity/hold semantics.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from vision_servo_msgs.msg import GimbalCmd


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class GimbalCommandState:
    yaw_rate: float = 0.0
    pitch_rate: float = 0.0
    hold_yaw: bool = True
    hold_pitch: bool = True


class PosixKeyboardReader:
    """Read one key from a POSIX terminal without blocking."""

    def __init__(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard teleop requires an interactive terminal")

        import termios
        import tty

        self._termios = termios
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def read_key(self) -> Optional[str]:
        import select

        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        return sys.stdin.read(1)

    def close(self) -> None:
        self._termios.tcsetattr(sys.stdin, self._termios.TCSADRAIN, self._old_settings)


class KeyboardGimbalTeleop(Node):
    """Publish GimbalCmd from simple keyboard input."""

    def __init__(self) -> None:
        super().__init__("keyboard_gimbal_teleop")

        self.declare_parameter("cmd_gimbal_topic", "/cmd_gimbal")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("key_timeout_sec", 0.35)
        self.declare_parameter("yaw_rate", 0.25)
        self.declare_parameter("pitch_rate", 0.20)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("speed_step", 0.25)
        self.declare_parameter("min_speed_scale", 0.25)
        self.declare_parameter("max_speed_scale", 3.0)
        self.declare_parameter("yaw_direction_sign", 1.0)
        self.declare_parameter("pitch_direction_sign", 1.0)
        self.declare_parameter("hold_on_idle", True)
        self.declare_parameter("frame_id", "gimbal_link")

        self.cmd_topic = str(self.get_parameter("cmd_gimbal_topic").value)
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.key_timeout_sec = max(float(self.get_parameter("key_timeout_sec").value), 0.05)
        self.base_yaw_rate = abs(float(self.get_parameter("yaw_rate").value))
        self.base_pitch_rate = abs(float(self.get_parameter("pitch_rate").value))
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.speed_step = abs(float(self.get_parameter("speed_step").value))
        self.min_speed_scale = max(float(self.get_parameter("min_speed_scale").value), 0.01)
        self.max_speed_scale = max(float(self.get_parameter("max_speed_scale").value), self.min_speed_scale)
        self.yaw_direction_sign = self._sign(float(self.get_parameter("yaw_direction_sign").value))
        self.pitch_direction_sign = self._sign(float(self.get_parameter("pitch_direction_sign").value))
        self.hold_on_idle = bool(self.get_parameter("hold_on_idle").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.speed_scale = clamp(self.speed_scale, self.min_speed_scale, self.max_speed_scale)

        self.publisher = self.create_publisher(GimbalCmd, self.cmd_topic, 10)
        self.command = GimbalCommandState()
        self.last_key_time = self.get_clock().now()
        self.shutdown_requested = False

        self.keyboard: Optional[PosixKeyboardReader] = None
        try:
            self.keyboard = PosixKeyboardReader()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            self.get_logger().error("run this node with `ros2 run`, not from a non-interactive shell")
            self.shutdown_requested = True

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self._tick)

        self._print_help()

    def close(self) -> None:
        if rclpy.ok():
            self._publish_stop(repeat=3)
        if self.keyboard is not None:
            self.keyboard.close()

    def _tick(self) -> None:
        if self.shutdown_requested:
            self._publish_stop(repeat=3)
            rclpy.shutdown()
            return

        key = self.keyboard.read_key() if self.keyboard is not None else None
        if key is not None:
            self._handle_key(key)
            if self.shutdown_requested:
                self._publish_stop(repeat=3)
                rclpy.shutdown()
                return

        if self._key_timed_out():
            self.command = self._idle_command()

        self._publish_command(self.command)

    def _handle_key(self, key: str) -> None:
        now = self.get_clock().now()
        self.last_key_time = now

        yaw = 0.0
        pitch = 0.0

        if key == "a":
            yaw = self.yaw_direction_sign * self.base_yaw_rate * self.speed_scale
        elif key == "d":
            yaw = -self.yaw_direction_sign * self.base_yaw_rate * self.speed_scale
        elif key == "w":
            pitch = self.pitch_direction_sign * self.base_pitch_rate * self.speed_scale
        elif key == "s":
            pitch = -self.pitch_direction_sign * self.base_pitch_rate * self.speed_scale
        elif key in (" ", "\x00"):
            self.command = self._idle_command()
            self._publish_stop(repeat=2)
            self.get_logger().info("stop")
            return
        elif key in ("+", "="):
            self._change_speed(+self.speed_step)
            return
        elif key in ("-", "_"):
            self._change_speed(-self.speed_step)
            return
        elif key in ("?", "h"):
            self._print_help()
            return
        elif key in ("q", "\x03"):
            self.get_logger().info("quit requested, sending stop")
            self.shutdown_requested = True
            return
        else:
            return

        self.command = GimbalCommandState(
            yaw_rate=float(yaw),
            pitch_rate=float(pitch),
            hold_yaw=math.isclose(yaw, 0.0, abs_tol=1e-9),
            hold_pitch=math.isclose(pitch, 0.0, abs_tol=1e-9),
        )

    def _change_speed(self, delta: float) -> None:
        self.speed_scale = clamp(self.speed_scale + delta, self.min_speed_scale, self.max_speed_scale)
        self.get_logger().info(
            f"speed_scale={self.speed_scale:.2f}, "
            f"yaw={self.base_yaw_rate * self.speed_scale:.3f} rad/s, "
            f"pitch={self.base_pitch_rate * self.speed_scale:.3f} rad/s")

    def _key_timed_out(self) -> bool:
        elapsed = (self.get_clock().now() - self.last_key_time).nanoseconds * 1e-9
        return elapsed > self.key_timeout_sec

    def _idle_command(self) -> GimbalCommandState:
        return GimbalCommandState(
            yaw_rate=0.0,
            pitch_rate=0.0,
            hold_yaw=self.hold_on_idle,
            hold_pitch=self.hold_on_idle,
        )

    def _publish_command(self, state: GimbalCommandState) -> None:
        msg = GimbalCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.yaw_rate = float(state.yaw_rate)
        msg.pitch_rate = float(state.pitch_rate)
        msg.hold_yaw = bool(state.hold_yaw)
        msg.hold_pitch = bool(state.hold_pitch)
        self.publisher.publish(msg)

    def _publish_stop(self, repeat: int = 1) -> None:
        stop = self._idle_command()
        for _ in range(max(repeat, 1)):
            self._publish_command(stop)

    def _print_help(self) -> None:
        self.get_logger().info(
            "keyboard gimbal teleop | w/s=pitch, a/d=yaw, space=stop, "
            "+/-=speed, h/?=help, q=quit")

    @staticmethod
    def _sign(value: float) -> float:
        return 1.0 if value >= 0.0 else -1.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardGimbalTeleop()
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
