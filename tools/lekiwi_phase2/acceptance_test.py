#!/usr/bin/env python3
"""Interactive ROS 2 low-speed, wheels-off-ground LeKiwi acceptance test."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class AcceptanceNode(Node):
    def __init__(self) -> None:
        super().__init__("lekiwi_phase2_acceptance")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(TwistStamped, "/cmd_vel", qos)
        self.subscription = self.create_subscription(
            Odometry, "/chassis/odom_raw", self._odom_callback, QoSProfile(depth=10)
        )
        self.latest_odom: Odometry | None = None
        self.odom_count = 0

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_count += 1

    def publish(self, vx: float, vy: float, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self.publisher.publish(msg)

    def spin_for(self, seconds: float, command: tuple[float, float, float] | None = None) -> list[dict]:
        samples: list[dict] = []
        end = time.monotonic() + seconds
        next_publish = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if command is not None and now >= next_publish:
                self.publish(*command)
                next_publish = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.latest_odom is not None:
                twist = self.latest_odom.twist.twist
                samples.append(
                    {"vx": twist.linear.x, "vy": twist.linear.y, "wz": twist.angular.z}
                )
        return samples

    def send_zero(self) -> None:
        for _ in range(5):
            self.publish(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def peak_sample(samples: list[dict]) -> dict:
    if not samples:
        return {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    return max(samples, key=lambda s: math.sqrt(s["vx"] ** 2 + s["vy"] ** 2 + s["wz"] ** 2))


def stopped(sample: dict, linear_epsilon: float = 0.015, angular_epsilon: float = 0.08) -> bool:
    return (
        abs(sample["vx"]) <= linear_epsilon
        and abs(sample["vy"]) <= linear_epsilon
        and abs(sample["wz"]) <= angular_epsilon
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--confirm-wheels-off-ground", action="store_true", required=True)
    parser.add_argument("--report", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "motions": [],
        "watchdog": {},
        "result": "FAILED",
    }
    rclpy.init()
    node = AcceptanceNode()
    try:
        print("Waiting for /chassis/odom_raw...")
        node.spin_for(3.0)
        if node.odom_count == 0:
            raise RuntimeError("no /chassis/odom_raw received within 3 seconds")

        tests = (
            ("forward", (0.02, 0.0, 0.0), "向前"),
            ("backward", (-0.02, 0.0, 0.0), "向后"),
            ("left", (0.0, 0.02, 0.0), "向左平移"),
            ("right", (0.0, -0.02, 0.0), "向右平移"),
            ("rotate_ccw", (0.0, 0.0, 0.15), "逆时针旋转"),
            ("rotate_cw", (0.0, 0.0, -0.15), "顺时针旋转"),
        )
        print("\n每项运行0.4秒。按 ENTER执行；Ctrl+C会发送零速度。")
        for name, command, expected in tests:
            input(f"[{name}] 预期机体{expected}。确认安全后按 ENTER：")
            moving = node.spin_for(0.4, command)
            node.send_zero()
            stopped_samples = node.spin_for(0.35)
            post_stop = stopped_samples[-1] if stopped_samples else peak_sample([])
            observed = input(f"实际是否为“{expected}”？输入 y 或问题描述：").strip()
            accepted = observed.lower() in {"y", "yes", "是"}
            item = {
                "name": name,
                "command": {"vx": command[0], "vy": command[1], "wz": command[2]},
                "peak_feedback": peak_sample(moving),
                "post_stop_feedback": post_stop,
                "operator_response": observed,
                "direction_pass": accepted,
                "stop_pass": stopped(post_stop),
            }
            report["motions"].append(item)
            if not accepted:
                raise RuntimeError(f"operator rejected direction test: {name}: {observed}")
            if not item["stop_pass"]:
                raise RuntimeError(f"base did not stop after explicit zero: {name}: {post_stop}")

        input("[watchdog] 将前进0.3秒，然后停止发布并等待驱动自动停车。按 ENTER：")
        node.spin_for(0.3, (0.02, 0.0, 0.0))
        watchdog_samples = node.spin_for(0.8)
        watchdog_final = watchdog_samples[-1] if watchdog_samples else peak_sample([])
        watchdog_pass = stopped(watchdog_final)
        report["watchdog"] = {
            "expected_timeout_ms": 250,
            "final_feedback": watchdog_final,
            "pass": watchdog_pass,
        }
        if not watchdog_pass:
            raise RuntimeError(f"command watchdog did not stop the base: {watchdog_final}")

        node.send_zero()
        report["result"] = "PASS"
        print("All six directions and the command watchdog passed.")
        return 0
    except KeyboardInterrupt:
        report["result"] = "ABORTED"
        print("\nABORTED: sending zero velocity.", file=sys.stderr)
        return 130
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        print(f"FAIL: {report['error']}", file=sys.stderr)
        return 1
    finally:
        try:
            node.send_zero()
        finally:
            node.destroy_node()
            rclpy.shutdown()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"Report written to {args.report}")


if __name__ == "__main__":
    raise SystemExit(main())
