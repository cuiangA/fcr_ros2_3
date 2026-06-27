#!/usr/bin/env python3
"""Evaluate visual-servo tracking smoothness and responsiveness from rosbag playback."""

import json
import math
from collections import deque
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from vision_servo_msgs.msg import GimbalCmd, ServoState, TargetArray


@dataclass
class Sample:
    t: float
    value: float


class ServoBagEvalNode(Node):
    """Compute online metrics while a rosbag is replayed with --clock."""

    METRIC_NAMES = [
        "overall_score",
        "smooth_score",
        "response_score",
        "tracking_score",
        "distance_score",
        "visibility_score",
        "tracking_error_rms",
        "distance_error_rms",
        "cmd_delta_rms",
        "cmd_jerk_rms",
        "sign_flip_rate",
        "response_delay",
        "lost_ratio",
        "time_to_first_tracking",
    ]

    def __init__(self):
        super().__init__("servo_bag_eval_node")

        self.declare_parameter("window_sec", 5.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("desired_depth", 2.0)
        self.declare_parameter("settling_tolerance", 0.05)
        self.declare_parameter("max_response_delay", 1.0)
        self.declare_parameter("prefer_stamped_cmd", True)
        self.declare_parameter("marker_frame", "odom")
        self.declare_parameter("max_path_points", 2000)

        self.declare_parameter("servo_state_topic", "/servo/state")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_unstamped")
        self.declare_parameter("cmd_vel_stamped_topic", "/cmd_vel")
        self.declare_parameter("cmd_gimbal_topic", "/cmd_gimbal")
        self.declare_parameter("targets_topic", "/perception/targets_3d")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("target_pose_topic", "/target/pose")
        self.declare_parameter("output_prefix", "/servo_eval")

        self.declare_parameter("tracking_error_norm", 0.25)
        self.declare_parameter("distance_error_norm", 0.30)
        self.declare_parameter("cmd_delta_norm", 0.20)
        self.declare_parameter("cmd_jerk_norm", 3.0)
        self.declare_parameter("sign_flip_rate_norm", 2.0)
        self.declare_parameter("response_delay_norm", 0.30)

        self.window_sec = float(self.get_parameter("window_sec").value)
        self.desired_depth = float(self.get_parameter("desired_depth").value)
        self.settling_tolerance = float(self.get_parameter("settling_tolerance").value)
        self.max_response_delay = float(self.get_parameter("max_response_delay").value)
        self.prefer_stamped_cmd = bool(self.get_parameter("prefer_stamped_cmd").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.max_path_points = int(self.get_parameter("max_path_points").value)
        self.output_prefix = str(self.get_parameter("output_prefix").value).rstrip("/")

        self.tracking_error_samples = deque()
        self.distance_error_samples = deque()
        self.state_samples = deque()
        self.cmd_samples = {
            "base_vx": deque(),
            "base_wz": deque(),
            "gimbal_yaw_rate": deque(),
            "gimbal_pitch_rate": deque(),
        }
        self.cmd_magnitude_samples = deque()
        self.robot_path = deque(maxlen=self.max_path_points)
        self.target_path = deque(maxlen=self.max_path_points)
        self.latest_robot_point = None
        self.latest_target_point = None
        self.latest_cmd_values = {
            "base_vx": 0.0,
            "base_wz": 0.0,
            "gimbal_yaw_rate": 0.0,
            "gimbal_pitch_rate": 0.0,
        }
        self.latest_servo_tolerance = self.settling_tolerance
        self.last_sample_time = None
        self.start_time = None
        self.first_tracking_time = None
        self.received_stamped_cmd = False
        self.last_metrics = {}

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            ServoState,
            str(self.get_parameter("servo_state_topic").value),
            self.servo_state_callback,
            qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_vel_callback,
            qos,
        )
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter("cmd_vel_stamped_topic").value),
            self.cmd_vel_stamped_callback,
            qos,
        )
        self.create_subscription(
            GimbalCmd,
            str(self.get_parameter("cmd_gimbal_topic").value),
            self.cmd_gimbal_callback,
            qos,
        )
        self.create_subscription(
            TargetArray,
            str(self.get_parameter("targets_topic").value),
            self.targets_callback,
            qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("target_pose_topic").value),
            self.target_pose_callback,
            qos,
        )

        self.float_pubs = {
            name: self.create_publisher(Float32, self.topic(name), 10)
            for name in self.METRIC_NAMES
        }
        self.metrics_pub = self.create_publisher(Float32MultiArray, self.topic("metrics"), 10)
        self.summary_pub = self.create_publisher(String, self.topic("summary"), 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.topic("markers"), 10)

        publish_rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_metrics)

        self.get_logger().info(
            "Servo bag evaluator started. Replay with: ros2 bag play <bag> --clock"
        )

    def topic(self, name):
        return f"{self.output_prefix}/{name}"

    def msg_time(self, header=None):
        if header is not None:
            stamp = header.stamp
            if stamp.sec != 0 or stamp.nanosec != 0:
                return float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return self.get_clock().now().nanoseconds * 1e-9

    def reset_for_time_jump(self, t):
        if self.last_sample_time is not None and t + 0.5 < self.last_sample_time:
            self.tracking_error_samples.clear()
            self.distance_error_samples.clear()
            self.state_samples.clear()
            for samples in self.cmd_samples.values():
                samples.clear()
            self.cmd_magnitude_samples.clear()
            self.robot_path.clear()
            self.target_path.clear()
            self.latest_robot_point = None
            self.latest_target_point = None
            self.start_time = None
            self.first_tracking_time = None
            self.get_logger().warn("Detected bag time jump backwards; metrics reset.")
        self.last_sample_time = max(t, self.last_sample_time or t)
        if self.start_time is None:
            self.start_time = t

    def append_sample(self, samples, t, value):
        if not math.isfinite(t) or not math.isfinite(value):
            return
        self.reset_for_time_jump(t)
        samples.append(Sample(t, float(value)))
        self.prune(samples, t)

    def prune(self, samples, now_t):
        cutoff = now_t - self.window_sec
        while samples and samples[0].t < cutoff:
            samples.popleft()

    def servo_state_callback(self, msg):
        t = self.msg_time(msg.header)
        self.append_sample(self.tracking_error_samples, t, float(msg.norm_error))
        self.append_sample(self.state_samples, t, float(msg.state))
        if msg.desired_norm_error > 0.0:
            self.latest_servo_tolerance = float(msg.desired_norm_error)
        if msg.state == ServoState.TRACKING and self.first_tracking_time is None:
            self.first_tracking_time = t

    def cmd_vel_callback(self, msg):
        if self.prefer_stamped_cmd and self.received_stamped_cmd:
            return
        t = self.msg_time()
        self.record_base_command(t, msg.linear.x, msg.angular.z)

    def cmd_vel_stamped_callback(self, msg):
        self.received_stamped_cmd = True
        t = self.msg_time(msg.header)
        self.record_base_command(t, msg.twist.linear.x, msg.twist.angular.z)

    def cmd_gimbal_callback(self, msg):
        t = self.msg_time(msg.header)
        yaw_rate = 0.0 if msg.hold_yaw else float(msg.yaw_rate)
        pitch_rate = 0.0 if msg.hold_pitch else float(msg.pitch_rate)
        self.append_command("gimbal_yaw_rate", t, yaw_rate)
        self.append_command("gimbal_pitch_rate", t, pitch_rate)
        self.append_command_magnitude(t)

    def targets_callback(self, msg):
        t = self.msg_time(msg.header)
        target = self.select_target(msg)
        if target is None:
            return
        depth = float(target.position[2])
        if depth > 0.0 and math.isfinite(depth):
            self.append_sample(self.distance_error_samples, t, depth - self.desired_depth)

    def odom_callback(self, msg):
        t = self.msg_time(msg.header)
        self.reset_for_time_jump(t)
        point = Point()
        point.x = float(msg.pose.pose.position.x)
        point.y = float(msg.pose.pose.position.y)
        point.z = float(msg.pose.pose.position.z)
        self.latest_robot_point = point
        self.robot_path.append(point)

    def target_pose_callback(self, msg):
        t = self.msg_time(msg.header)
        self.reset_for_time_jump(t)
        point = Point()
        point.x = float(msg.pose.position.x)
        point.y = float(msg.pose.position.y)
        point.z = float(msg.pose.position.z)
        self.latest_target_point = point
        self.target_path.append(point)

    def record_base_command(self, t, vx, wz):
        self.append_command("base_vx", t, vx)
        self.append_command("base_wz", t, wz)
        self.append_command_magnitude(t)

    def append_command(self, name, t, value):
        self.latest_cmd_values[name] = float(value)
        self.append_sample(self.cmd_samples[name], t, float(value))

    def append_command_magnitude(self, t):
        values = self.latest_cmd_values.values()
        magnitude = math.sqrt(sum(value * value for value in values))
        self.append_sample(self.cmd_magnitude_samples, t, magnitude)

    def select_target(self, msg):
        if not msg.targets:
            return None
        if msg.tracking_id >= 0:
            for target in msg.targets:
                if target.id == msg.tracking_id:
                    return target
        return max(msg.targets, key=lambda target: float(target.confidence))

    def publish_metrics(self):
        now_t = self.current_eval_time()
        if now_t is None:
            return

        for samples in [
            self.tracking_error_samples,
            self.distance_error_samples,
            self.state_samples,
            self.cmd_magnitude_samples,
        ]:
            self.prune(samples, now_t)
        for samples in self.cmd_samples.values():
            self.prune(samples, now_t)

        cmd_stats = self.compute_command_stats()
        tracking_error_rms = self.rms_from_samples(self.tracking_error_samples)
        distance_error_rms = self.rms_from_samples(self.distance_error_samples)
        lost_ratio = self.compute_lost_ratio()
        response_delay = self.estimate_response_delay()

        tracking_score = self.score_from_rms(
            tracking_error_rms,
            float(self.get_parameter("tracking_error_norm").value),
        )
        distance_score = self.score_from_rms(
            distance_error_rms,
            float(self.get_parameter("distance_error_norm").value),
        )
        smooth_score = self.compute_smooth_score(cmd_stats)
        response_score = self.compute_response_score(response_delay)
        visibility_score = max(0.0, min(100.0, 100.0 * (1.0 - lost_ratio)))
        overall_score = (
            0.30 * tracking_score
            + 0.20 * distance_score
            + 0.25 * smooth_score
            + 0.15 * response_score
            + 0.10 * visibility_score
        )
        time_to_first_tracking = (
            self.first_tracking_time - self.start_time
            if self.first_tracking_time is not None and self.start_time is not None
            else math.nan
        )

        metrics = {
            "overall_score": overall_score,
            "smooth_score": smooth_score,
            "response_score": response_score,
            "tracking_score": tracking_score,
            "distance_score": distance_score,
            "visibility_score": visibility_score,
            "tracking_error_rms": tracking_error_rms,
            "distance_error_rms": distance_error_rms,
            "cmd_delta_rms": cmd_stats["delta_rms"],
            "cmd_jerk_rms": cmd_stats["jerk_rms"],
            "sign_flip_rate": cmd_stats["sign_flip_rate"],
            "response_delay": response_delay,
            "lost_ratio": lost_ratio,
            "time_to_first_tracking": time_to_first_tracking,
        }
        self.last_metrics = metrics

        for name, value in metrics.items():
            self.publish_float(name, value)

        array_msg = Float32MultiArray()
        array_msg.data = [float(metrics[name]) for name in self.METRIC_NAMES]
        self.metrics_pub.publish(array_msg)

        summary = String()
        summary.data = json.dumps(metrics, ensure_ascii=False, allow_nan=True)
        self.summary_pub.publish(summary)
        self.publish_markers()

    def current_eval_time(self):
        candidates = []
        for samples in [
            self.tracking_error_samples,
            self.distance_error_samples,
            self.state_samples,
            self.cmd_magnitude_samples,
        ]:
            if samples:
                candidates.append(samples[-1].t)
        for samples in self.cmd_samples.values():
            if samples:
                candidates.append(samples[-1].t)
        if candidates:
            return max(candidates)
        return None

    def compute_command_stats(self):
        all_deltas = []
        all_jerks = []
        total_flips = 0
        total_duration = 0.0
        sign_eps = 1e-4

        for samples in self.cmd_samples.values():
            if len(samples) < 2:
                continue
            values = list(samples)
            total_duration += max(0.0, values[-1].t - values[0].t)

            rates = []
            for prev, cur in zip(values[:-1], values[1:]):
                dt = cur.t - prev.t
                if dt <= 1e-6:
                    continue
                delta = cur.value - prev.value
                all_deltas.append(delta)
                rates.append(Sample(0.5 * (prev.t + cur.t), delta / dt))
                if (
                    abs(prev.value) > sign_eps
                    and abs(cur.value) > sign_eps
                    and prev.value * cur.value < 0.0
                ):
                    total_flips += 1

            for prev, cur in zip(rates[:-1], rates[1:]):
                dt = cur.t - prev.t
                if dt > 1e-6:
                    all_jerks.append((cur.value - prev.value) / dt)

        delta_rms = self.rms(all_deltas)
        jerk_rms = self.rms(all_jerks)
        sign_flip_rate = total_flips / total_duration if total_duration > 1e-6 else 0.0
        return {
            "delta_rms": delta_rms,
            "jerk_rms": jerk_rms,
            "sign_flip_rate": sign_flip_rate,
        }

    def compute_lost_ratio(self):
        if not self.state_samples:
            return 1.0
        lost_count = sum(1 for sample in self.state_samples if int(sample.value) == ServoState.LOST)
        return lost_count / float(len(self.state_samples))

    def estimate_response_delay(self):
        errors = list(self.tracking_error_samples)
        commands = list(self.cmd_magnitude_samples)
        if len(errors) < 5 or len(commands) < 5:
            return math.nan

        cmd_start = commands[0].t
        cmd_end = commands[-1].t
        step = max(0.02, self.max_response_delay / 20.0)
        best_lag = math.nan
        best_corr = -2.0

        lag = 0.0
        while lag <= self.max_response_delay + 1e-9:
            pairs = []
            for error in errors:
                cmd_t = error.t + lag
                if cmd_t < cmd_start or cmd_t > cmd_end:
                    continue
                cmd_value = self.interpolate(commands, cmd_t)
                if cmd_value is not None:
                    pairs.append((abs(error.value), cmd_value))

            corr = self.correlation(pairs)
            if corr is not None and corr > best_corr:
                best_corr = corr
                best_lag = lag
            lag += step

        if best_corr < 0.1:
            return math.nan
        return best_lag

    def publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        if self.robot_path:
            markers.markers.append(
                self.make_line_marker(0, "robot_path", list(self.robot_path), stamp, 0.0, 0.75, 0.25)
            )
        if self.target_path:
            markers.markers.append(
                self.make_line_marker(1, "target_path", list(self.target_path), stamp, 1.0, 0.15, 0.05)
            )
        if self.latest_target_point is not None:
            markers.markers.append(self.make_desired_distance_marker(stamp))
        markers.markers.append(self.make_score_text_marker(stamp))
        self.marker_pub.publish(markers)

    def make_line_marker(self, marker_id, ns, points, stamp, r, g, b):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.marker_frame
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 0.95
        marker.points = points
        return marker

    def make_desired_distance_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.marker_frame
        marker.ns = "desired_distance"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.025
        marker.color.r = 0.2
        marker.color.g = 0.55
        marker.color.b = 1.0
        marker.color.a = 0.7
        center = self.latest_target_point
        points = []
        for idx in range(73):
            angle = 2.0 * math.pi * idx / 72.0
            point = Point()
            point.x = center.x + self.desired_depth * math.cos(angle)
            point.y = center.y + self.desired_depth * math.sin(angle)
            point.z = center.z
            points.append(point)
        marker.points = points
        return marker

    def make_score_text_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.marker_frame
        marker.ns = "score_text"
        marker.id = 3
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        base = self.latest_robot_point or self.latest_target_point
        marker.pose.position.x = base.x if base is not None else 0.0
        marker.pose.position.y = base.y if base is not None else 0.0
        marker.pose.position.z = (base.z if base is not None else 0.0) + 0.8
        marker.text = (
            f"overall {self.last_metrics.get('overall_score', math.nan):.1f} | "
            f"smooth {self.last_metrics.get('smooth_score', math.nan):.1f} | "
            f"response {self.last_metrics.get('response_score', math.nan):.1f}"
        )
        return marker

    def publish_float(self, name, value):
        msg = Float32()
        msg.data = float(value)
        self.float_pubs[name].publish(msg)

    @staticmethod
    def interpolate(samples, t):
        if not samples:
            return None
        if t < samples[0].t or t > samples[-1].t:
            return None
        for prev, cur in zip(samples[:-1], samples[1:]):
            if prev.t <= t <= cur.t:
                dt = cur.t - prev.t
                if dt <= 1e-9:
                    return cur.value
                ratio = (t - prev.t) / dt
                return prev.value + ratio * (cur.value - prev.value)
        return samples[-1].value

    @staticmethod
    def correlation(pairs):
        if len(pairs) < 5:
            return None
        xs = [pair[0] for pair in pairs]
        ys = [pair[1] for pair in pairs]
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        num = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
        den_x = math.sqrt(sum((x - mean_x) ** 2 for x in xs))
        den_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys))
        if den_x <= 1e-9 or den_y <= 1e-9:
            return None
        return num / (den_x * den_y)

    @staticmethod
    def rms(values):
        values = [value for value in values if math.isfinite(value)]
        if not values:
            return math.nan
        return math.sqrt(sum(value * value for value in values) / len(values))

    def rms_from_samples(self, samples):
        return self.rms([sample.value for sample in samples])

    @staticmethod
    def score_from_rms(value, norm):
        if not math.isfinite(value):
            return 0.0
        norm = max(float(norm), 1e-6)
        return max(0.0, min(100.0, 100.0 / (1.0 + value / norm)))

    def compute_smooth_score(self, cmd_stats):
        delta_norm = max(float(self.get_parameter("cmd_delta_norm").value), 1e-6)
        jerk_norm = max(float(self.get_parameter("cmd_jerk_norm").value), 1e-6)
        flip_norm = max(float(self.get_parameter("sign_flip_rate_norm").value), 1e-6)
        delta = 0.0 if not math.isfinite(cmd_stats["delta_rms"]) else cmd_stats["delta_rms"]
        jerk = 0.0 if not math.isfinite(cmd_stats["jerk_rms"]) else cmd_stats["jerk_rms"]
        flips = cmd_stats["sign_flip_rate"]
        penalty = delta / delta_norm + jerk / jerk_norm + flips / flip_norm
        return max(0.0, min(100.0, 100.0 / (1.0 + penalty)))

    def compute_response_score(self, response_delay):
        if not math.isfinite(response_delay):
            return 50.0
        norm = max(float(self.get_parameter("response_delay_norm").value), 1e-6)
        return max(0.0, min(100.0, 100.0 / (1.0 + response_delay / norm)))


def main(args=None):
    rclpy.init(args=args)
    node = ServoBagEvalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
