#!/usr/bin/env python3
"""A lightweight 2D closed-loop simulator for the MVP follow controller."""

import math

import rclpy
from geometry_msgs.msg import Point, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from vision_servo_msgs.msg import (
    GimbalCmd,
    PlatformState,
    ShotReference,
    Target,
    TargetArray,
)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_quaternion(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def pitch_quaternion(pitch):
    return 0.0, math.sin(0.5 * pitch), 0.0, math.cos(0.5 * pitch)


def rpy_quaternion(roll, pitch, yaw):
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class SimpleRobotSim(Node):
    def __init__(self):
        super().__init__("simple_robot_sim_node")

        self.declare_parameter("target_topic", "/target/current")
        self.declare_parameter("platform_state_topic", "/platform/state")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_gimbal_topic", "/cmd_gimbal")
        self.declare_parameter("shot_reference_topic", "/shot/reference")
        self.declare_parameter("use_twist_stamped", True)

        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("desired_distance", 2.0)
        self.declare_parameter("horizontal_fov_deg", 70.0)

        self.declare_parameter("initial_robot_x", 0.0)
        self.declare_parameter("initial_robot_y", 0.0)
        self.declare_parameter("initial_robot_yaw", 0.0)
        self.declare_parameter("initial_gimbal_yaw", 0.0)
        self.declare_parameter("initial_gimbal_pitch", 0.0)
        self.declare_parameter("target_x", 3.0)
        self.declare_parameter("target_y", 1.0)
        self.declare_parameter("target_motion", "circle")
        self.declare_parameter("target_speed", 0.2)
        self.declare_parameter("target_radius", 1.0)
        self.declare_parameter("target_line_heading_deg", 0.0)
        self.declare_parameter("target_line_loop_distance", 0.0)

        self.declare_parameter("gimbal_yaw_min", -3.14)
        self.declare_parameter("gimbal_yaw_max", 3.14)
        self.declare_parameter("gimbal_pitch_min", -1.57)
        self.declare_parameter("gimbal_pitch_max", 1.57)

        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/visualization_marker")
        self.declare_parameter("marker_array_topic", "/visualization_marker_array")

        self.robot_x = float(self.get_parameter("initial_robot_x").value)
        self.robot_y = float(self.get_parameter("initial_robot_y").value)
        self.robot_yaw = float(self.get_parameter("initial_robot_yaw").value)
        self.gimbal_yaw = float(self.get_parameter("initial_gimbal_yaw").value)
        self.gimbal_pitch = float(self.get_parameter("initial_gimbal_pitch").value)

        self.base_vx = 0.0
        self.base_wz = 0.0
        self.gimbal_yaw_rate = 0.0
        self.gimbal_pitch_rate = 0.0
        self.path_points = []
        self.target_path_points = []
        self.current_marker_array = None
        self.latest_shot_reference = None
        self.reported_unknown_target_motion = False

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        target_topic = self.get_parameter("target_topic").value
        platform_topic = self.get_parameter("platform_state_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        cmd_gimbal_topic = self.get_parameter("cmd_gimbal_topic").value
        shot_reference_topic = self.get_parameter("shot_reference_topic").value
        use_twist_stamped = bool(self.get_parameter("use_twist_stamped").value)

        if use_twist_stamped:
            self.cmd_vel_sub = self.create_subscription(
                TwistStamped, cmd_vel_topic, self.cmd_vel_stamped_callback, command_qos
            )
        else:
            self.cmd_vel_sub = self.create_subscription(
                Twist, cmd_vel_topic, self.cmd_vel_callback, command_qos
            )
        self.cmd_gimbal_sub = self.create_subscription(
            GimbalCmd, cmd_gimbal_topic, self.cmd_gimbal_callback, command_qos
        )
        self.shot_reference_sub = self.create_subscription(
            ShotReference,
            shot_reference_topic,
            self.shot_reference_callback,
            command_qos,
        )

        self.target_pub = self.create_publisher(TargetArray, target_topic, command_qos)
        self.platform_pub = self.create_publisher(PlatformState, platform_topic, state_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", command_qos)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", command_qos)
        marker_topic = self.get_parameter("marker_topic").value
        marker_array_topic = self.get_parameter("marker_array_topic").value
        self.marker_pub = self.create_publisher(Marker, marker_topic, command_qos)
        self.marker_array_pub = self.create_publisher(MarkerArray, marker_array_topic, command_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.start_time = self.last_time
        update_rate_hz = max(float(self.get_parameter("update_rate_hz").value), 1.0)
        self.default_dt = 1.0 / update_rate_hz
        self.timer = self.create_timer(self.default_dt, self.update)

        self.get_logger().info(
            f"Simple 2D robot sim started: target={target_topic}, platform={platform_topic}, "
            f"cmd_vel={cmd_vel_topic} ({'TwistStamped' if use_twist_stamped else 'Twist'}), "
            f"markers={marker_topic}/{marker_array_topic}"
        )

    def cmd_vel_stamped_callback(self, msg):
        self.store_twist(msg.twist)

    def cmd_vel_callback(self, msg):
        self.store_twist(msg)

    def store_twist(self, twist):
        self.base_vx = float(twist.linear.x)
        self.base_wz = float(twist.angular.z)

    def cmd_gimbal_callback(self, msg):
        self.gimbal_yaw_rate = 0.0 if msg.hold_yaw else float(msg.yaw_rate)
        self.gimbal_pitch_rate = 0.0 if msg.hold_pitch else float(msg.pitch_rate)

    def shot_reference_callback(self, msg):
        self.latest_shot_reference = msg

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.2:
            dt = self.default_dt
        self.last_time = now

        self.integrate(dt)
        observation = self.compute_observation(now)

        self.publish_target(now, observation)
        self.publish_platform_state(now)

        if bool(self.get_parameter("publish_odom").value):
            self.publish_odom(now)
        if bool(self.get_parameter("publish_joint_states").value):
            self.publish_joint_states(now)
        if bool(self.get_parameter("publish_tf").value):
            self.publish_tf(now)
        if bool(self.get_parameter("publish_markers").value):
            self.publish_markers(now)

    def integrate(self, dt):
        self.robot_x += self.base_vx * math.cos(self.robot_yaw) * dt
        self.robot_y += self.base_vx * math.sin(self.robot_yaw) * dt
        self.robot_yaw = wrap_pi(self.robot_yaw + self.base_wz * dt)

        yaw_min = float(self.get_parameter("gimbal_yaw_min").value)
        yaw_max = float(self.get_parameter("gimbal_yaw_max").value)
        pitch_min = float(self.get_parameter("gimbal_pitch_min").value)
        pitch_max = float(self.get_parameter("gimbal_pitch_max").value)
        self.gimbal_yaw = clamp(self.gimbal_yaw + self.gimbal_yaw_rate * dt, yaw_min, yaw_max)
        self.gimbal_pitch = clamp(
            self.gimbal_pitch + self.gimbal_pitch_rate * dt, pitch_min, pitch_max
        )

    def current_target_position(self, now):
        center_x = float(self.get_parameter("target_x").value)
        center_y = float(self.get_parameter("target_y").value)
        motion = str(self.get_parameter("target_motion").value).lower()
        speed = max(float(self.get_parameter("target_speed").value), 0.0)
        radius = max(float(self.get_parameter("target_radius").value), 0.0)
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        phase = speed * elapsed

        if motion == "static" or speed <= 1e-6:
            return center_x, center_y
        if motion in ("linear", "straight"):
            heading = math.radians(float(self.get_parameter("target_line_heading_deg").value))
            distance = speed * elapsed
            loop_distance = max(
                float(self.get_parameter("target_line_loop_distance").value),
                0.0,
            )
            if loop_distance > 1e-6:
                distance = math.fmod(distance, loop_distance)
            return (
                center_x + distance * math.cos(heading),
                center_y + distance * math.sin(heading),
            )
        if radius <= 1e-6:
            return center_x, center_y
        if motion == "line":
            return center_x + radius * math.sin(phase), center_y
        if motion == "figure8":
            return (
                center_x + radius * math.sin(phase),
                center_y + 0.5 * radius * math.sin(2.0 * phase),
            )
        if motion != "circle" and not self.reported_unknown_target_motion:
            self.get_logger().warn(
                f"Unknown target_motion '{motion}', falling back to circle."
            )
            self.reported_unknown_target_motion = True

        return center_x + radius * math.cos(phase), center_y + radius * math.sin(phase)

    def compute_observation(self, now):
        target_x, target_y = self.current_target_position(now)
        image_width = float(self.get_parameter("image_width").value)
        image_height = float(self.get_parameter("image_height").value)
        half_fov = math.radians(float(self.get_parameter("horizontal_fov_deg").value)) * 0.5

        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        xb = cos_yaw * dx + sin_yaw * dy
        yb = -sin_yaw * dx + cos_yaw * dy

        bearing = math.atan2(yb, xb)
        image_angle = wrap_pi(self.gimbal_yaw - bearing)
        ex = image_angle / half_fov if half_fov > 1e-6 else 0.0
        depth = math.hypot(xb, yb)
        valid = xb > 0.05 and abs(ex) <= 1.0

        cx = 0.5 * image_width + ex * 0.5 * image_width
        cy = 0.5 * image_height
        optical_x = depth * math.sin(image_angle)
        optical_z = depth * math.cos(image_angle)

        return {
            "valid": valid,
            "cx": cx,
            "cy": cy,
            "ex": ex,
            "depth": depth,
            "optical_x": optical_x,
            "optical_z": optical_z,
            "xb": xb,
            "yb": yb,
            "target_x": target_x,
            "target_y": target_y,
        }

    def publish_target(self, now, observation):
        msg = TargetArray()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "camera_optical_link"

        if not observation["valid"]:
            msg.tracking_id = -1
            self.target_pub.publish(msg)
            return

        image_width = float(self.get_parameter("image_width").value)
        image_height = float(self.get_parameter("image_height").value)
        cx = observation["cx"]
        cy = observation["cy"]
        depth = observation["depth"]

        bbox_w = 0.125 * image_width
        bbox_h = 0.25 * image_height

        target = Target()
        target.header = msg.header
        target.id = 0
        target.class_name = "sim_target"
        target.center = [float(cx), float(cy)]
        target.bbox = [
            float(clamp(cx - 0.5 * bbox_w, 0.0, image_width)),
            float(clamp(cy - 0.5 * bbox_h, 0.0, image_height)),
            float(clamp(cx + 0.5 * bbox_w, 0.0, image_width)),
            float(clamp(cy + 0.5 * bbox_h, 0.0, image_height)),
        ]
        target.width = float(target.bbox[2] - target.bbox[0])
        target.height = float(target.bbox[3] - target.bbox[1])
        target.confidence = 1.0
        target.position = [
            float(observation["optical_x"]),
            0.0,
            float(observation["optical_z"]),
        ]
        target.velocity = [0.0, 0.0, 0.0]
        target.depth_confidence = 1.0

        msg.targets = [target]
        msg.tracking_id = target.id
        self.target_pub.publish(msg)

    def publish_platform_state(self, now):
        msg = PlatformState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.chassis_pose = [float(self.robot_x), float(self.robot_y), float(self.robot_yaw)]
        msg.chassis_velocity = [float(self.base_vx), 0.0, float(self.base_wz)]
        msg.battery_voltage = 24.0
        msg.gimbal_yaw = float(self.gimbal_yaw)
        msg.gimbal_pitch = float(self.gimbal_pitch)
        msg.gimbal_yaw_rate = float(self.gimbal_yaw_rate)
        msg.gimbal_pitch_rate = float(self.gimbal_pitch_rate)
        msg.angular_velocity = [0.0, 0.0, float(self.base_wz)]
        msg.linear_acceleration = [0.0, 0.0, 0.0]
        _, _, qz, qw = yaw_quaternion(self.robot_yaw)
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.chassis_connected = True
        msg.gimbal_connected = True
        msg.imu_connected = False
        msg.emergency_stop = False
        msg.system_mode = 3
        self.platform_pub.publish(msg)

    def publish_odom(self, now):
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(self.robot_x)
        msg.pose.pose.position.y = float(self.robot_y)
        qx, qy, qz, qw = yaw_quaternion(self.robot_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = float(self.base_vx)
        msg.twist.twist.angular.z = float(self.base_wz)
        self.odom_pub.publish(msg)

    def publish_joint_states(self, now):
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = ["gimbal_yaw_joint", "gimbal_pitch_joint"]
        msg.position = [float(self.gimbal_yaw), float(self.gimbal_pitch)]
        msg.velocity = [float(self.gimbal_yaw_rate), float(self.gimbal_pitch_rate)]
        self.joint_pub.publish(msg)

    def publish_tf(self, now):
        base_tf = TransformStamped()
        base_tf.header.stamp = now.to_msg()
        base_tf.header.frame_id = "odom"
        base_tf.child_frame_id = "base_link"
        base_tf.transform.translation.x = float(self.robot_x)
        base_tf.transform.translation.y = float(self.robot_y)
        _, _, qz, qw = yaw_quaternion(self.robot_yaw)
        base_tf.transform.rotation.z = qz
        base_tf.transform.rotation.w = qw

        yaw_tf = TransformStamped()
        yaw_tf.header.stamp = now.to_msg()
        yaw_tf.header.frame_id = "base_link"
        yaw_tf.child_frame_id = "gimbal_yaw_link"
        yaw_tf.transform.translation.z = 0.2
        _, _, qz, qw = yaw_quaternion(self.gimbal_yaw)
        yaw_tf.transform.rotation.z = qz
        yaw_tf.transform.rotation.w = qw

        camera_tf = TransformStamped()
        camera_tf.header.stamp = now.to_msg()
        camera_tf.header.frame_id = "gimbal_yaw_link"
        camera_tf.child_frame_id = "camera_link"
        camera_tf.transform.translation.x = 0.15
        qx, qy, qz, qw = pitch_quaternion(self.gimbal_pitch)
        camera_tf.transform.rotation.x = qx
        camera_tf.transform.rotation.y = qy
        camera_tf.transform.rotation.z = qz
        camera_tf.transform.rotation.w = qw

        optical_tf = TransformStamped()
        optical_tf.header.stamp = now.to_msg()
        optical_tf.header.frame_id = "camera_link"
        optical_tf.child_frame_id = "camera_optical_link"
        qx, qy, qz, qw = rpy_quaternion(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        optical_tf.transform.rotation.x = qx
        optical_tf.transform.rotation.y = qy
        optical_tf.transform.rotation.z = qz
        optical_tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform([base_tf, yaw_tf, camera_tf, optical_tf])

    def publish_markers(self, now):
        self.current_marker_array = MarkerArray()
        self.publish_robot_marker(now)
        self.publish_target_marker(now)
        self.publish_path_marker(now)
        self.publish_camera_sight_marker(now)
        self.publish_desired_distance_marker(now)
        self.publish_virtual_shot_marker(now)
        self.marker_array_pub.publish(self.current_marker_array)
        self.current_marker_array = None

    def publish_marker(self, marker):
        self.marker_pub.publish(marker)
        if self.current_marker_array is not None:
            self.current_marker_array.markers.append(marker)

    def publish_robot_marker(self, now):
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "mvp_simple_sim"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.robot_x)
        marker.pose.position.y = float(self.robot_y)
        _, _, qz, qw = yaw_quaternion(self.robot_yaw)
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 0.35
        marker.scale.y = 0.14
        marker.scale.z = 0.14
        marker.color.r = 0.1
        marker.color.g = 0.45
        marker.color.b = 0.95
        marker.color.a = 0.9
        self.publish_marker(marker)

    def publish_target_marker(self, now):
        target_x, target_y = self.current_target_position(now)

        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "mvp_simple_sim"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(target_x)
        marker.pose.position.y = float(target_y)
        marker.pose.position.z = 0.2
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 0.95
        marker.color.g = 0.2
        marker.color.b = 0.1
        marker.color.a = 0.95
        self.publish_marker(marker)

    def publish_path_marker(self, now):
        point = Point()
        point.x = float(self.robot_x)
        point.y = float(self.robot_y)
        self.path_points.append(point)
        self.path_points = self.path_points[-300:]

        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "mvp_simple_sim"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 0.35
        marker.color.a = 0.85
        marker.points = self.path_points
        self.publish_marker(marker)

        target_x, target_y = self.current_target_position(now)
        target_point = Point()
        target_point.x = float(target_x)
        target_point.y = float(target_y)
        target_point.z = 0.2
        self.target_path_points.append(target_point)
        self.target_path_points = self.target_path_points[-300:]

        target_marker = Marker()
        target_marker.header.stamp = now.to_msg()
        target_marker.header.frame_id = "odom"
        target_marker.ns = "mvp_simple_sim"
        target_marker.id = 3
        target_marker.type = Marker.LINE_STRIP
        target_marker.action = Marker.ADD
        target_marker.scale.x = 0.025
        target_marker.color.r = 0.95
        target_marker.color.g = 0.45
        target_marker.color.b = 0.05
        target_marker.color.a = 0.8
        target_marker.points = self.target_path_points
        self.publish_marker(target_marker)

    def camera_position(self):
        camera_yaw = self.robot_yaw + self.gimbal_yaw
        return (
            self.robot_x + 0.15 * math.cos(camera_yaw),
            self.robot_y + 0.15 * math.sin(camera_yaw),
        )

    def publish_camera_sight_marker(self, now):
        target_x, target_y = self.current_target_position(now)
        camera_x, camera_y = self.camera_position()

        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "mvp_simple_sim"
        marker.id = 4
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.0
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.9

        camera_point = Point()
        camera_point.x = float(camera_x)
        camera_point.y = float(camera_y)
        camera_point.z = 0.2
        target_point = Point()
        target_point.x = float(target_x)
        target_point.y = float(target_y)
        target_point.z = 0.2
        marker.points = [camera_point, target_point]
        self.publish_marker(marker)

    def publish_desired_distance_marker(self, now):
        target_x, target_y = self.current_target_position(now)
        radius = max(float(self.get_parameter("desired_distance").value), 0.0)
        if radius <= 1e-6:
            return

        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "mvp_simple_sim"
        marker.id = 5
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.015
        marker.color.r = 0.55
        marker.color.g = 0.55
        marker.color.b = 0.55
        marker.color.a = 0.55

        points = []
        for idx in range(65):
            angle = 2.0 * math.pi * idx / 64.0
            point = Point()
            point.x = float(target_x + radius * math.cos(angle))
            point.y = float(target_y + radius * math.sin(angle))
            point.z = 0.02
            points.append(point)
        marker.points = points
        self.publish_marker(marker)

    def publish_virtual_shot_marker(self, now):
        if self.latest_shot_reference is None or not self.latest_shot_reference.valid:
            return

        pose = self.latest_shot_reference.virtual_base_pose

        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "mvp_simple_sim"
        marker.id = 6
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.pose.position.z = 0.08
        marker.scale.x = 0.32
        marker.scale.y = 0.22
        marker.scale.z = 0.16
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.45
        marker.color.a = 0.85
        self.publish_marker(marker)

        target_x, target_y = self.current_target_position(now)
        line = Marker()
        line.header.stamp = now.to_msg()
        line.header.frame_id = "odom"
        line.ns = "mvp_simple_sim"
        line.id = 7
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.025
        line.color.r = 0.1
        line.color.g = 0.9
        line.color.b = 0.45
        line.color.a = 0.8

        target_point = Point()
        target_point.x = float(target_x)
        target_point.y = float(target_y)
        target_point.z = 0.08
        virtual_point = Point()
        virtual_point.x = float(pose.position.x)
        virtual_point.y = float(pose.position.y)
        virtual_point.z = 0.08
        line.points = [target_point, virtual_point]
        self.publish_marker(line)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
