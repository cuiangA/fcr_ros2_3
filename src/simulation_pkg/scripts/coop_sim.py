#!/usr/bin/env python3
"""
coop_sim — 仿真感知层 + 执行层，对接真实伺服控制链路。

架构:
  虚拟目标 → TargetArray(/perception/targets_3d) → servo_manager(IBVS/PBVS + ControlAllocator)
           → CameraInfo(/camera/camera_info)
                                                    ↓
                              /cmd_vel(TwistStamped) + /cmd_gimbal(GimbalCmd)
                                                    ↓
                                          coop_sim 运动学积分
                                                    ↓
                              /odom + /platform/state + /joint_states + /tf
                                                    ↓
                                          servo_manager 反馈闭环

只聚焦: 云台偏航 ⨯ 底盘旋转 ⨯ 底盘平移

用法:
  ros2 launch bringup_pkg coop_sim.launch.py
  ros2 launch bringup_pkg coop_sim.launch.py target_motion:=circle allocation_ratio:=0.3
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, JointState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from vision_servo_msgs.msg import GimbalCmd, PlatformState, Target, TargetArray


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


class CoopSim(Node):
    """感知+执行仿真节点。

    感知侧: 虚拟目标 → 投影到相机坐标系 → TargetArray + CameraInfo
    执行侧: 接收 cmd_vel + cmd_gimbal → 2D 运动学积分 → odom/platform_state/tf
    """

    def __init__(self):
        super().__init__("coop_sim")

        # ── 目标参数 ──────────────────────────────────────────────
        self.declare_parameter("target_motion", "figure8")
        self.declare_parameter("target_distance", 3.0)
        self.declare_parameter("target_angle_deg", 20.0)
        self.declare_parameter("target_radius", 1.0)
        self.declare_parameter("target_speed", 0.3)
        self.declare_parameter("target_size_m", 0.3)

        # ── 相机内参 (fx=400 → 水平FOV≈77°, 确保目标在视野内) ────
        self.declare_parameter("fx", 400.0)
        self.declare_parameter("fy", 400.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        # ── 初始条件 ──────────────────────────────────────────────
        self.declare_parameter("initial_robot_x", 0.0)
        self.declare_parameter("initial_robot_y", 0.0)
        self.declare_parameter("initial_robot_yaw_deg", 0.0)
        self.declare_parameter("initial_gimbal_yaw_deg", 0.0)

        # ── 物理约束（用于限位和可视化） ──────────────────────────
        self.declare_parameter("gimbal_yaw_limit_deg", 150.0)

        # ── 仿真参数 ──────────────────────────────────────────────
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("target_topic", "/perception/targets_3d")

        # ── 状态初始化 ────────────────────────────────────────────
        self.robot_x = float(self.get_parameter("initial_robot_x").value)
        self.robot_y = float(self.get_parameter("initial_robot_y").value)
        self.robot_yaw = math.radians(
            float(self.get_parameter("initial_robot_yaw_deg").value))
        self.gimbal_yaw = math.radians(
            float(self.get_parameter("initial_gimbal_yaw_deg").value))
        self.gimbal_pitch = 0.0
        self.base_vx = 0.0
        self.base_wz = 0.0
        self.gimbal_yaw_rate = 0.0
        self.gimbal_pitch_rate = 0.0
        self.path = []
        self.target_path = []

        # ── QoS ───────────────────────────────────────────────────
        # 控制节点 (servo_manager / mvp) 都用 RELIABLE 订阅，
        # 发布端必须匹配否则消息收不到
        cmd_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                             reliability=ReliabilityPolicy.RELIABLE)
        # PlatformState 加 TRANSIENT_LOCAL，匹配 mvp/servo_manager 的订阅
        platform_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                  reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        state_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                               reliability=ReliabilityPolicy.RELIABLE)
        cam_info_qos = QoSProfile(depth=1,
                                  reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ── 订阅: 来自 servo_manager 的控制指令 ───────────────────
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, "/cmd_vel", self.cmd_vel_cb, cmd_qos)
        self.cmd_gimbal_sub = self.create_subscription(
            GimbalCmd, "/cmd_gimbal", self.cmd_gimbal_cb, cmd_qos)

        # ── 发布: 感知数据 ────────────────────────────────────────
        target_topic = str(self.get_parameter("target_topic").value)
        self.target_pub = self.create_publisher(
            TargetArray, target_topic, cmd_qos)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, "/camera/camera_info", cam_info_qos)

        # ── 发布: 执行反馈 ────────────────────────────────────────
        self.platform_pub = self.create_publisher(
            PlatformState, "/platform/state", platform_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", state_qos)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", state_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/visualization_marker_array", state_qos)
        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/target/pose", state_qos)
        self.tf_bc = TransformBroadcaster(self)

        # ── 诊断话题 ──────────────────────────────────────────────
        self.diag_yaw_pub = self.create_publisher(
            TwistStamped, "/diag/yaw_error", state_qos)
        self.diag_dist_pub = self.create_publisher(
            TwistStamped, "/diag/distance_error", state_qos)
        self.diag_cmd_pub = self.create_publisher(
            TwistStamped, "/diag/cmd", state_qos)

        # ── 发布一次 CameraInfo (TRANSIENT_LOCAL) ──────────────────
        self.publish_camera_info()

        # ── 定时器 ────────────────────────────────────────────────
        rate = max(float(self.get_parameter("update_rate_hz").value), 1.0)
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.update)
        self.frame_count = 0
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f"CoopSim 启动 (真实伺服链路) | rate={rate:.0f}Hz "
            f"| 目标={self.get_parameter('target_motion').value}"
            f"| 感知→servo_manager→执行 闭环")

    # ═══════════════════════════════════════════════════════════════
    # 控制指令回调
    # ═══════════════════════════════════════════════════════════════

    def cmd_vel_cb(self, msg):
        self.base_vx = msg.twist.linear.x
        self.base_wz = msg.twist.angular.z

    def cmd_gimbal_cb(self, msg):
        self.gimbal_yaw_rate = 0.0 if msg.hold_yaw else msg.yaw_rate
        self.gimbal_pitch_rate = 0.0 if msg.hold_pitch else msg.pitch_rate

    # ═══════════════════════════════════════════════════════════════
    # 感知: 虚拟目标 → TargetArray
    # ═══════════════════════════════════════════════════════════════

    def target_position(self, now):
        """目标在 odom 坐标系中的位置。"""
        motion = str(self.get_parameter("target_motion").value).lower()
        dist = float(self.get_parameter("target_distance").value)
        angle = math.radians(float(self.get_parameter("target_angle_deg").value))
        r = max(float(self.get_parameter("target_radius").value), 0.01)
        speed = float(self.get_parameter("target_speed").value)
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        if motion == "static":
            cx, cy = 0.0, 0.0
        elif motion == "line":
            cx = r * math.sin(speed * elapsed)
            cy = 0.0
        elif motion == "figure8":
            cx = r * math.sin(speed * elapsed)
            cy = 0.5 * r * math.sin(2.0 * speed * elapsed)
        else:  # circle
            cx = r * math.cos(speed * elapsed)
            cy = r * math.sin(speed * elapsed)

        return cx + dist * math.cos(angle), cy + dist * math.sin(angle)

    def target_in_camera_frame(self, tx, ty):
        """将 odom 系下的目标位置转换到 camera_optical_link 系。

        camera_optical_link: z 向前(光轴), x 向右, y 向下
        """
        camera_yaw = self.robot_yaw + self.gimbal_yaw
        # 相机在 odom 中的位置 (机器人中心 + 相机前向偏移)
        cam_x = self.robot_x + 0.15 * math.cos(camera_yaw)
        cam_y = self.robot_y + 0.15 * math.sin(camera_yaw)

        dx = tx - cam_x
        dy = ty - cam_y
        cyaw = math.cos(camera_yaw)
        syaw = math.sin(camera_yaw)

        forward = cyaw * dx + syaw * dy          # base_link x → 光轴 z
        left = -syaw * dx + cyaw * dy            # base_link y → 光轴 -x
        optical_x = -left                         # 相机右侧
        optical_z = forward                       # 相机前方

        return optical_x, optical_z

    def project_to_image(self, optical_x, optical_z):
        """将相机系 3D 点投影到图像坐标。"""
        if optical_z < 0.05:
            return None

        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)
        w = int(self.get_parameter("image_width").value)
        h = int(self.get_parameter("image_height").value)

        u = fx * optical_x / optical_z + cx
        v = fy * 0.0 / optical_z + cy             # 目标在地面，高度 ≈ 0

        target_size = float(self.get_parameter("target_size_m").value)
        bbox_w = fx * target_size / optical_z
        bbox_h = fy * target_size / optical_z

        x_min = u - 0.5 * bbox_w
        y_min = v - 0.5 * bbox_h
        x_max = u + 0.5 * bbox_w
        y_max = v + 0.5 * bbox_h

        if x_max <= 0 or y_max <= 0 or x_min >= w or y_min >= h:
            return None

        x_min = clamp(x_min, 0, w)
        y_min = clamp(y_min, 0, h)
        x_max = clamp(x_max, 0, w)
        y_max = clamp(y_max, 0, h)

        if x_max - x_min < 2 or y_max - y_min < 2:
            return None

        return {
            "center": [float(u), float(v)],
            "bbox": [float(x_min), float(y_min), float(x_max), float(y_max)],
            "width": float(x_max - x_min),
            "height": float(y_max - y_min),
        }

    def publish_camera_info(self):
        msg = CameraInfo()
        msg.header.frame_id = "camera_optical_link"
        msg.width = int(self.get_parameter("image_width").value)
        msg.height = int(self.get_parameter("image_height").value)
        msg.k = [
            float(self.get_parameter("fx").value), 0.0,
            float(self.get_parameter("cx").value),
            0.0, float(self.get_parameter("fy").value),
            float(self.get_parameter("cy").value),
            0.0, 0.0, 1.0,
        ]
        self.camera_info_pub.publish(msg)
        self.get_logger().info("CameraInfo 已发布 (TRANSIENT_LOCAL)")

    # ═══════════════════════════════════════════════════════════════
    # 主循环
    # ═══════════════════════════════════════════════════════════════

    def update(self):
        now = self.get_clock().now()
        self.frame_count += 1

        # 1Hz 诊断：确认目标运动模式
        if self.frame_count % 50 == 0:
            motion = str(self.get_parameter("target_motion").value)
            self.get_logger().info(
                f"[diag] frame={self.frame_count} motion={motion} "
                f"robot=({self.robot_x:.2f},{self.robot_y:.2f},{math.degrees(self.robot_yaw):.1f}°) "
                f"gimbal={math.degrees(self.gimbal_yaw):.1f}° "
                f"cmd=(vx={self.base_vx:.3f}, wz={math.degrees(self.base_wz):.1f}°/s)",
                throttle_duration_sec=0.9)

        # ── 1. 运动学积分（执行来自 servo_manager 的指令）─────────
        self.robot_x += self.base_vx * math.cos(self.robot_yaw) * self.dt
        self.robot_y += self.base_vx * math.sin(self.robot_yaw) * self.dt
        self.robot_yaw = wrap_pi(self.robot_yaw + self.base_wz * self.dt)

        lim = math.radians(float(self.get_parameter("gimbal_yaw_limit_deg").value))
        self.gimbal_yaw = clamp(self.gimbal_yaw + self.gimbal_yaw_rate * self.dt, -lim, lim)
        self.gimbal_pitch = clamp(self.gimbal_pitch + self.gimbal_pitch_rate * self.dt,
                                   -math.pi / 2, math.pi / 2)

        # ── 2. 感知: 虚拟目标 → TargetArray ────────────────────────
        tx, ty = self.target_position(now)
        optical_x, optical_z = self.target_in_camera_frame(tx, ty)
        projection = self.project_to_image(optical_x, optical_z)

        target_array = TargetArray()
        target_array.header.stamp = now.to_msg()
        target_array.header.frame_id = "camera_optical_link"

        if projection is not None:
            t = Target()
            t.header = target_array.header
            t.id = 0
            t.class_name = "person"
            t.tracking_state = Target.TRACKING_STATE_CONFIRMED
            t.visible = True
            t.confidence = 0.95
            t.depth_confidence = 1.0
            t.position = [float(optical_x), 0.0, float(optical_z)]
            t.velocity = [0.0, 0.0, 0.0]
            t.center = projection["center"]
            t.bbox = projection["bbox"]
            t.width = projection["width"]
            t.height = projection["height"]
            target_array.targets = [t]
            target_array.tracking_id = 0
        else:
            target_array.tracking_id = -1

        self.target_pub.publish(target_array)

        # ── 3. 执行反馈: odom + platform_state + joint_states + tf ─
        self.publish_odom(now)
        self.publish_platform_state(now)
        self.publish_joint_states(now)
        if bool(self.get_parameter("publish_tf").value):
            self.publish_tf(now)

        # ── 4. 可视化 ──────────────────────────────────────────────
        if bool(self.get_parameter("publish_markers").value):
            self.publish_markers(now, tx, ty)

        self.publish_target_pose(now, tx, ty)
        self.publish_diagnostics(now, optical_x, optical_z, tx, ty)

    # ═══════════════════════════════════════════════════════════════
    # 发布
    # ═══════════════════════════════════════════════════════════════

    def publish_platform_state(self, now):
        msg = PlatformState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.chassis_pose = [float(self.robot_x), float(self.robot_y),
                            float(self.robot_yaw)]
        msg.chassis_velocity = [float(self.base_vx), 0.0, float(self.base_wz)]
        msg.battery_voltage = 24.0
        msg.gimbal_yaw = float(self.gimbal_yaw)
        msg.gimbal_pitch = float(self.gimbal_pitch)
        msg.gimbal_yaw_rate = float(self.gimbal_yaw_rate)
        msg.gimbal_pitch_rate = float(self.gimbal_pitch_rate)
        msg.angular_velocity = [0.0, 0.0, float(self.base_wz)]
        msg.linear_acceleration = [0.0, 0.0, 0.0]
        _, _, qz, qw = yaw_to_quat(self.robot_yaw)
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
        msg.pose.pose.position.x = self.robot_x
        msg.pose.pose.position.y = self.robot_y
        _, _, qz, qw = yaw_to_quat(self.robot_yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.base_vx
        msg.twist.twist.angular.z = self.base_wz
        self.odom_pub.publish(msg)

    def publish_joint_states(self, now):
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = ["gimbal_yaw_joint", "gimbal_pitch_joint"]
        msg.position = [float(self.gimbal_yaw), float(self.gimbal_pitch)]
        msg.velocity = [float(self.gimbal_yaw_rate), float(self.gimbal_pitch_rate)]
        self.joint_pub.publish(msg)

    def publish_tf(self, now):
        bt = TransformStamped()
        bt.header.stamp = now.to_msg()
        bt.header.frame_id = "odom"
        bt.child_frame_id = "base_link"
        bt.transform.translation.x = self.robot_x
        bt.transform.translation.y = self.robot_y
        _, _, qz, qw = yaw_to_quat(self.robot_yaw)
        bt.transform.rotation.z = qz
        bt.transform.rotation.w = qw

        gt = TransformStamped()
        gt.header.stamp = now.to_msg()
        gt.header.frame_id = "base_link"
        gt.child_frame_id = "gimbal_yaw_link"
        gt.transform.translation.z = 0.2
        _, _, qz, qw = yaw_to_quat(self.gimbal_yaw)
        gt.transform.rotation.z = qz
        gt.transform.rotation.w = qw

        self.tf_bc.sendTransform([bt, gt])

    def publish_target_pose(self, now, tx, ty):
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = tx
        msg.pose.position.y = ty
        msg.pose.position.z = 0.35
        msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(msg)

    def publish_diagnostics(self, now, optical_x, optical_z, tx, ty):
        """Foxglove Plot 用诊断信号。"""
        # 偏航误差
        camera_yaw = self.robot_yaw + self.gimbal_yaw
        dx = tx - self.robot_x
        dy = ty - self.robot_y
        angle_to_target = math.atan2(dy, dx)
        yaw_error = wrap_pi(angle_to_target - camera_yaw)

        d = TwistStamped()
        d.header.stamp = now.to_msg()
        d.twist.angular.z = yaw_error
        self.diag_yaw_pub.publish(d)

        # 距离
        distance = math.hypot(dx, dy)
        d2 = TwistStamped()
        d2.header.stamp = now.to_msg()
        d2.twist.linear.x = float(optical_z)
        d2.twist.linear.y = float(distance)
        self.diag_dist_pub.publish(d2)

        # 控制指令
        d3 = TwistStamped()
        d3.header.stamp = now.to_msg()
        d3.twist.linear.x = self.base_vx
        d3.twist.linear.y = self.gimbal_yaw_rate
        d3.twist.angular.z = self.base_wz
        self.diag_cmd_pub.publish(d3)

    # ═══════════════════════════════════════════════════════════════
    # Foxglove 3D Markers
    # ═══════════════════════════════════════════════════════════════

    def publish_markers(self, now, tx, ty):
        arr = MarkerArray()

        # 0. 机器人 (蓝色箭头)
        m = Marker()
        m.header.stamp = now.to_msg(); m.header.frame_id = "odom"
        m.ns = "coop"; m.id = 0; m.type = Marker.ARROW; m.action = Marker.ADD
        m.pose.position.x = self.robot_x; m.pose.position.y = self.robot_y
        _, _, qz, qw = yaw_to_quat(self.robot_yaw)
        m.pose.orientation.z = qz; m.pose.orientation.w = qw
        m.scale.x = 0.4; m.scale.y = 0.15; m.scale.z = 0.15
        m.color.r = 0.1; m.color.g = 0.4; m.color.b = 0.9; m.color.a = 0.9
        arr.markers.append(m)

        # 1. 云台偏航 (青色小箭头, base_link 系)
        m = Marker()
        m.header.stamp = now.to_msg(); m.header.frame_id = "base_link"
        m.ns = "coop"; m.id = 1; m.type = Marker.ARROW; m.action = Marker.ADD
        m.pose.position.z = 0.2
        _, _, qz, qw = yaw_to_quat(self.gimbal_yaw)
        m.pose.orientation.z = qz; m.pose.orientation.w = qw
        m.scale.x = 0.2; m.scale.y = 0.07; m.scale.z = 0.07
        m.color.r = 0.0; m.color.g = 0.8; m.color.b = 1.0; m.color.a = 0.85
        arr.markers.append(m)

        # 2. 相机视线 (绿线)
        camera_yaw = self.robot_yaw + self.gimbal_yaw
        cam_x = self.robot_x + 0.15 * math.cos(camera_yaw)
        cam_y = self.robot_y + 0.15 * math.sin(camera_yaw)
        m = Marker()
        m.header.stamp = now.to_msg(); m.header.frame_id = "odom"
        m.ns = "coop"; m.id = 2; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.r = 0.0; m.color.g = 0.9; m.color.b = 0.3; m.color.a = 0.9
        cp = Point(); cp.x = cam_x; cp.y = cam_y; cp.z = 0.3
        tp = Point(); tp.x = tx; tp.y = ty; tp.z = 0.35
        m.points = [cp, tp]
        arr.markers.append(m)

        # 3. 目标 (红球)
        m = Marker()
        m.header.stamp = now.to_msg(); m.header.frame_id = "odom"
        m.ns = "coop"; m.id = 3; m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = tx; m.pose.position.y = ty; m.pose.position.z = 0.35
        m.scale.x = m.scale.y = m.scale.z = 0.22
        m.color.r = 0.95; m.color.g = 0.15; m.color.b = 0.1; m.color.a = 0.95
        arr.markers.append(m)

        # 4. 期望距离圈
        desired_dist = 2.0
        if desired_dist > 0.01:
            m = Marker()
            m.header.stamp = now.to_msg(); m.header.frame_id = "odom"
            m.ns = "coop"; m.id = 4; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.015
            m.color.r = 0.6; m.color.g = 0.6; m.color.b = 0.6; m.color.a = 0.5
            pts = []
            for i in range(65):
                a = 2.0 * math.pi * i / 64.0
                p = Point(); p.x = tx + desired_dist * math.cos(a)
                p.y = ty + desired_dist * math.sin(a); p.z = 0.03
                pts.append(p)
            m.points = pts
            arr.markers.append(m)

        # 5. 机器人轨迹 (青)
        p = Point(); p.x = self.robot_x; p.y = self.robot_y
        self.path.append(p); self.path = self.path[-500:]
        m = Marker()
        m.header.stamp = now.to_msg(); m.header.frame_id = "odom"
        m.ns = "coop"; m.id = 5; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.03
        m.color.r = 0.0; m.color.g = 0.7; m.color.b = 0.8; m.color.a = 0.7
        m.points = self.path
        arr.markers.append(m)

        # 6. 目标轨迹 (红)
        tp = Point(); tp.x = tx; tp.y = ty; tp.z = 0.35
        self.target_path.append(tp); self.target_path = self.target_path[-500:]
        m = Marker()
        m.header.stamp = now.to_msg(); m.header.frame_id = "odom"
        m.ns = "coop"; m.id = 6; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.r = 0.95; m.color.g = 0.4; m.color.b = 0.1; m.color.a = 0.65
        m.points = self.target_path
        arr.markers.append(m)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CoopSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
