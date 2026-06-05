#!/usr/bin/env python3
# simulation_pkg/scripts/target_simulator.py
"""
@file target_simulator.py
@brief 目标轨迹仿真器 — 生成运动目标位姿用于视觉伺服算法测试。

在仿真环境中模拟一个沿预设轨迹运动的目标物体，同时发布：
  - 目标在 odom 坐标系下的 3D 位姿
  - RViz 可视化用的红色球体 Marker

支持的轨迹类型通过 trajectory 参数切换，无需重新编译或重启节点。

@note 发布话题：
  - /target/pose   (geometry_msgs/PoseStamped)  目标当前位姿
  - /target/marker (visualization_msgs/Marker)   RViz 可视化标记
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from vision_servo_msgs.msg import Target, TargetArray
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

try:
    from gazebo_msgs.msg import ModelState
except ImportError:
    ModelState = None


class TargetSimulator(Node):
    """
    @class TargetSimulator
    @brief 以固定频率更新并发布运动目标轨迹的 ROS2 仿真节点。

    支持三种预定义轨迹（可通过 trajectory 参数切换）：
      - "circle"   圆形轨迹，半径和角速度可调
      - "figure8"  八字形（Lissajous 曲线），用于测试非线性跟踪
      - "line"     匀速直线运动，验证基础伺服跟踪能力

    参数（均可在运行时声明）：
      - trajectory (string, 默认 "circle"): 轨迹类型 (circle / figure8 / line)
      - radius     (float,  默认 1.0):     圆形/八字形的特征半径，单位米
      - speed      (float,  默认 0.5):     角速度或线速度，单位取决于轨迹类型
      - height     (float,  默认 0.35):    目标在 Z 轴上的高度，单位米
    """

    def __init__(self):
        """构造函数 — 声明参数、创建发布者并启动定时器。"""
        super().__init__("target_simulator")

        # ── 1. 声明参数 ────────────────────────────────────────────
        self.declare_parameter("trajectory", "circle")  # circle, line, figure8
        self.declare_parameter("radius", 1.0)
        self.declare_parameter("speed", 0.5)
        self.declare_parameter("height", 0.35)
        self.declare_parameter("publish_target_array", False)
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("camera_frame", "camera_optical_link")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("fx", 600.0)
        self.declare_parameter("fy", 600.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("target_real_size", 0.30)
        self.declare_parameter("min_depth", 0.15)
        self.declare_parameter("publish_gazebo_model_state", True)
        self.declare_parameter("gazebo_model_name", "target_box")

        self.world_frame_ = self.get_parameter("world_frame").value
        self.camera_frame_ = self.get_parameter("camera_frame").value
        self.image_width_ = int(self.get_parameter("image_width").value)
        self.image_height_ = int(self.get_parameter("image_height").value)
        self.fx_ = float(self.get_parameter("fx").value)
        self.fy_ = float(self.get_parameter("fy").value)
        self.cx_ = float(self.get_parameter("cx").value)
        self.cy_ = float(self.get_parameter("cy").value)
        self.target_real_size_ = float(self.get_parameter("target_real_size").value)
        self.min_depth_ = float(self.get_parameter("min_depth").value)
        self._last_warnings = {}

        # ── 2. 创建发布者 ──────────────────────────────────────────
        # 队列深度 10：允许短暂的处理抖动而不丢失目标数据
        self.pose_pub = self.create_publisher(PoseStamped, "/target/pose", 10)
        self.marker_pub = self.create_publisher(Marker, "/target/marker", 10)
        if self.get_parameter("publish_gazebo_model_state").value:
            if ModelState is None:
                self.get_logger().warning(
                    "gazebo_msgs 不可用，跳过 Gazebo 可见目标同步")
                self.model_state_pub = None
            else:
                self.model_state_pub = self.create_publisher(
                    ModelState, "/gazebo/set_model_state", 10)
                self.gazebo_model_name = self.get_parameter("gazebo_model_name").value
        else:
            self.model_state_pub = None

        if self.get_parameter("publish_target_array").value:
            self.target3d_pub = self.create_publisher(
                TargetArray, "/perception/targets_3d", 10)
            camera_info_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.camera_info_sub = self.create_subscription(
                CameraInfo, "/camera/camera_info",
                self.camera_info_callback, camera_info_qos)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.target3d_pub = None
            self.camera_info_sub = None
            self.tf_buffer = None
            self.tf_listener = None

        # ── 3. 50 Hz 更新频率 = 20ms 间隔，确保轨迹平滑 ────────────
        self.timer = self.create_timer(0.02, self.update)
        self.t = 0.0  # 累计时间参数，驱动轨迹方程

    def camera_info_callback(self, msg):
        """缓存相机内参，保证虚拟检测与 servo_manager 使用同一套投影模型。"""
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            self._warn_throttled("camera_info",
                                 "收到无效 camera_info，继续使用目标模拟器默认内参")
            return

        self.fx_ = float(msg.k[0])
        self.fy_ = float(msg.k[4])
        self.cx_ = float(msg.k[2])
        self.cy_ = float(msg.k[5])
        self.image_width_ = int(msg.width)
        self.image_height_ = int(msg.height)

    def update(self):
        """
        @brief 定时器回调 — 根据 trajectory 参数计算当前位置并发布位姿和标记。

        每次调用递增 self.t，使轨迹沿时间轴推进。50 Hz 的更新频率保证
        RViz 中显示的球体运动平滑无抖动。
        """
        traj = self.get_parameter("trajectory").value
        r = self.get_parameter("radius").value
        speed = self.get_parameter("speed").value
        h = self.get_parameter("height").value

        # ── 根据轨迹类型计算目标在当前时刻的 X/Y 位置 ──────────────
        if traj == "circle":
            # 标准参数方程：x = r*cos(wt), y = r*sin(wt)
            x = r * math.cos(speed * self.t)
            y = r * math.sin(speed * self.t)
        elif traj == "figure8":
            # Lissajous 曲线：x = R*sin(wt), y = R/2*sin(2wt)
            # 八字形交叉可测试控制器的方向切换能力
            x = r * math.sin(speed * self.t)
            y = r * math.sin(2 * speed * self.t) / 2
        else:  # line
            # 沿 X 轴匀速运动，从 x=-1.0 开始向右移动
            x = self.t * speed - 1.0
            y = 0.0

        self.t += 0.02  # 与定时器周期保持一致

        # ── 组装并发布 PoseStamped 消息 ────────────────────────────
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.world_frame_  # 以世界/里程计坐标系为参考
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = h
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        # ── 组装并发布 Marker 消息，用于 RViz 可视化 ───────────────
        marker = Marker()
        marker.header = pose.header
        marker.type = Marker.SPHERE         # 球体，清晰可见
        marker.pose = pose.pose             # 直接使用 pose 的位置
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2  # 直径 20cm
        marker.color.a = 1.0                # 完全不透明
        marker.color.r = 1.0                # 红色，突出显示
        self.marker_pub.publish(marker)

        # ── 4. 同步 Gazebo 中的可见目标模型 ───────────────────────
        if self.model_state_pub is not None:
            state = ModelState()
            state.model_name = self.gazebo_model_name
            state.pose = pose.pose
            state.reference_frame = "world"
            self.model_state_pub.publish(state)

        # ── 5. 可选：发布 3D 目标位姿（直连伺服控制回路）──────────
        if self.target3d_pub is not None:
            target_array = self.build_virtual_camera_target(pose)
            if target_array is not None:
                self.target3d_pub.publish(target_array)

    def build_virtual_camera_target(self, pose):
        """将 odom 中的目标点通过 TF 投影成 camera optical frame 下的检测结果。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame_, pose.header.frame_id, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self._warn_throttled(
                "tf",
                f"无法将目标从 {pose.header.frame_id} 变换到 {self.camera_frame_}: {exc}")
            return None

        x_cam, y_cam, z_cam = self._transform_point(
            transform,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z)

        return self._project_target(pose.header.stamp, x_cam, y_cam, z_cam)

    def _project_target(self, stamp, x_cam, y_cam, z_cam):
        if z_cam <= self.min_depth_:
            self._warn_throttled(
                "behind_camera",
                f"目标不在相机前方或距离过近，z={z_cam:.2f}m，暂停发布伺服目标")
            return None

        u = self.fx_ * x_cam / z_cam + self.cx_
        v = self.fy_ * y_cam / z_cam + self.cy_
        if not all(math.isfinite(value) for value in (u, v, z_cam)):
            self._warn_throttled("projection", "目标投影结果无效，暂停发布伺服目标")
            return None

        if u < 0.0 or u >= self.image_width_ or v < 0.0 or v >= self.image_height_:
            self._warn_throttled(
                "out_of_view",
                f"目标中心超出相机视野 u={u:.1f}, v={v:.1f}，暂停发布伺服目标")
            return None

        bbox_size = max(2.0, self.fx_ * self.target_real_size_ / z_cam)
        half = 0.5 * bbox_size
        x_min = max(0.0, u - half)
        y_min = max(0.0, v - half)
        x_max = min(float(self.image_width_ - 1), u + half)
        y_max = min(float(self.image_height_ - 1), v + half)
        bbox_w = x_max - x_min
        bbox_h = y_max - y_min
        if bbox_w <= 1.0 or bbox_h <= 1.0:
            return None

        target = Target()
        target.header.stamp = stamp
        target.header.frame_id = self.camera_frame_
        target.id = 0
        target.class_name = "target"
        target.confidence = 0.9
        target.depth_confidence = 1.0
        target.position = [float(x_cam), float(y_cam), float(z_cam)]
        target.bbox = [float(x_min), float(y_min), float(x_max), float(y_max)]
        target.center = [float(u), float(v)]
        target.width = float(bbox_w)
        target.height = float(bbox_h)

        target_array = TargetArray()
        target_array.header.stamp = stamp
        target_array.header.frame_id = self.camera_frame_
        target_array.targets = [target]
        target_array.tracking_id = 0
        return target_array

    def _transform_point(self, transform, x, y, z):
        rx, ry, rz = self._rotate_vector(transform.transform.rotation, x, y, z)
        t = transform.transform.translation
        return rx + t.x, ry + t.y, rz + t.z

    @staticmethod
    def _rotate_vector(q, x, y, z):
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            return x, y, z
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        uvx = qy * z - qz * y
        uvy = qz * x - qx * z
        uvz = qx * y - qy * x
        uuvx = qy * uvz - qz * uvy
        uuvy = qz * uvx - qx * uvz
        uuvz = qx * uvy - qy * uvx

        return (
            x + 2.0 * (qw * uvx + uuvx),
            y + 2.0 * (qw * uvy + uuvy),
            z + 2.0 * (qw * uvz + uuvz),
        )

    def _warn_throttled(self, key, message, period_sec=2.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        last = self._last_warnings.get(key)
        if last is None or now - last >= period_sec:
            self.get_logger().warning(message)
            self._last_warnings[key] = now


def main(args=None):
    """
    @brief 入口函数 — 初始化 ROS2、启动目标轨迹仿真节点、完成后关闭。

    使用单线程默认执行器（rclpy.spin）。如需多线程执行，
    可替换为 MultiThreadedExecutor 或自定义 executor。
    """
    rclpy.init(args=args)
    rclpy.spin(TargetSimulator())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
