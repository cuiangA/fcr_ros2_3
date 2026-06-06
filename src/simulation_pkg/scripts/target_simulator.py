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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from vision_servo_msgs.msg import Target, TargetArray
import math

try:
    from gazebo_msgs.msg import EntityState
except ImportError:
    EntityState = None


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
      - height     (float,  默认 1.0):     目标在 Z 轴上的高度，单位米
    """

    def __init__(self):
        """构造函数 — 声明参数、创建发布者并启动定时器。"""
        super().__init__("target_simulator")

        # ── 1. 声明参数 ────────────────────────────────────────────
        self.declare_parameter("trajectory", "circle")  # circle, line, figure8
        self.declare_parameter("radius", 1.0)
        self.declare_parameter("speed", 0.5)
        self.declare_parameter("height", 1.0)
        self.declare_parameter("publish_target_array", False)
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("publish_gazebo_model_state", True)
        self.declare_parameter("gazebo_model_name", "target_box")

        # ── 2. 创建发布者 ──────────────────────────────────────────
        # 队列深度 10：允许短暂的处理抖动而不丢失目标数据
        self.pose_pub = self.create_publisher(PoseStamped, "/target/pose", 10)
        self.marker_pub = self.create_publisher(Marker, "/target/marker", 10)
        if self.get_parameter("publish_gazebo_model_state").value:
            if EntityState is None:
                self.get_logger().warning(
                    "gazebo_msgs 不可用，跳过 Gazebo 可见目标同步")
                self.model_state_pub = None
            else:
                self.model_state_pub = self.create_publisher(
                    EntityState, "/gazebo/set_entity_state", 10)
                self.gazebo_model_name = self.get_parameter("gazebo_model_name").value
        else:
            self.model_state_pub = None

        if self.get_parameter("publish_target_array").value:
            self.target3d_pub = self.create_publisher(
                TargetArray, "/perception/targets_3d", 10)
            self.camera_frame_ = self.get_parameter("camera_frame").value
        else:
            self.target3d_pub = None

        # ── 3. 订阅 odometry 和 joint_states，用于计算相机坐标系下的目标位置 ─
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.gimbal_yaw = 0.0   # 云台偏航角 (rad)
        self.gimbal_pitch = 0.0  # 云台俯仰角 (rad)

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_callback, 10)

        # ── 4. 50 Hz 更新频率 = 20ms 间隔，确保轨迹平滑 ────────────
        self.timer = self.create_timer(0.02, self.update)
        self.t = 0.0  # 累计时间参数，驱动轨迹方程

    def odom_callback(self, msg):
        """里程计回调 — 缓存机器人当前位姿（odom 坐标系下）。"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # 从四元数提取偏航角
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def joint_callback(self, msg):
        """关节状态回调 — 缓存云台当前角度。"""
        try:
            idx_yaw = msg.name.index("gimbal_yaw_joint")
            self.gimbal_yaw = msg.position[idx_yaw]
        except ValueError:
            pass
        try:
            idx_pitch = msg.name.index("gimbal_pitch_joint")
            self.gimbal_pitch = msg.position[idx_pitch]
        except ValueError:
            pass

    def transform_to_camera_frame(self, x, y, z):
        """
        将目标在 odom 坐标系下的位置转换到相机光心坐标系。

        完整变换链:
          1. odom → base_link (平移 + 绕 Z 轴反向旋转)
          2. base_link → gimbal_yaw_link (减去云台偏航关节偏移 + 反向偏航旋转)
          3. gimbal_yaw → gimbal_pitch_link (减去俯仰关节偏移 + 反向俯仰旋转)
          4. gimbal_pitch → camera_link (减去相机安装偏移)
          5. camera_link → camera_optical_link (轴重映射到 ROS 光学约定)

        URDF 关节偏移量 (来自 lekiwi_gimbal.xacro):
          base_link  → gimbal_yaw_joint:   (0, 0, 0.13)
          yaw_link   → gimbal_pitch_joint: (0, 0, 0.10)
          pitch_link → camera_joint:       (0.11, 0, 0)
          camera_link → camera_optical:     rpy=(-π/2, 0, -π/2)

        base_link:   x 向前, y 向左, z 向上
        相机光心系:   z 向前(光轴), x 向右, y 向下

        @param x, y, z: 目标在 odom 坐标系下的位置
        @return [cam_x, cam_y, cam_z]: 目标在相机光心系下的位置
        """
        yaw = self.gimbal_yaw
        pitch = self.gimbal_pitch

        # ── 步骤 1: odom → base_link（R_z(-robot_yaw) 反向旋转）────
        dx = x - self.robot_x
        dy = y - self.robot_y
        cy = math.cos(self.robot_yaw)
        sy = math.sin(self.robot_yaw)
        bx = dx * cy + dy * sy     # R_z(-yaw): x' =  x*cos + y*sin
        by = -dx * sy + dy * cy    #           y' = -x*sin + y*cos
        bz = z

        # ── 步骤 2: base_link → gimbal_yaw 关节原点 + 反向偏航旋转 ─
        pz = bz - 0.13             # 减去 yaw 关节 Z 偏移
        cyaw = math.cos(yaw)
        syaw = math.sin(yaw)
        py2 = -bx * syaw + by * cyaw   # R_z(-yaw)
        px2 =  bx * cyaw + by * syaw
        pz2 = pz

        # ── 步骤 3: yaw → pitch 关节原点 + 反向俯仰旋转 ────────────
        pz3 = pz2 - 0.10           # 减去 pitch 关节 Z 偏移
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        px4 = px2 * cp - pz3 * sp   # R_y(-pitch)
        py4 = py2
        pz4 = px2 * sp + pz3 * cp

        # ── 步骤 4: pitch → camera_link 原点（减去相机 X 偏移）────
        px5 = px4 - 0.11

        # ── 步骤 5: camera_link → camera_optical_link（轴重映射）───
        # ROS 光学约定: z 向前(光轴), x 向右, y 向下
        # camera_link 坐标系 → 光学: x_opt=-link_y, y_opt=-link_z, z_opt=link_x
        cam_x = -py4               # 右方向
        cam_y = -pz4               # 下方向
        cam_z =  px5               # 前方向（含相机 X 偏移）

        return [cam_x, cam_y, cam_z]

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

        # ── 根据轨迹类型计算目标在当前时刻的 X/Y 位置（odom 坐标系）──
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

        # ── 组装并发布 PoseStamped 消息（odom 坐标系，RViz 可视化用） ──
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"  # 以里程计坐标系为参考
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

        # ── 同步 Gazebo 中的可见目标模型（world 坐标系） ──────────
        if self.model_state_pub is not None:
            state = EntityState()
            state.name = self.gazebo_model_name
            state.pose = pose.pose
            state.reference_frame = "world"
            self.model_state_pub.publish(state)

        # ── 可选：发布 3D 目标位姿（直连伺服控制回路）──────────────
        if self.target3d_pub is not None:
            # 将目标位置从 odom 坐标系转换到相机光心坐标系
            cam_pos = self.transform_to_camera_frame(x, y, h)

            target = Target()
            target.class_name = "target"
            target.confidence = 0.9
            target.depth_confidence = 1.0
            # 位置：相机光心坐标系 (camera_optical_link) 下的 3D 坐标
            target.position = cam_pos
            # 填充虚拟的图像特征（供 IBVS 控制器使用）
            # bbox：640x480 图像中央的 80x80 像素框
            target.bbox = [280.0, 200.0, 360.0, 280.0]
            target.center = [320.0, 240.0]
            target_array = TargetArray()
            target_array.header = pose.header
            target_array.header.frame_id = self.camera_frame_
            target_array.targets = [target]
            target_array.tracking_id = 0
            self.target3d_pub.publish(target_array)


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
