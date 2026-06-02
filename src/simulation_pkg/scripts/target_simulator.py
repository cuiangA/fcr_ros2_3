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
from visualization_msgs.msg import Marker
import math


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

        # ── 2. 创建发布者 ──────────────────────────────────────────
        # 队列深度 10：允许短暂的处理抖动而不丢失目标数据
        self.pose_pub = self.create_publisher(PoseStamped, "/target/pose", 10)
        self.marker_pub = self.create_publisher(Marker, "/target/marker", 10)

        # ── 3. 50 Hz 更新频率 = 20ms 间隔，确保轨迹平滑 ────────────
        self.timer = self.create_timer(0.02, self.update)
        self.t = 0.0  # 累计时间参数，驱动轨迹方程

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
        pose.header.frame_id = "odom"  # 以里程计坐标系为参考
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = h
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
