#!/usr/bin/env python3
# simulation_pkg/scripts/camera_simulator.py
"""
@file camera_simulator.py
@brief 仿真相机发布者 — 生成合成图像用于离线测试。

在仿真环境中替代真实相机，以固定帧率发布合成测试图案。
主要用途：
  - 视觉伺服算法的离线调试
  - 无需真实摄像头的 CI/CD 测试
  - 图像处理流水线的快速原型验证

发布的合成图像包含递增帧号文字，便于观察帧率和丢帧情况。

@note 发布话题：
  - /camera/image_raw   (sensor_msgs/Image)      合成测试图像
  - /camera/camera_info (sensor_msgs/CameraInfo) 静态相机内参（仅发布一次）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraSimulator(Node):
    """
    @class CameraSimulator
    @brief 以固定帧率发布合成图像的 ROS2 仿真节点。

    参数（均可在运行时声明）：
      - width    (int,   默认 640):  图像宽度，单位像素
      - height   (int,   默认 480):  图像高度，单位像素
      - fps      (float, 默认 30.0): 发布帧率，单位 Hz
      - frame_id (string, 默认 "camera_optical_link"): 消息头中的 TF 坐标系名称
    """

    def __init__(self):
        """构造函数 — 声明参数、创建发布者、初始化相机内参并启动定时器。"""
        super().__init__("camera_simulator")

        # ── 1. 声明参数并设置合理的仿真默认值 ──────────────────────
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("frame_id", "camera_optical_link")

        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        fps = self.get_parameter("fps").value

        # ── 2. 初始化 OpenCV 桥接工具 ──────────────────────────────
        # cv_bridge 负责在 OpenCV cv::Mat 和 ROS sensor_msgs/Image 之间转换
        self.bridge = CvBridge()

        # ── 3. 创建发布者，队列深度设为 1 ──────────────────────────
        # 仿真图像对流式旧数据无意义：只需最新帧，避免管线积压
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 1)
        # TRANSIENT_LOCAL + RELIABLE = 迟加入的订阅者也能收到最后一次发布的内参
        self.camera_info_pub = self.create_publisher(
            CameraInfo, "/camera/camera_info",
            rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                 durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))


        # ── 4. 创建定时器，驱动周期性图像发布 ──────────────────────
        self.timer = self.create_timer(1.0 / fps, self.publish)
        self.frame_count = 0

        # ── 5. 仅发布一次静态相机内参 ──────────────────────────────
        # CameraInfo 在物理相机中通常是固定不变的，仿真中也只需一次
        info = CameraInfo()
        info.header.frame_id = self.get_parameter("frame_id").value
        info.width = self.width
        info.height = self.height
        # 使用针孔相机模型的近似内参矩阵 K
        info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        self.camera_info_pub.publish(info)

    def publish(self):
        """
        @brief 定时器回调 — 生成一帧合成图像并发布。

        生成黑色背景上叠加递增帧号的测试图案。通过 frame_count
        递增使帧号连续变化，便于在 RViz 或录像回放中识别丢帧/跳帧。
        """
        # 生成纯黑背景图像（BGR 三个通道均为零）
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # 在图像中央叠加当前帧号文字，用于肉眼观察和帧率验证
        cv2.putText(img, f"Frame {self.frame_count}", (50, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        self.frame_count += 1

        # 将 OpenCV 图像转换为 ROS Image 消息并填充时间戳与坐标系
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value
        self.image_pub.publish(msg)


def main(args=None):
    """
    @brief 入口函数 — 初始化 ROS2、启动仿真相机节点、完成后关闭。

    使用单线程默认执行器（rclpy.spin）。如需多线程执行，
    可替换为 MultiThreadedExecutor 或自定义 executor。
    """
    rclpy.init(args=args)
    rclpy.spin(CameraSimulator())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
