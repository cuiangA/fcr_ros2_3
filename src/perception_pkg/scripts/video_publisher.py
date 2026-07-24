#!/usr/bin/env python3
"""从视频文件逐帧发布到 ROS2 图像话题（默认 /sony/image_raw）。

用于在无相机环境下使用录制视频验证感知管线。
"""

import argparse
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class VideoPublisher(Node):
    def __init__(self, video_path, topic="sony/image_raw", loop=True):
        super().__init__("video_publisher")
        self._loop = loop

        if not os.path.isfile(video_path):
            self.get_logger().fatal(f"视频文件不存在: {video_path}")
            raise FileNotFoundError(f"视频文件不存在: {video_path}")

        self._cap = cv2.VideoCapture(video_path)
        if not self._cap.isOpened():
            self.get_logger().fatal(f"无法打开视频文件: {video_path}")
            raise RuntimeError(f"无法打开视频文件: {video_path}")

        self._width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._fps = self._cap.get(cv2.CAP_PROP_FPS)
        if self._fps <= 0:
            self._fps = 30.0
        self._frame_count = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))

        self.get_logger().info(
            f"视频: {os.path.basename(video_path)} "
            f"分辨率={self._width}x{self._height} "
            f"FPS={self._fps:.1f} "
            f"总帧数={self._frame_count}"
        )

        period = 1.0 / self._fps
        self._pub = self.create_publisher(Image, topic, 10)
        self._bridge = CvBridge()
        self._timer = self.create_timer(period, self._publish_frame)
        self._frame_index = 0
        self.get_logger().info(f"开始向 {topic} 发布视频帧")

    def _publish_frame(self):
        ret, frame = self._cap.read()
        if not ret:
            if not self._loop:
                self.get_logger().info("视频播放完毕，退出")
                rclpy.shutdown()
                return
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self._frame_index = 0
            self.get_logger().info("循环播放")
            ret, frame = self._cap.read()
            if not ret:
                self.get_logger().error("重新读取视频失败")
                rclpy.shutdown()
                return

        self._frame_index += 1
        msg = self._bridge.cv2_to_imgmsg(frame, "bgr8")
        msg.header.frame_id = "camera_frame"
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(
        description="将视频文件发布为 ROS2 图像话题"
    )
    parser.add_argument(
        "--path",
        required=True,
        help="视频文件路径",
    )
    parser.add_argument(
        "--topic",
        default="/sony/image_raw",
        help="输出图像话题（默认: /sony/image_raw）",
    )
    parser.add_argument(
        "--no-loop",
        action="store_true",
        help="播放完不循环，直接退出",
    )
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = VideoPublisher(
        video_path=parsed.path,
        topic=parsed.topic,
        loop=not parsed.no_loop,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
