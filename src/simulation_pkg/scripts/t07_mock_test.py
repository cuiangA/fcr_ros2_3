#!/usr/bin/env python3
"""Standalone mock data publisher for T07 Foxglove visualizer testing.

Publishes everything the perception_visualizer_node needs to render:
  - Synthetic test image (simulated person + background)
  - Mock TargetArray tracks (CONFIRMED person, position cycles left/right)
  - Mock AimTarget2D (source cycles: UPPER_BODY → FACE → LOST_PRED → UPPER_BODY)
  - CameraInfo

Usage:
  # Terminal 1: start visualizer + foxglove
  ros2 run remote_monitor_pkg perception_visualizer_node --ros-args \
    -r image:=/test/image_raw -r tracks:=/test/tracks \
    -r aim_target:=/test/aim_target_2d -p remote_max_width:=640

  # Terminal 2: start this mock publisher
  ros2 run simulation_pkg t07_mock_test.py
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from vision_servo_msgs.msg import AimTarget2D, Target, TargetArray


class T07MockTest(Node):
    def __init__(self):
        super().__init__("t07_mock_test")
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("image_topic", "/test/image_raw")
        self.declare_parameter("tracks_topic", "/test/tracks")
        self.declare_parameter("aim_topic", "/test/aim_target_2d")

        rate = float(self.get_parameter("publish_rate_hz").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.image_pub = self.create_publisher(
            Image, str(self.get_parameter("image_topic").value), sensor_qos)
        self.tracks_pub = self.create_publisher(
            TargetArray, str(self.get_parameter("tracks_topic").value), sensor_qos)
        self.aim_pub = self.create_publisher(
            AimTarget2D, str(self.get_parameter("aim_topic").value), sensor_qos)
        self.camera_pub = self.create_publisher(
            CameraInfo, "/test/camera_info", sensor_qos)

        self.WIDTH, self.HEIGHT = 640, 480
        self.started_at = time.monotonic()

        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        self.get_logger().info(
            "T07 mock test ready → /test/image_raw + /test/tracks + /test/aim_target_2d")

    def _phase(self, elapsed):
        if elapsed < 3.0:
            return 0  # person center + UPPER_BODY
        elif elapsed < 6.0:
            return 1  # person right + UPPER_BODY
        elif elapsed < 9.0:
            return 2  # person center + FACE
        elif elapsed < 12.0:
            return 3  # person center + LOST_PREDICTION (valid=false)
        else:
            return 4  # person center + UPPER_BODY resumed

    def _make_image(self, phase, elapsed):
        img = np.zeros((self.HEIGHT, self.WIDTH, 3), dtype=np.uint8)

        for y in range(self.HEIGHT):
            t = y / self.HEIGHT
            b = int(80 + t * 20)
            g = int(100 + t * 40)
            r = int(40 + t * 10)
            img[y, :] = (b, g, r)

        cx = self.WIDTH // 2
        if phase == 1:
            offset = int(100 * (1.0 + np.sin(elapsed * 0.5)))
            cx += offset

        person_w = 120
        person_h = 320
        px1 = cx - person_w // 2
        py1 = 100
        px2 = cx + person_w // 2
        py2 = py1 + person_h

        color_body = (100, 150, 200)
        color_face = (180, 200, 220)
        color_eye = (0, 0, 0)

        cv2.rectangle(img, (px1, py1), (px2, py2), color_body, -1)
        face_size = 60
        fx1 = cx - face_size // 2
        fy1 = py1
        fx2 = cx + face_size // 2
        fy2 = py1 + face_size
        cv2.rectangle(img, (fx1, fy1), (fx2, fy2), color_face, -1)
        cv2.circle(img, (cx - 12, fy1 + 30), 3, color_eye, -1)
        cv2.circle(img, (cx + 12, fy1 + 30), 3, color_eye, -1)

        noise = np.random.randint(0, 8, img.shape, dtype=np.uint8)
        img = cv2.add(img, noise)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "test_camera"
        msg.height = self.HEIGHT
        msg.width = self.WIDTH
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = self.WIDTH * 3
        msg.data = img.tobytes()
        return msg

    def _make_camera_info(self, stamp):
        ci = CameraInfo()
        ci.header.stamp = stamp
        ci.header.frame_id = "test_camera"
        ci.width = self.WIDTH
        ci.height = self.HEIGHT
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = self.WIDTH * 0.6
        fy = self.WIDTH * 0.6
        cx = self.WIDTH / 2.0
        cy = self.HEIGHT / 2.0
        ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def _make_tracks(self, stamp, phase, cx):
        person_w = 120
        person_h = 320
        px1 = cx - person_w // 2
        py1 = 100
        px2 = cx + person_w // 2
        py2 = py1 + person_h

        t = Target()
        t.header.stamp = stamp
        t.header.frame_id = "test_camera"
        t.id = 1
        t.class_name = "person"
        t.tracking_state = Target.TRACKING_STATE_CONFIRMED
        t.visible = True
        t.bbox = [float(px1), float(py1), float(px2), float(py2)]
        t.center = [float(cx), float((py1 + py2) / 2)]
        t.confidence = 0.92
        t.height = float(person_h)
        t.width = float(person_w)
        t.position = [0.0, 0.0, 3.0]
        t.velocity = [0.0, 0.0, 0.0]
        t.depth_confidence = 1.0

        arr = TargetArray()
        arr.header.stamp = stamp
        arr.header.frame_id = "test_camera"
        arr.tracking_id = 1
        arr.targets = [t]
        return arr

    def _make_aim(self, stamp, phase, cx):
        a = AimTarget2D()
        a.header.stamp = stamp
        a.header.frame_id = "test_camera"
        a.tracking_id = 1
        a.confidence = 0.92

        if phase == 0:
            a.pixel_x = float(cx)
            a.pixel_y = 120.0
            a.source = AimTarget2D.UPPER_BODY
            a.valid = True
        elif phase == 1:
            a.pixel_x = float(cx)
            a.pixel_y = 120.0
            a.source = AimTarget2D.UPPER_BODY
            a.valid = True
        elif phase == 2:
            a.pixel_x = float(cx)
            a.pixel_y = 130.0
            a.source = AimTarget2D.FACE
            a.valid = True
        elif phase == 3:
            a.pixel_x = float(cx)
            a.pixel_y = 240.0
            a.source = AimTarget2D.LOST_PREDICTION
            a.valid = False
        else:
            a.pixel_x = float(cx)
            a.pixel_y = 120.0
            a.source = AimTarget2D.UPPER_BODY
            a.valid = True
        return a

    def publish_frame(self):
        elapsed = time.monotonic() - self.started_at
        phase = self._phase(elapsed)
        phase_names = ["UPPER_BODY", "UPPER_BODY(R)", "FACE", "LOST", "RESUMED"]

        cx = self.WIDTH // 2
        if phase == 1:
            offset = int(100 * (1.0 + np.sin(elapsed * 1.5)))
            cx += offset

        image_msg = self._make_image(phase, elapsed)
        stamp = image_msg.header.stamp

        self.image_pub.publish(image_msg)
        self.camera_pub.publish(self._make_camera_info(stamp))
        self.tracks_pub.publish(self._make_tracks(stamp, phase, cx))
        self.aim_pub.publish(self._make_aim(stamp, phase, cx))

        self.get_logger().info(
            f"Phase {phase}: {phase_names[phase]}  person_cx={cx}",
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = T07MockTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
