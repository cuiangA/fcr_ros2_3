#!/usr/bin/env python3

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

from vision_servo_msgs.msg import Target, TargetArray
from vision_servo_msgs.srv import SetTrackingTarget


@pytest.mark.launch_test
def generate_test_description():
    tracking = launch_ros.actions.Node(
        package="perception_pkg",
        executable="tracking_node",
        name="tracking_node",
        parameters=[
            {
                "tracker_type": "bytetrack",
                "min_confirm_hits": 1,
                "new_track_delay_frames": 1,
                "lost_timeout_seconds": 0.2,
                "enable_camera_motion_compensation": False,
                "auto_select": True,
                "class_filter": "person",
            }
        ],
        remappings=[
            ("detections", "/test/detections"),
            ("tracks", "/test/tracks"),
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [tracking, launch_testing.actions.ReadyToTest()]
        ),
        {"tracking": tracking},
    )


class TestTrackingRosChain(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("tracking_launch_test")

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def test_tracks_and_service_contract(self):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        received = []
        subscription = self.node.create_subscription(
            TargetArray, "/test/tracks", received.append, qos
        )
        publisher = self.node.create_publisher(
            TargetArray, "/test/detections", qos
        )
        self.assertTrue(
            self._wait_for(lambda: publisher.get_subscription_count() > 0),
            "tracking_node did not subscribe to mock detections",
        )

        frame = TargetArray()
        frame.header.stamp.sec = 123
        frame.header.stamp.nanosec = 456
        frame.header.frame_id = "sony_camera_optical_frame"
        detection = Target()
        detection.id = -1
        detection.class_name = "person"
        detection.tracking_state = Target.TRACKING_STATE_UNTRACKED
        detection.visible = True
        detection.confidence = 0.9
        detection.center = [320.0, 240.0]
        detection.bbox = [270.0, 140.0, 370.0, 340.0]
        detection.width = 100.0
        detection.height = 200.0
        frame.targets = [detection]
        publisher.publish(frame)

        self.assertTrue(self._wait_for(lambda: len(received) > 0))
        tracks = received[-1]
        self.assertEqual(tracks.header, frame.header)
        self.assertEqual(len(tracks.targets), 1)
        self.assertEqual(tracks.targets[0].header, frame.header)
        self.assertTrue(tracks.targets[0].visible)
        self.assertEqual(
            tracks.targets[0].tracking_state, Target.TRACKING_STATE_CONFIRMED
        )
        self.assertEqual(tracks.tracking_id, tracks.targets[0].id)

        client = self.node.create_client(
            SetTrackingTarget, "/tracking_node/set_tracking_target"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        request = SetTrackingTarget.Request()
        request.target_id = 9999
        request.class_name = "person"
        request.enable = True
        future = client.call_async(request)
        self.assertTrue(self._wait_for(future.done))
        self.assertFalse(future.result().success)

        self.node.destroy_subscription(subscription)
        self.node.destroy_publisher(publisher)
        self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestTrackingShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
