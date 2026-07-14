"""Sony RGB detection and tracking launch.

Camera drivers and RGB/depth fusion are intentionally outside the current
scope. Topic arguments allow the same graph to consume a live Sony camera,
rosbag2 playback, a video publisher, or simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("perception_pkg")
    use_sim_time = LaunchConfiguration("use_sim_time")
    detections_topic = LaunchConfiguration("detections_topic")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_detection", default_value="true"),
        DeclareLaunchArgument("enable_tracking", default_value="true"),
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution(
                [package_share, "models", "yolov8n.onnx"]
            ),
            description="Absolute or package-resolved YOLO ONNX model path",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="cpu",
            description="Inference backend: cpu, cuda_fp16, cuda_fp32, or tensorrt",
        ),
        DeclareLaunchArgument("confidence_threshold", default_value="0.5"),
        DeclareLaunchArgument(
            "detection_params",
            default_value=PathJoinSubstitution(
                [package_share, "config", "detection_params.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "tracking_params",
            default_value=PathJoinSubstitution(
                [package_share, "config", "tracking_params.yaml"]
            ),
        ),
        DeclareLaunchArgument("sony_image_topic", default_value="/sony/image_raw"),
        DeclareLaunchArgument("detections_topic", default_value="/perception/detections"),
        DeclareLaunchArgument("tracks_topic", default_value="/perception/tracks"),
        DeclareLaunchArgument("debug_image_topic", default_value="/perception/debug_image"),
    ]

    detection_node = Node(
        package="perception_pkg",
        executable="detection_node",
        name="detection_node",
        output="screen",
        parameters=[
            LaunchConfiguration("detection_params"),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "model_path": LaunchConfiguration("model_path"),
                "device": LaunchConfiguration("device"),
                "confidence_threshold": ParameterValue(
                    LaunchConfiguration("confidence_threshold"), value_type=float
                ),
            },
        ],
        remappings=[
            ("image", LaunchConfiguration("sony_image_topic")),
            ("detections", detections_topic),
            ("debug_image", LaunchConfiguration("debug_image_topic")),
        ],
        condition=IfCondition(LaunchConfiguration("enable_detection")),
    )

    tracking_node = Node(
        package="perception_pkg",
        executable="tracking_node",
        name="tracking_node",
        output="screen",
        parameters=[
            LaunchConfiguration("tracking_params"),
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ("detections", detections_topic),
            ("tracks", LaunchConfiguration("tracks_topic")),
        ],
        condition=IfCondition(LaunchConfiguration("enable_tracking")),
    )

    return LaunchDescription(arguments + [detection_node, tracking_node])
