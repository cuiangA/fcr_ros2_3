"""Launch the Sony CRSDK driver and optionally the complete 2D perception chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("sony_camera_pkg")
    perception_share = FindPackageShare("perception_pkg")

    arguments = [
        DeclareLaunchArgument("camera_index", default_value="1"),
        DeclareLaunchArgument("camera_frame", default_value="sony_camera_optical_frame"),
        DeclareLaunchArgument("camera_info_url", default_value=""),
        DeclareLaunchArgument("image_topic", default_value="/sony/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/sony/camera_info"),
        DeclareLaunchArgument("enable_perception", default_value="false"),
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution(
                [perception_share, "models", "yolov8n.onnx"]
            ),
        ),
        DeclareLaunchArgument("device", default_value="cpu"),
    ]

    sony_node = Node(
        package="sony_camera_pkg",
        executable="sony_camera_node",
        name="sony_camera_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([package_share, "config", "sony_camera_params.yaml"]),
            {
                "camera_index": ParameterValue(
                    LaunchConfiguration("camera_index"), value_type=int
                ),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "camera_info_url": LaunchConfiguration("camera_info_url"),
            },
        ],
        remappings=[
            ("image_raw", LaunchConfiguration("image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
            ("set_camera_info", "/sony/set_camera_info"),
        ],
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([perception_share, "launch", "perception.launch.py"])
        ),
        launch_arguments={
            "sony_image_topic": LaunchConfiguration("image_topic"),
            "model_path": LaunchConfiguration("model_path"),
            "device": LaunchConfiguration("device"),
            "enable_detection": "true",
            "enable_tracking": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_perception")),
    )

    return LaunchDescription(arguments + [sony_node, perception_launch])
