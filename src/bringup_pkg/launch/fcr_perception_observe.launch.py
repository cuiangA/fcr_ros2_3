"""Complete read-only Sony + TensorRT + tracking + Foxglove observation mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("sony_camera_pkg"), "launch", "sony_camera.launch.py"
        ]),
        launch_arguments={
            "enable_perception": "false",
            "camera_info_url": LaunchConfiguration("sony_camera_info_url"),
            "image_topic": "/sony/image_raw",
        }.items(),
    )
    perception = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("perception_pkg"), "launch", "perception.launch.py"
        ]),
        launch_arguments={
            "model_path": LaunchConfiguration("model_path"),
            "device": "tensorrt",
            "enable_camera_motion_compensation": "false",
            "sony_image_topic": "/sony/image_raw",
        }.items(),
    )
    monitor = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("remote_monitor_pkg"),
            "launch",
            "remote_monitor.launch.py",
        ]),
        launch_arguments={
            "image_topic": "/sony/image_raw",
            "model_path": LaunchConfiguration("model_path"),
            "inference_backend": "tensorrt",
            "enable_foxglove": "true",
            "foxglove_address": "0.0.0.0",
            "foxglove_port": "8765",
            "aim_target_topic": "/perception/aim_target_2d",
            "max_frame_age_ms": "1000",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"), "fcr_models", "yolov8n_fp16.engine"
            ]),
        ),
        DeclareLaunchArgument(
            "sony_camera_info_url",
            default_value=[
                TextSubstitution(text="file://"),
                PathJoinSubstitution([
                    EnvironmentVariable("HOME"),
                    ".ros",
                    "camera_info",
                    "sony_zv_e10_ii.yaml",
                ]),
            ],
        ),
        camera,
        perception,
        monitor,
    ])
