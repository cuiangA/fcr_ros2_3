"""Common complete real-hardware bringup for one safe MVP control mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    servo_share = FindPackageShare("servo_control_pkg")

    system = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("bringup_pkg"), "launch", "fcr_bringup.launch.py"
        ]),
        launch_arguments={
            "use_sim": "false",
            "can_interface": LaunchConfiguration("can_interface"),
            "enable_imu": "false",
            "enable_chassis": LaunchConfiguration("enable_chassis"),
            "model_path": LaunchConfiguration("model_path"),
            "detection_device": LaunchConfiguration("detection_device"),
            "enable_camera_motion_compensation": LaunchConfiguration(
                "enable_camera_motion_compensation"
            ),
            "enable_servo": "false",
            "sony_camera_info_url": LaunchConfiguration("sony_camera_info_url"),
            "use_foxglove": LaunchConfiguration("use_foxglove"),
            "foxglove_address": LaunchConfiguration("foxglove_address"),
            "foxglove_port": LaunchConfiguration("foxglove_port"),
            "remote_max_frame_age_ms": LaunchConfiguration(
                "remote_max_frame_age_ms"
            ),
            "use_rviz": "false",
        }.items(),
    )

    mvp = IncludeLaunchDescription(
        PathJoinSubstitution([
            servo_share, "launch", "mvp_follow_core.launch.py"
        ]),
        launch_arguments={"mvp_config": LaunchConfiguration("mvp_config")}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can1"),
        DeclareLaunchArgument("enable_chassis", default_value="false"),
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"), "fcr_models", "yolov8n_fp16.engine"
            ]),
        ),
        DeclareLaunchArgument("detection_device", default_value="tensorrt"),
        DeclareLaunchArgument(
            "enable_camera_motion_compensation", default_value="false"
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
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("remote_max_frame_age_ms", default_value="1000"),
        DeclareLaunchArgument(
            "mvp_config",
            default_value=PathJoinSubstitution([
                servo_share, "config", "mvp_gimbal_only.yaml"
            ]),
        ),
        system,
        TimerAction(period=4.0, actions=[mvp]),
    ])
