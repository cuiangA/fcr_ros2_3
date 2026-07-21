"""Isolated real-RS2 speed test: no command mux, camera, servo, or chassis."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gimbal = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("robot_platform_pkg"),
            "launch",
            "gimbal_bringup.launch.py",
        ]),
        launch_arguments={
            "use_sim": "false",
            "can_interface": LaunchConfiguration("can_interface"),
            "control_mode": "speed",
            "speed_control_byte": LaunchConfiguration("speed_control_byte"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can1"),
        DeclareLaunchArgument(
            "speed_control_byte",
            default_value="128",
            description="DJI RS2 speed-control byte (128 = 0x80).",
        ),
        gimbal,
    ])
