"""Automatic acceptance wrapper for MVP gimbal plus chassis-yaw mode."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("bringup_pkg"),
                "launch",
                "mvp_mode_acceptance.launch.py",
            ]),
            launch_arguments={"mode": "cooperative_yaw"}.items(),
        )
    ])
