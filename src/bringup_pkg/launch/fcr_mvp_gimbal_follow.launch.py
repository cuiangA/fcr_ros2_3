"""Complete real-hardware test mode: 2D person tracking with gimbal only."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("bringup_pkg"), "launch", "fcr_mvp_mode.launch.py"
            ]),
            launch_arguments={
                "mvp_config": PathJoinSubstitution([
                    FindPackageShare("servo_control_pkg"),
                    "config",
                    "mvp_gimbal_only.yaml",
                ])
            }.items(),
        )
    ])
