"""Complete real-hardware test mode: gimbal tracking plus chassis yaw only."""

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
                "enable_chassis": "true",
                "mvp_config": PathJoinSubstitution([
                    FindPackageShare("servo_control_pkg"),
                    "config",
                    "mvp_cooperative_yaw.yaml",
                ])
            }.items(),
        )
    ])
