"""Minimal real-hardware manual test: can1 gimbal driver plus command mux."""

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
        }.items(),
    )
    mux = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("teleop_control_pkg"),
            "launch",
            "remote_control.launch.py",
        ]),
        launch_arguments={"start_keyboard": "false"}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can1"),
        gimbal,
        mux,
    ])
