"""Launch the safe MVP controller with a selected mode configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("servo_control_pkg"),
        "config",
        "mvp_gimbal_only.yaml",
    ])
    controller = Node(
        package="servo_control_pkg",
        executable="mvp_follow_controller_node",
        name="mvp_follow_controller_node",
        output="screen",
        parameters=[LaunchConfiguration("mvp_config")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mvp_config",
            default_value=default_config,
            description="Absolute path to one safe MVP mode YAML file.",
        ),
        controller,
    ])
