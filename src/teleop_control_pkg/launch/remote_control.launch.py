"""Start the Jetson-side safety command mux and optional terminal keyboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("teleop_control_pkg"), "config", "remote_control.yaml"]
    )
    mux = Node(
        package="teleop_control_pkg",
        executable="command_mux_node",
        name="command_mux",
        output="screen",
        parameters=[config],
    )
    keyboard = Node(
        package="teleop_control_pkg",
        executable="keyboard_platform_teleop",
        name="keyboard_platform_teleop",
        output="screen",
        emulate_tty=True,
        parameters=[config],
        condition=IfCondition(LaunchConfiguration("start_keyboard")),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_keyboard",
            default_value="false",
            description="Start terminal keyboard; ros2 run in an interactive shell is preferred.",
        ),
        mux,
        keyboard,
    ])
