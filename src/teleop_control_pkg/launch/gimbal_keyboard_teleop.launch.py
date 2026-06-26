"""Start the gimbal keyboard teleop path.

This launch file can start the platform gimbal driver and the keyboard teleop
node together. If the keyboard node cannot read keys through ros2 launch on a
given terminal, run the gimbal driver with launch and start teleop with ros2 run
in a second interactive terminal.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_gimbal_driver = LaunchConfiguration("start_gimbal_driver")
    use_sim = LaunchConfiguration("use_sim")

    platform_config = PathJoinSubstitution([
        FindPackageShare("robot_platform_pkg"), "config"
    ])
    teleop_config = PathJoinSubstitution([
        FindPackageShare("teleop_control_pkg"), "config",
        "gimbal_keyboard_teleop.yaml",
    ])

    gimbal_driver = Node(
        package="robot_platform_pkg",
        executable="gimbal_driver_node",
        name="gimbal_driver",
        output="screen",
        parameters=[
            PathJoinSubstitution([platform_config, "gimbal_params.yaml"]),
            {"use_sim": use_sim},
        ],
        condition=IfCondition(start_gimbal_driver),
    )

    keyboard_teleop = Node(
        package="teleop_control_pkg",
        executable="keyboard_gimbal_teleop",
        name="keyboard_gimbal_teleop",
        output="screen",
        emulate_tty=True,
        parameters=[teleop_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_gimbal_driver",
            default_value="true",
            description="Whether to start robot_platform_pkg/gimbal_driver_node."),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Use the simulated gimbal backend while RS2 SDK is TODO."),
        gimbal_driver,
        keyboard_teleop,
    ])
