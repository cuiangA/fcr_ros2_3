# robot_platform_pkg/launch/gimbal_bringup.launch.py
"""
Start only the gimbal driver path for RS2 bring-up and debugging.

Examples:
  ros2 launch robot_platform_pkg gimbal_bringup.launch.py use_sim:=true
  ros2 launch robot_platform_pkg gimbal_bringup.launch.py use_sim:=false can_interface:=can0
  ros2 launch robot_platform_pkg gimbal_bringup.launch.py control_mode:=speed
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    can_interface = LaunchConfiguration("can_interface")
    autostart = LaunchConfiguration("autostart")
    control_mode = LaunchConfiguration("control_mode")

    config_dir = PathJoinSubstitution([
        FindPackageShare("robot_platform_pkg"),
        "config",
    ])

    gimbal_node = Node(
        package="robot_platform_pkg",
        executable="gimbal_driver_node",
        name="gimbal_driver",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir, "gimbal_params.yaml"]),
            {
                "autostart": autostart,
                "use_sim": use_sim,
                "can_interface": can_interface,
                "control_mode": control_mode,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically configure and activate the lifecycle gimbal driver.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use simulated gimbal backend instead of SocketCAN hardware.",
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="Linux SocketCAN interface used by the RS2 gimbal.",
        ),
        DeclareLaunchArgument(
            "control_mode",
            default_value="incremental_position",
            description="Gimbal command mode: speed or native incremental_position.",
        ),
        gimbal_node,
    ])
