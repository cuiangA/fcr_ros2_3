"""Launch the safe MVP controller with a selected mode configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
        parameters=[
            LaunchConfiguration("mvp_config"),
            {
                "yaw_sign": ParameterValue(
                    LaunchConfiguration("yaw_sign"), value_type=float
                ),
                "pitch_sign": ParameterValue(
                    LaunchConfiguration("pitch_sign"), value_type=float
                ),
                "base_yaw_sign": ParameterValue(
                    LaunchConfiguration("base_yaw_sign"), value_type=float
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mvp_config",
            default_value=default_config,
            description="Absolute path to one safe MVP mode YAML file.",
        ),
        DeclareLaunchArgument(
            "yaw_sign",
            default_value="1.0",
            description="RS2 yaw direction multiplier; calibrated on real hardware.",
        ),
        DeclareLaunchArgument(
            "pitch_sign",
            default_value="-1.0",
            description="RS2 pitch direction multiplier; calibrate before full testing.",
        ),
        DeclareLaunchArgument(
            "base_yaw_sign",
            default_value="-1.0",
            description="Direction from relative RS2 yaw to ROS base angular.z.",
        ),
        controller,
    ])
