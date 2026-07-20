"""Start only the LeKiwi chassis driver and safety mux for initial acceptance."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    chassis_config = PathJoinSubstitution(
        [FindPackageShare("robot_platform_pkg"), "config", "chassis_params.yaml"]
    )
    chassis = Node(
        package="robot_platform_pkg",
        executable="chassis_driver_node",
        name="chassis_driver",
        output="screen",
        parameters=[chassis_config],
    )
    remote_control = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("teleop_control_pkg"), "launch", "remote_control.launch.py"]
        ),
        launch_arguments={"start_keyboard": "false"}.items(),
    )
    return LaunchDescription([chassis, remote_control])
