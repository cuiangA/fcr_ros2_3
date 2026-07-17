"""Start the platform drivers and the safety command mux for remote operation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    platform = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("robot_platform_pkg"), "launch", "platform.launch.py"]
        ),
        launch_arguments={"use_sim": LaunchConfiguration("use_sim")}.items(),
    )
    remote_control = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("teleop_control_pkg"), "launch", "remote_control.launch.py"]
        ),
        launch_arguments={"start_keyboard": "false"}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim", default_value="false", description="Use simulated platform hardware."
        ),
        platform,
        remote_control,
    ])
