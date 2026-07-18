"""Phase-1 acceptance: mock 3D target through servo, mux, and simulated chassis."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    active_duration = LaunchConfiguration("active_duration_sec")
    max_stop_latency = LaunchConfiguration("max_stop_latency_sec")

    platform = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("robot_platform_pkg"), "launch", "platform.launch.py"
        ]),
        launch_arguments={"use_sim": "true", "enable_imu": "true"}.items(),
    )

    mock_source = Node(
        package="simulation_pkg",
        executable="servo_chain_mock_source.py",
        name="servo_chain_mock_source",
        output="screen",
        parameters=[{
            "start_delay_sec": 2.0,
            "active_duration_sec": ParameterValue(active_duration, value_type=float),
        }],
    )

    servo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("servo_control_pkg"), "launch", "servo_control.launch.py"
        ]),
        launch_arguments={
            "controller_plugin": "servo_control_pkg::PBVSController",
            "allocation_ratio": "1.0",
            "auto_start": "true",
            "target_timeout": "0.20",
            "camera_info_input": "/sony/camera_info",
            "cmd_vel_output": "/auto/cmd_vel",
            "cmd_gimbal_output": "/auto/cmd_gimbal",
            "enable_velocity_commander": "false",
        }.items(),
    )

    mux_config = PathJoinSubstitution([
        FindPackageShare("teleop_control_pkg"), "config", "remote_control.yaml"
    ])
    command_mux = Node(
        package="teleop_control_pkg",
        executable="command_mux_node",
        name="command_mux",
        output="screen",
        parameters=[mux_config, {"default_mode": "auto"}],
    )

    monitor = Node(
        package="simulation_pkg",
        executable="servo_chain_acceptance.py",
        name="servo_chain_acceptance",
        output="screen",
        parameters=[{
            "max_stop_latency_sec": ParameterValue(
                max_stop_latency, value_type=float),
        }],
    )

    stop_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor,
            on_exit=[EmitEvent(event=Shutdown(
                reason="phase-1 acceptance monitor finished"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "active_duration_sec", default_value="3.0",
            description="Duration of mock target publication"),
        DeclareLaunchArgument(
            "max_stop_latency_sec", default_value="0.30",
            description="Maximum allowed target-loss to final-zero latency"),
        platform,
        command_mux,
        servo,
        mock_source,
        monitor,
        stop_when_done,
    ])
