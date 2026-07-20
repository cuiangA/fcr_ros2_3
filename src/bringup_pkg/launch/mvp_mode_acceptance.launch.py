"""Deterministic automatic acceptance for one safe MVP mode."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    configs = {
        "gimbal": "mvp_gimbal_only.yaml",
        "cooperative_yaw": "mvp_cooperative_yaw.yaml",
        "full": "mvp_full_follow.yaml",
    }
    if mode not in configs:
        raise ValueError("mode must be gimbal, cooperative_yaw, or full")

    servo_share = get_package_share_directory("servo_control_pkg")
    teleop_share = get_package_share_directory("teleop_control_pkg")
    target_mode = "3d" if mode == "full" else "2d"
    target_topic = "/perception/targets_3d" if mode == "full" else "/perception/tracks"
    platform_yaw = 0.30 if mode in ("cooperative_yaw", "full") else 0.0

    source = Node(
        package="simulation_pkg",
        executable="servo_chain_mock_source.py",
        name="mvp_mock_source",
        output="screen",
        parameters=[{
            "target_mode": target_mode,
            "target_topic": target_topic,
            "platform_yaw": platform_yaw,
            "start_delay_sec": 1.0,
            "active_duration_sec": 3.0,
        }],
    )
    mux = Node(
        package="teleop_control_pkg",
        executable="command_mux_node",
        name="command_mux",
        output="screen",
        parameters=[
            f"{teleop_share}/config/remote_control.yaml",
            {"default_mode": "auto", "zero_dwell_ms": 0},
        ],
    )
    controller = Node(
        package="servo_control_pkg",
        executable="mvp_follow_controller_node",
        name="mvp_follow_controller_node",
        output="screen",
        parameters=[f"{servo_share}/config/{configs[mode]}"],
    )
    monitor = Node(
        package="simulation_pkg",
        executable="mvp_mode_acceptance.py",
        name="mvp_mode_acceptance",
        output="screen",
        parameters=[{"mode": mode}],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor,
            on_exit=[EmitEvent(event=Shutdown(reason="MVP acceptance completed"))],
        )
    )
    return [source, mux, controller, monitor, shutdown]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="gimbal"),
        OpaqueFunction(function=_setup),
    ])
