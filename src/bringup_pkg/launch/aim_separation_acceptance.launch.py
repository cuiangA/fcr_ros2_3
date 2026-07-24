"""Deterministic automatic acceptance for T06: gimbal/chassis target separation.

Validates:
  1. Aim point changes do NOT affect chassis distance control
  2. Aim target timeout (250ms) causes gimbal to stop
  3. Gimbal does not retain non-zero velocity after timeout
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def _setup(context):
    servo_share = get_package_share_directory("servo_control_pkg")
    config = f"{servo_share}/config/mvp_gimbal_only.yaml"

    source = Node(
        package="simulation_pkg",
        executable="aim_mock_source.py",
        name="aim_mock_source",
        output="screen",
    )
    controller = Node(
        package="servo_control_pkg",
        executable="mvp_follow_controller_node",
        name="mvp_follow_controller_node",
        output="screen",
        parameters=[config],
    )
    monitor = Node(
        package="simulation_pkg",
        executable="aim_separation_acceptance.py",
        name="aim_separation_acceptance",
        output="screen",
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor,
            on_exit=[EmitEvent(event=Shutdown(reason="aim separation acceptance completed"))],
        )
    )
    return [source, controller, monitor, shutdown]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
