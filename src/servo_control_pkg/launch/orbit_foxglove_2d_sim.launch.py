"""2D orbit-shot verification scene for Foxglove visualization.

Target motion: slow straight line.
Robot behavior: continuous orbit around the moving target while the gimbal tracks it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target_speed = LaunchConfiguration("target_speed")
    target_heading_deg = LaunchConfiguration("target_heading_deg")
    orbit_radius = LaunchConfiguration("orbit_radius")
    orbit_duration = LaunchConfiguration("orbit_duration")
    foxglove_bridge = LaunchConfiguration("foxglove_bridge")
    foxglove_port = LaunchConfiguration("foxglove_port")

    perception_config = PathJoinSubstitution([
        FindPackageShare("perception_pkg"),
        "config",
        "target_state_params.yaml",
    ])
    shot_config = PathJoinSubstitution([
        FindPackageShare("servo_control_pkg"),
        "config",
        "shot_reference_params.yaml",
    ])
    base_config = PathJoinSubstitution([
        FindPackageShare("servo_control_pkg"),
        "config",
        "base_pose_controller.yaml",
    ])
    gimbal_config = PathJoinSubstitution([
        FindPackageShare("servo_control_pkg"),
        "config",
        "gimbal_target_controller.yaml",
    ])
    safety_config = PathJoinSubstitution([
        FindPackageShare("servo_control_pkg"),
        "config",
        "safety_filter.yaml",
    ])

    simple_robot_sim = Node(
        package="simulation_pkg",
        executable="simple_robot_sim_node.py",
        name="simple_robot_sim_node",
        output="screen",
        parameters=[{
            "target_topic": "/target/current",
            "platform_state_topic": "/platform/state",
            "cmd_vel_topic": "/cmd_vel",
            "cmd_gimbal_topic": "/cmd_gimbal",
            "use_twist_stamped": True,
            "shot_reference_topic": "/shot/reference",
            "image_width": 640,
            "image_height": 480,
            "desired_distance": ParameterValue(orbit_radius, value_type=float),
            "initial_robot_x": 0.0,
            "initial_robot_y": 0.0,
            "initial_robot_yaw": 0.0,
            "target_x": 2.8,
            "target_y": -1.2,
            "target_motion": "linear",
            "target_speed": ParameterValue(target_speed, value_type=float),
            "target_radius": 1.0,
            "target_line_heading_deg": ParameterValue(target_heading_deg, value_type=float),
            "target_line_loop_distance": 0.0,
            "publish_tf": True,
            "publish_odom": True,
            "publish_joint_states": True,
            "publish_markers": True,
            "marker_topic": "/visualization_marker",
            "marker_array_topic": "/visualization_marker_array",
        }],
    )

    target_state_estimator = Node(
        package="perception_pkg",
        executable="target_state_estimator_node",
        name="target_state_estimator_node",
        output="screen",
        parameters=[perception_config],
    )

    shot_reference_generator = Node(
        package="servo_control_pkg",
        executable="shot_reference_generator_node",
        name="shot_reference_generator_node",
        output="screen",
        parameters=[
            shot_config,
            {
                "auto_start": True,
                "loop_auto_start_shot": True,
                "default_shot_name": "orbit",
                "default_duration": ParameterValue(orbit_duration, value_type=float),
                "default_radius": ParameterValue(orbit_radius, value_type=float),
                "default_angle_deg": 360.0,
                "max_base_speed": 0.9,
                "target_speed_fallback_threshold": 2.0,
            },
        ],
    )

    base_pose_controller = Node(
        package="servo_control_pkg",
        executable="base_pose_controller_node",
        name="base_pose_controller_node",
        output="screen",
        parameters=[base_config, {
            "max_vx": 0.9,
            "max_vy": 0.7,
            "holonomic": True,
        }],
    )

    gimbal_target_controller = Node(
        package="servo_control_pkg",
        executable="gimbal_target_controller_node",
        name="gimbal_target_controller_node",
        output="screen",
        parameters=[gimbal_config],
    )

    safety_filter = Node(
        package="servo_control_pkg",
        executable="command_safety_filter_node",
        name="command_safety_filter_node",
        output="screen",
        parameters=[safety_config, {
            "max_vx": 0.9,
            "max_vy": 0.7,
        }],
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        condition=IfCondition(foxglove_bridge),
        parameters=[{
            "port": ParameterValue(foxglove_port, value_type=int),
            "address": "0.0.0.0",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "target_speed",
            default_value="0.06",
            description="Straight-line target speed in m/s.",
        ),
        DeclareLaunchArgument(
            "target_heading_deg",
            default_value="90.0",
            description="Straight-line target heading in odom frame.",
        ),
        DeclareLaunchArgument(
            "orbit_radius",
            default_value="2.0",
            description="Robot orbit radius around the moving target in meters.",
        ),
        DeclareLaunchArgument(
            "orbit_duration",
            default_value="28.0",
            description="Seconds per 360-degree orbit.",
        ),
        DeclareLaunchArgument(
            "foxglove_bridge",
            default_value="false",
            description="Start foxglove_bridge if the package is installed.",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove WebSocket bridge port.",
        ),
        simple_robot_sim,
        target_state_estimator,
        shot_reference_generator,
        base_pose_controller,
        gimbal_target_controller,
        safety_filter,
        foxglove_bridge_node,
    ])
