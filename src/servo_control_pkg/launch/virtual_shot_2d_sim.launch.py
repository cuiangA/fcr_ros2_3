"""Launch the virtual-shot control chain with the lightweight 2D simulator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target_motion = LaunchConfiguration("target_motion")
    target_speed = LaunchConfiguration("target_speed")
    target_radius = LaunchConfiguration("target_radius")
    shot_name = LaunchConfiguration("shot_name")
    auto_start = LaunchConfiguration("auto_start")
    rviz = LaunchConfiguration("rviz")

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
            "desired_distance": 2.0,
            "target_motion": target_motion,
            "target_speed": ParameterValue(target_speed, value_type=float),
            "target_radius": ParameterValue(target_radius, value_type=float),
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
                "default_shot_name": shot_name,
                "auto_start": ParameterValue(auto_start, value_type=bool),
            },
        ],
    )

    base_pose_controller = Node(
        package="servo_control_pkg",
        executable="base_pose_controller_node",
        name="base_pose_controller_node",
        output="screen",
        parameters=[base_config],
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
        parameters=[safety_config],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "target_motion",
            default_value="circle",
            description="Target motion pattern: static, circle, line, or figure8.",
        ),
        DeclareLaunchArgument(
            "target_speed",
            default_value="0.18",
            description="Target motion phase speed in rad/s.",
        ),
        DeclareLaunchArgument(
            "target_radius",
            default_value="1.0",
            description="Target motion radius/amplitude in meters.",
        ),
        DeclareLaunchArgument(
            "shot_name",
            default_value="orbit",
            description="Default shot: follow, orbit, dolly_in, dolly_out, truck, arc_in, pan, recenter.",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="true",
            description="Publish the default shot reference without an action goal.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz2.",
        ),
        simple_robot_sim,
        target_state_estimator,
        shot_reference_generator,
        base_pose_controller,
        gimbal_target_controller,
        safety_filter,
        rviz_node,
    ])
