"""Launch the MVP controller with a synthetic TargetArray publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    target_topic = LaunchConfiguration("target_topic")
    use_twist_stamped = LaunchConfiguration("use_twist_stamped")
    mock_q_yaw = LaunchConfiguration("mock_q_yaw")
    mock_q_pitch = LaunchConfiguration("mock_q_pitch")

    controller_config = PathJoinSubstitution([
        FindPackageShare("servo_control_pkg"),
        "config",
        "mvp_follow_controller.yaml",
    ])

    mock_target_publisher = Node(
        package="simulation_pkg",
        executable="mock_target_publisher_node.py",
        name="mock_target_publisher_node",
        output="screen",
        parameters=[{
            "target_topic": target_topic,
            "scenario": scenario,
            "image_width": 640,
            "image_height": 480,
            "desired_distance": 2.0,
        }],
    )

    mvp_controller = Node(
        package="servo_control_pkg",
        executable="mvp_follow_controller_node",
        name="mvp_follow_controller_node",
        output="screen",
        parameters=[
            controller_config,
            {
                "target_topic": target_topic,
                "use_twist_stamped": ParameterValue(use_twist_stamped, value_type=bool),
                "use_mock_gimbal_state": True,
                "mock_q_yaw": ParameterValue(mock_q_yaw, value_type=float),
                "mock_q_pitch": ParameterValue(mock_q_pitch, value_type=float),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value="center",
            description="Mock target scenario: center/left/right/up/down/far/near/lost/sinusoidal.",
        ),
        DeclareLaunchArgument(
            "target_topic",
            default_value="/target/current",
            description="TargetArray topic used by the mock publisher and MVP controller.",
        ),
        DeclareLaunchArgument(
            "use_twist_stamped",
            default_value="true",
            description="Publish /cmd_vel as TwistStamped when true, Twist when false.",
        ),
        DeclareLaunchArgument(
            "mock_q_yaw",
            default_value="0.0",
            description="Mock gimbal yaw angle for base yaw control tests.",
        ),
        DeclareLaunchArgument(
            "mock_q_pitch",
            default_value="0.0",
            description="Mock gimbal pitch angle.",
        ),
        mock_target_publisher,
        mvp_controller,
    ])
