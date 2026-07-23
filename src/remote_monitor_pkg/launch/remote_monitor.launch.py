"""Launch the read-only perception visualizer and LAN Foxglove bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("remote_monitor_pkg")
    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_visualizer", default_value="true"),
        DeclareLaunchArgument("enable_foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument(
            "visualizer_params",
            default_value=PathJoinSubstitution(
                [package_share, "config", "remote_monitor_params.yaml"]
            ),
        ),
        DeclareLaunchArgument("image_topic", default_value="/sony/image_raw"),
        DeclareLaunchArgument(
            "detections_topic", default_value="/perception/detections"
        ),
        DeclareLaunchArgument("tracks_topic", default_value="/perception/tracks"),
        DeclareLaunchArgument(
            "tracking_image_topic", default_value="/perception/tracking_image"
        ),
        DeclareLaunchArgument(
            "monitor_status_topic", default_value="/perception/monitor_status"
        ),
        DeclareLaunchArgument(
            "target_3d_topic", default_value="/perception/targets_3d"
        ),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("gimbal_state_topic", default_value="/gimbal/status"),
        DeclareLaunchArgument(
            "aim_target_topic", default_value="/perception/aim_target_2d"
        ),
        DeclareLaunchArgument("enable_future_inputs", default_value="false"),
        DeclareLaunchArgument("remote_publish_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("remote_max_width", default_value="960"),
        DeclareLaunchArgument("jpeg_quality", default_value="65"),
        DeclareLaunchArgument("max_frame_age_ms", default_value="300"),
        DeclareLaunchArgument("yolo_model", default_value="yolov8n"),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("inference_backend", default_value="tensorrt"),
    ]

    visualizer = Node(
        package="remote_monitor_pkg",
        executable="perception_visualizer_node",
        name="perception_visualizer_node",
        output="screen",
        parameters=[
            LaunchConfiguration("visualizer_params"),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "yolo_model": LaunchConfiguration("yolo_model"),
                "model_path": LaunchConfiguration("model_path"),
                "inference_backend": LaunchConfiguration("inference_backend"),
                "enable_future_inputs": ParameterValue(
                    LaunchConfiguration("enable_future_inputs"), value_type=bool
                ),
                "remote_publish_rate_hz": ParameterValue(
                    LaunchConfiguration("remote_publish_rate_hz"), value_type=float
                ),
                "remote_max_width": ParameterValue(
                    LaunchConfiguration("remote_max_width"), value_type=int
                ),
                "jpeg_quality": ParameterValue(
                    LaunchConfiguration("jpeg_quality"), value_type=int
                ),
                "max_frame_age_ms": ParameterValue(
                    LaunchConfiguration("max_frame_age_ms"), value_type=int
                ),
            },
        ],
        remappings=[
            ("image", LaunchConfiguration("image_topic")),
            ("detections", LaunchConfiguration("detections_topic")),
            ("tracks", LaunchConfiguration("tracks_topic")),
            ("diagnostics", "/diagnostics"),
            # This is a direct sensor_msgs/CompressedImage publisher. There is
            # no raw remote image and no image_transport compression queue.
            (
                "tracking_image/compressed",
                PathJoinSubstitution(
                    [LaunchConfiguration("tracking_image_topic"), "compressed"]
                ),
            ),
            ("monitor_status", LaunchConfiguration("monitor_status_topic")),
            ("target_3d", LaunchConfiguration("target_3d_topic")),
            ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
            ("gimbal_state", LaunchConfiguration("gimbal_state_topic")),
            ("aim_target", LaunchConfiguration("aim_target_topic")),
        ],
        condition=IfCondition(LaunchConfiguration("enable_visualizer")),
    )

    # Observation-only bridge: no client publishing, service calls, parameter
    # writes, or arbitrary file assets are exposed over the robot LAN socket.
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "address": LaunchConfiguration("foxglove_address"),
                "port": ParameterValue(
                    LaunchConfiguration("foxglove_port"), value_type=int
                ),
                "topic_whitelist": [
                    # Never expose raw Image topics over the robot Wi-Fi link.
                    # A stale Foxglove panel must not be able to re-subscribe
                    # to multi-megabyte BGR frames and build a send backlog.
                    "^/sony/camera_info$",
                    "^/perception/tracking_image/compressed$",
                    "^/perception/(?:detections|tracks|monitor_status|targets_3d|aim_target_2d)$",
                    "^/diagnostics$",
                    "^/rosout$",
                    "^/cmd_vel$",
                    "^/gimbal/status$",
                    "^/platform/state$",
                    "^/foxglove_bridge/(?:sysinfo|client_count)$",
                ],
                # A ROS graph name/asset URI is never empty, so ^$ is a
                # portable match-nothing expression for all bridge versions.
                "service_whitelist": ["^$"],
                "param_whitelist": ["^$"],
                "client_topic_whitelist": ["^$"],
                "asset_uri_allowlist": ["^$"],
                "capabilities": ["connectionGraph"],
                "best_effort_qos_topic_whitelist": [
                    "^/perception/tracking_image/compressed$",
                ],
                "min_qos_depth": 1,
                # Monitoring is latest-frame-only. Deeper queues convert a
                # temporary Wi-Fi slowdown into seconds of visible latency.
                "max_qos_depth": 1,
                "num_threads": 2,
                "include_hidden": False,
                "tls": False,
                "sysinfo": True,
                "sysinfo_topic": "/foxglove_bridge/sysinfo",
                "sysinfo_refresh_interval": 500,
                "publish_client_count": True,
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_foxglove")),
    )

    return LaunchDescription(arguments + [visualizer, foxglove])
