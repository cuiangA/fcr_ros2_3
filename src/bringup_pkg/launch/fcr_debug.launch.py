# bringup_pkg/launch/fcr_debug.launch.py
"""
FCR 调试/开发启动文件。

与生产启动的区别：
  - 所有仿真硬件驱动（use_sim=True），无需真实硬件
  - 每个节点独立启动，便于进程隔离和单独调试
  - 全部输出到屏幕（output="screen"）
  - 可通过 debug_level 参数统一控制日志级别

用法：
  ros2 launch bringup_pkg fcr_debug.launch.py debug_level:=debug
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 日志级别：debug / info / warn / error
    debug_level = LaunchConfiguration("debug_level")
    perception_share = FindPackageShare("perception_pkg")

    # 各子系统节点独立启动，便于隔离调试
    nodes = [
        # ── 机器人平台驱动（仿真模式） ───────────────────────
        Node(package="robot_platform_pkg", executable="chassis_driver_node",
             name="chassis_driver", output="screen",
             parameters=[{"use_sim": True}],
             arguments=["--ros-args", "--log-level", debug_level]),

        Node(package="robot_platform_pkg", executable="gimbal_driver_node",
             name="gimbal_driver", output="screen",
             parameters=[{"use_sim": True}],
             arguments=["--ros-args", "--log-level", debug_level]),

        Node(package="robot_platform_pkg", executable="imu_driver_node",
             name="imu_driver", output="screen",
             parameters=[{"use_sim": True}],
             arguments=["--ros-args", "--log-level", debug_level]),

        Node(package="robot_platform_pkg", executable="platform_manager_node",
             name="platform_manager", output="screen",
             arguments=["--ros-args", "--log-level", debug_level]),

        # ── 感知子系统 ───────────────────────────────────────
        Node(package="perception_pkg", executable="detection_node",
             name="detection_node", output="screen",
             parameters=[
                 PathJoinSubstitution(
                     [perception_share, "config", "detection_params.yaml"]
                 ),
                 {
                     "model_path": PathJoinSubstitution(
                         [perception_share, "models", "yolov8n.onnx"]
                     )
                 },
             ],
             remappings=[
                 ("image", "/sony/image_raw"),
                 ("detections", "/perception/detections"),
                 ("debug_image", "/perception/debug_image"),
             ],
             arguments=["--ros-args", "--log-level", debug_level]),

        Node(package="perception_pkg", executable="tracking_node",
             name="tracking_node", output="screen",
             parameters=[
                 PathJoinSubstitution(
                     [perception_share, "config", "tracking_params.yaml"]
                 )
             ],
             remappings=[
                 ("image", "/sony/image_raw"),
                 ("detections", "/perception/detections"),
                 ("tracks", "/perception/tracks"),
             ],
             arguments=["--ros-args", "--log-level", debug_level]),

        # ── 伺服控制子系统 ───────────────────────────────────
        Node(package="servo_control_pkg", executable="servo_manager_node",
             name="servo_manager", output="screen",
             arguments=["--ros-args", "--log-level", debug_level]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("debug_level", default_value="debug",
                              description="日志级别：debug, info, warn, error"),
        *nodes,
    ])
