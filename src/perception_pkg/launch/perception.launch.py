# perception_pkg/launch/perception.launch.py
"""
@file perception.launch.py
@brief 感知管线启动文件 — 支持独立节点和可组合管线两种部署模式。

两种模式通过 use_composition 参数切换：
  - use_composition=false：启动三个独立进程（检测 + 跟踪 + 深度估计）
  - use_composition=true： 启动一个 ComposableNodeContainer，三阶段零拷贝运行

独立模式适合调试（每个节点可单独查看日志），可组合模式适合部署（零拷贝 + 低延迟）。

参数：
  - use_composition (bool, 默认 true):  是否使用可组合管线
  - confidence_threshold (float, 默认 0.3): YOLO 检测置信度阈值
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # ── 启动配置参数 ──────────────────────────────────────────────
    use_composition = LaunchConfiguration("use_composition")
    conf_threshold = LaunchConfiguration("confidence_threshold")

    # 配置文件目录（所有节点共享同一 config/ 目录）
    config_dir = PathJoinSubstitution([
        FindPackageShare("perception_pkg"), "config"
    ])

    # ── 独立节点模式（use_composition=false） ────────────────────
    detection_node = Node(
        package="perception_pkg",
        executable="detection_node",
        name="detection",
        parameters=[PathJoinSubstitution([config_dir, "detection_params.yaml"]),
                    {"confidence_threshold": conf_threshold}],
        remappings=[
            ("~/image_raw", "/camera/image_raw"),       # 输入：来自相机驱动
            ("~/detections", "/perception/detections"), # 输出：2D 检测框
        ],
        condition=UnlessCondition(use_composition),
    )

    tracking_node = Node(
        package="perception_pkg",
        executable="tracking_node",
        name="tracking",
        parameters=[PathJoinSubstitution([config_dir, "tracking_params.yaml"])],
        remappings=[
            ("detections", "/perception/detections"),   # 输入：来自检测节点
            ("~/tracks", "/perception/tracks"),         # 输出：带 ID 的跟踪轨迹
        ],
        condition=UnlessCondition(use_composition),
    )

    depth_node = Node(
        package="perception_pkg",
        executable="depth_estimator_node",
        name="depth_estimator",
        parameters=[PathJoinSubstitution([config_dir, "depth_params.yaml"])],
        remappings=[
            ("detections", "/perception/detections"),        # 输入：检测结果
            ("depth/image_raw", "/camera/depth/image_raw"),  # 输入：深度图
            ("~/targets_3d", "/perception/targets_3d"),      # 输出：3D 目标位姿
        ],
        condition=UnlessCondition(use_composition),
    )

    # ── 可组合管线模式（use_composition=true） ───────────────────
    # 三阶段全部在同一进程内运行，图像数据零拷贝传递
    container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # 多线程执行器，并行处理
        composable_node_descriptions=[
            ComposableNode(
                package="perception_pkg",
                plugin="perception_pkg::PerceptionPipeline",
                name="perception_pipeline",
                parameters=[PathJoinSubstitution([config_dir, "detection_params.yaml"]),
                            PathJoinSubstitution([config_dir, "tracking_params.yaml"])],
                remappings=[
                    ("image_raw", "/camera/image_raw"),
                    ("depth/image_raw", "/camera/depth/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                    ("~/detections", "/perception/detections"),
                    ("~/tracks", "/perception/tracks"),
                    ("~/targets_3d", "/perception/targets_3d"),
                ],
            ),
        ],
        condition=IfCondition(use_composition),
    )

    # ── 组装 LaunchDescription ────────────────────────────────────
    return LaunchDescription([
        DeclareLaunchArgument("use_composition", default_value="true",
                              description="是否使用进程内可组合节点（零拷贝）"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.3",
                              description="YOLO 检测置信度阈值"),
        detection_node, tracking_node, depth_node, container,
    ])
