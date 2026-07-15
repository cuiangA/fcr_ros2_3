# bringup_pkg/launch/fcr_bringup.launch.py
"""
FCR 生产环境一键启动文件。

启动顺序（分阶段定时启动，确保依赖就绪）：
  1. t=0s:  机器人平台（底盘/云台/IMU/里程计硬件驱动）
  2. t=2s:  感知管线（检测 + 跟踪）—— 等待相机驱动就绪
  3. t=3s:  伺服控制 —— 等待感知输出 + 平台状态反馈就绪
  4. t=4s:  可视化（RViz2 + 远程感知监控/Foxglove，可选）

用法：
  ros2 launch bringup_pkg fcr_bringup.launch.py use_sim:=false use_rviz:=true
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # ── 启动配置参数 ──────────────────────────────────────────
    use_sim = LaunchConfiguration("use_sim")
    controller_plugin = LaunchConfiguration("controller_plugin")
    conf_threshold = LaunchConfiguration("confidence_threshold")
    model_path = LaunchConfiguration("model_path")
    detection_device = LaunchConfiguration("detection_device")
    use_mock_detector = LaunchConfiguration("use_mock_detector")
    enable_detection = LaunchConfiguration("enable_detection")

    # 各包的共享目录，用于引用 launch 文件
    pkg_share = FindPackageShare("bringup_pkg")
    perception_share = FindPackageShare("perception_pkg")
    servo_share = FindPackageShare("servo_control_pkg")
    platform_share = FindPackageShare("robot_platform_pkg")
    sony_share = FindPackageShare("sony_camera_pkg")
    remote_monitor_share = FindPackageShare("remote_monitor_pkg")

    # ── 1. 机器人平台（硬件驱动层） ─────────────────────────
    platform_launch = IncludeLaunchDescription(
        PathJoinSubstitution([platform_share, "launch", "platform.launch.py"]),
        launch_arguments={"use_sim": use_sim}.items(),
    )

    # ── 1b. Sony RGB相机（实机模式）─────────────────────────
    sony_launch = IncludeLaunchDescription(
        PathJoinSubstitution([sony_share, "launch", "sony_camera.launch.py"]),
        launch_arguments={
            "enable_perception": "false",
            "camera_index": LaunchConfiguration("sony_camera_index"),
            "camera_info_url": LaunchConfiguration("sony_camera_info_url"),
            "image_topic": LaunchConfiguration("sony_image_topic"),
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("enable_sony_camera"), "' == 'true' and '",
                use_sim, "' != 'true'",
            ])
        ),
    )

    # Mock和真实检测互斥；跟踪节点在两种模式下都可以运行。
    effective_detection = PythonExpression([
        "'", enable_detection, "' == 'true' and '", use_mock_detector, "' != 'true'"
    ])

    # ── 2. 感知管线 ─────────────────────────────────────────
    perception_launch = IncludeLaunchDescription(
        PathJoinSubstitution([perception_share, "launch", "perception.launch.py"]),
        launch_arguments={
            "model_path": model_path,
            "device": detection_device,
            "confidence_threshold": conf_threshold,
            "enable_detection": effective_detection,
            "enable_tracking": LaunchConfiguration("enable_tracking"),
            "sony_image_topic": LaunchConfiguration("sony_image_topic"),
        }.items(),
    )

    # ── 2b. Mock 检测器（仿真模式下的合成检测结果） ──────────
    mock_detector = Node(
        package="simulation_pkg",
        executable="mock_detector.py",
        name="mock_detector",
        output="screen",
        condition=IfCondition(use_mock_detector),
    )

    # ── 3. 伺服控制 ─────────────────────────────────────────
    servo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([servo_share, "launch", "servo_control.launch.py"]),
        launch_arguments={
            "controller_plugin": controller_plugin,
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_servo")),
    )

    # ── 4. RViz2（可选可视化） ──────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "fcr_system.rviz"])],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # ── 5. 只读远程感知监控 + Foxglove Bridge ──────────────
    remote_monitor_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [remote_monitor_share, "launch", "remote_monitor.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": use_sim,
            "enable_visualizer": "true",
            "enable_foxglove": "true",
            "foxglove_address": LaunchConfiguration("foxglove_address"),
            "foxglove_port": LaunchConfiguration("foxglove_port"),
            "image_topic": LaunchConfiguration("sony_image_topic"),
            "model_path": model_path,
            "inference_backend": detection_device,
            "yolo_model": LaunchConfiguration("yolo_model_name"),
            "enable_future_inputs": LaunchConfiguration(
                "enable_monitor_future_inputs"
            ),
            "remote_publish_rate_hz": LaunchConfiguration(
                "remote_publish_rate_hz"
            ),
            "remote_max_width": LaunchConfiguration("remote_max_width"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
    )

    # 分阶段启动：平台 → 感知 → 控制 → 可视化
    # 使用 TimerAction 确保前置节点已就绪
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="是否启用仿真模式"),
        DeclareLaunchArgument("controller_plugin",
                              default_value="servo_control_pkg::IBVSController",
                              description="视觉伺服控制器插件类名"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.5",
                              description="YOLO 检测置信度阈值"),
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution(
                [perception_share, "models", "yolov8n.onnx"]
            ),
            description="YOLO ONNX 模型路径",
        ),
        DeclareLaunchArgument(
            "detection_device",
            default_value="cpu",
            description="检测推理设备：cpu、cuda_fp16、cuda_fp32 或 tensorrt",
        ),
        DeclareLaunchArgument("enable_detection", default_value="true",
                              description="是否启动真实 YOLO 检测节点"),
        DeclareLaunchArgument("enable_tracking", default_value="true",
                              description="是否启动目标跟踪节点"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="是否启动 RViz2"),
        DeclareLaunchArgument("use_foxglove", default_value="false",
                              description="是否启动远程感知监控与 Foxglove WebSocket 桥接"),
        DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0",
                              description="Foxglove Bridge 监听地址"),
        DeclareLaunchArgument("foxglove_port", default_value="8765",
                              description="Foxglove Bridge WebSocket 端口"),
        DeclareLaunchArgument("yolo_model_name", default_value="yolov8n",
                              description="远程状态面板显示的模型名称"),
        DeclareLaunchArgument(
            "enable_monitor_future_inputs", default_value="false",
            description="预留观察 target_3d/cmd_vel/gimbal_state，不发布控制指令",
        ),
        DeclareLaunchArgument(
            "remote_publish_rate_hz", default_value="10.0",
            description="仅限制Foxglove标注图帧率，不影响检测和跟踪",
        ),
        DeclareLaunchArgument(
            "remote_max_width", default_value="960",
            description="Foxglove标注图最大宽度；0表示保留原始宽度",
        ),
        DeclareLaunchArgument("use_mock_detector", default_value="false",
                              description="是否使用合成检测器（绕过 YOLO）"),
        DeclareLaunchArgument("enable_sony_camera", default_value="true",
                              description="实机模式下是否启动Sony相机"),
        DeclareLaunchArgument("sony_image_topic", default_value="/sony/image_raw",
                              description="2D检测使用的RGB图像话题"),
        DeclareLaunchArgument("sony_camera_index", default_value="1",
                              description="CRSDK枚举得到的一基相机序号"),
        DeclareLaunchArgument(
            "sony_camera_info_url", default_value="",
            description="Sony精确分辨率标定文件，例如file:///home/nvidia/.ros/camera_info/sony.yaml",
        ),
        DeclareLaunchArgument(
            "enable_servo", default_value="false",
            description="融合前保持false；仅在有效3D目标源可用时启动控制闭环",
        ),

        # 阶段 1：平台驱动 (t=0s)
        platform_launch,
        sony_launch,
        mock_detector,

        # 阶段 2：感知管线 (t=2s，等待相机驱动就绪)
        TimerAction(period=2.0, actions=[perception_launch]),

        # 阶段 3：伺服控制 (t=3s，等待感知输出 + 平台状态)
        TimerAction(period=3.0, actions=[servo_launch]),

        # 可视化 (t=4s)
        TimerAction(period=4.0, actions=[rviz_node, remote_monitor_launch]),
    ])
