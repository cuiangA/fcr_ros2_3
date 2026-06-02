# bringup_pkg/launch/fcr_sim_bringup.launch.py
"""
FCR 仿真环境一键启动文件。

启动顺序：
  1. t=0s:  Gazebo 仿真器 + 机器人模型生成 + RViz 可视化
  2. t=5s:  完整 FCR 系统（仿真模式）—— 等待 Gazebo 完全启动

用法：
  ros2 launch bringup_pkg fcr_sim_bringup.launch.py controller_plugin:=servo_control_pkg::PBVSController
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = FindPackageShare("simulation_pkg")
    bringup_share = FindPackageShare("bringup_pkg")

    # ── 1. Gazebo + 生成机器人模型 ──────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([sim_share, "launch", "gazebo.launch.py"]),
    )

    # ── 2. RViz 可视化 ──────────────────────────────────────
    rviz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([sim_share, "launch", "rviz.launch.py"]),
    )

    # ── 3. 完整 FCR 系统（仿真模式，use_sim=true） ─────────
    # 复用生产启动文件，通过参数切换到仿真模式
    fcr_launch = IncludeLaunchDescription(
        PathJoinSubstitution([bringup_share, "launch", "fcr_bringup.launch.py"]),
        launch_arguments={
            "use_sim": "true",
            "use_composition": "true",
            "use_rviz": "false",   # 已单独启动 RViz，避免重复
            "use_foxglove": "true",
            "controller_plugin": LaunchConfiguration("controller_plugin"),
            "confidence_threshold": LaunchConfiguration("confidence_threshold"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("controller_plugin",
                              default_value="servo_control_pkg::IBVSController",
                              description="视觉伺服控制器插件类名"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.5",
                              description="YOLO 检测置信度阈值"),

        # 阶段 1：Gazebo + 机器人生成 + RViz
        gazebo_launch,
        rviz_launch,

        # 阶段 2：FCR 系统（延迟 5s 等待 Gazebo 完全启动）
        TimerAction(period=5.0, actions=[fcr_launch]),
    ])
