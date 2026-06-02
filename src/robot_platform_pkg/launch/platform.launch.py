# robot_platform_pkg/launch/platform.launch.py
"""
@file platform.launch.py
@brief 机器人平台硬件驱动启动文件 — 启动底盘/云台/IMU/里程计/平台管理节点。

所有驱动节点共享 use_sim 参数：
  - use_sim=false（默认）：连接真实硬件（串口/CAN/I2C）
  - use_sim=true：         使用仿真后端，无需硬件

节点列表：
  - chassis_driver  — LEKIWI 三全向轮底盘驱动
  - gimbal_driver   — DJI RS2 云台驱动（CAN）
  - imu_driver      — BNO055 IMU 驱动（I2C）
  - odometry        — 轮速 + IMU 融合里程计
  - platform_manager — 聚合所有平台状态为统一的 PlatformState 消息

用法：
  ros2 launch robot_platform_pkg platform.launch.py use_sim:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")

    # 配置文件目录
    config_dir = PathJoinSubstitution([
        FindPackageShare("robot_platform_pkg"), "config"
    ])

    # ── 底盘驱动节点 ──────────────────────────────────────────────
    chassis_node = Node(
        package="robot_platform_pkg",
        executable="chassis_driver_node",
        name="chassis_driver",
        output="screen",
        parameters=[PathJoinSubstitution([config_dir, "chassis_params.yaml"]),
                    {"use_sim": use_sim}],
    )

    # ── 云台驱动节点 ──────────────────────────────────────────────
    gimbal_node = Node(
        package="robot_platform_pkg",
        executable="gimbal_driver_node",
        name="gimbal_driver",
        output="screen",
        parameters=[PathJoinSubstitution([config_dir, "gimbal_params.yaml"]),
                    {"use_sim": use_sim}],
    )

    # ── IMU 驱动节点 ──────────────────────────────────────────────
    imu_node = Node(
        package="robot_platform_pkg",
        executable="imu_driver_node",
        name="imu_driver",
        output="screen",
        parameters=[PathJoinSubstitution([config_dir, "imu_params.yaml"]),
                    {"use_sim": use_sim}],
    )

    # ── 里程计节点 ────────────────────────────────────────────────
    odom_node = Node(
        package="robot_platform_pkg",
        executable="odometry_node",
        name="odometry",
        output="screen",
        parameters=[PathJoinSubstitution([config_dir, "odometry_params.yaml"])],
    )

    # ── 平台管理节点（状态聚合） ──────────────────────────────────
    platform_mgr = Node(
        package="robot_platform_pkg",
        executable="platform_manager_node",
        name="platform_manager",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="是否使用仿真硬件驱动"),
        chassis_node, gimbal_node, imu_node, odom_node, platform_mgr,
    ])
