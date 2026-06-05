# bringup_pkg/launch/gimbal_sim.launch.py
"""
FCR 云台伺服专项仿真启动文件。

仅测试伺服→云台的控制链，不启动 Gazebo、相机、底盘。

启动节点：
  - target_simulator     合成 3D 目标位姿（直出 TargetArray）
  - servo_manager        加载 PBVS 控制器，allocation_ratio=0.0（纯云台）
  - gimbal_driver        仿真云台驱动（use_sim=true）
  - platform_manager     平台状态聚合
  - imu_driver           仿真 IMU 驱动

数据流：
  target_sim → /perception/targets_3d → servo_manager → /cmd_gimbal → gimbal_driver

用法：
  ros2 launch bringup_pkg gimbal_sim.launch.py trajectory:=circle speed:=0.5 use_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # ── 启动参数 ──────────────────────────────────────────────────
    trajectory = LaunchConfiguration("trajectory")
    speed = LaunchConfiguration("speed")
    height = LaunchConfiguration("height")
    radius = LaunchConfiguration("radius")
    use_rviz = LaunchConfiguration("use_rviz")

    sim_share = FindPackageShare("simulation_pkg")
    servo_share = FindPackageShare("servo_control_pkg")
    platform_share = FindPackageShare("robot_platform_pkg")
    bringup_share = FindPackageShare("bringup_pkg")

    servo_config = PathJoinSubstitution([servo_share, "config"])
    platform_config = PathJoinSubstitution([platform_share, "config"])

    # ── 1. 目标仿真器（直出 3D 位姿 TargetArray） ─────────────────
    target_sim = Node(
        package="simulation_pkg",
        executable="target_simulator.py",
        name="target_simulator",
        output="screen",
        parameters=[{
            "publish_target_array": True,
            "trajectory": trajectory,
            "speed": speed,
            "height": height,
            "radius": radius,
        }],
    )

    # ── 2. 伺服管理节点（纯云台模式，PBVS 控制器） ───────────────
    servo_manager = Node(
        package="servo_control_pkg",
        executable="servo_manager_node",
        name="servo_manager",
        output="screen",
        parameters=[
            PathJoinSubstitution([servo_config, "ibvs_params.yaml"]),
            PathJoinSubstitution([servo_config, "allocator_params.yaml"]),
            {"controller_plugin": "servo_control_pkg::PBVSController",
             "allocation_ratio": 0.0,
             "auto_start": True},
        ],
    )

    # ── 3. 云台驱动（仿真模式） ───────────────────────────────────
    gimbal_driver = Node(
        package="robot_platform_pkg",
        executable="gimbal_driver_node",
        name="gimbal_driver",
        output="screen",
        parameters=[PathJoinSubstitution([platform_config, "gimbal_params.yaml"]),
                    {"use_sim": True}],
    )

    # ── 4. 平台管理节点（状态聚合） ──────────────────────────────
    platform_mgr = Node(
        package="robot_platform_pkg",
        executable="platform_manager_node",
        name="platform_manager",
        output="screen",
    )

    # ── 5. IMU 驱动（仿真模式，提供 /imu/data） ──────────────────
    imu_driver = Node(
        package="robot_platform_pkg",
        executable="imu_driver_node",
        name="imu_driver",
        output="screen",
        parameters=[PathJoinSubstitution([platform_config, "imu_params.yaml"]),
                    {"use_sim": True}],
    )

    # ── 6. RViz（可选） ───────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([bringup_share, "rviz", "fcr_system.rviz"])],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("trajectory", default_value="circle",
                              description="目标轨迹类型: circle, line, figure8"),
        DeclareLaunchArgument("speed", default_value="0.5",
                              description="轨迹速度"),
        DeclareLaunchArgument("height", default_value="1.0",
                              description="目标高度 (m)"),
        DeclareLaunchArgument("radius", default_value="1.0",
                              description="轨迹半径 (m)"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="是否启动 RViz2"),

        target_sim, servo_manager, gimbal_driver, platform_mgr,
        imu_driver, rviz,
    ])
