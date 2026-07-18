# bringup_pkg/launch/chassis_sim.launch.py
"""
FCR 底盘伺服专项仿真启动文件。

仅测试伺服→底盘的完整控制链，不启动 Gazebo、相机、云台。

启动节点：
  - target_simulator     合成 3D 目标位姿（直出 TargetArray）
  - servo_manager        加载 PBVS 控制器，allocation_ratio=1.0（纯底盘）
  - chassis_driver       仿真底盘驱动（use_sim=true）
  - odometry             轮速+IMU 里程计
  - platform_manager     平台状态聚合

数据流：
  target_sim → /perception/targets_3d → servo_manager → /cmd_vel → chassis_driver → /odom
                                                                              ↓
                                                           platform_manager → /platform/state → servo_manager

用法：
  ros2 launch bringup_pkg chassis_sim.launch.py trajectory:=circle speed:=0.3 use_rviz:=true
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

    # ── 1. 仿真相机（发布 CameraInfo，完成伺服管理器标定初始化） ──
    camera_sim = Node(
        package="simulation_pkg",
        executable="camera_simulator.py",
        name="camera_simulator",
        output="screen",
    )

    # ── 2. 目标仿真器（直出 3D 位姿 TargetArray） ─────────────────
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

    # ── 3. 伺服管理节点（纯底盘模式） ─────────────────────────────
    servo_manager = Node(
        package="servo_control_pkg",
        executable="servo_manager_node",
        name="servo_manager",
        output="screen",
        parameters=[
            PathJoinSubstitution([servo_config, "pbvs_params.yaml"]),
            PathJoinSubstitution([servo_config, "allocator_params.yaml"]),
            {"controller_plugin": "servo_control_pkg::PBVSController",
             "allocation_ratio": 1.0,
             "auto_start": True,
             "allow_chassis_translation": True,
             "publish_unstamped_cmd_vel": True},
        ],
    )

    # ── 4. 底盘驱动（仿真模式） ───────────────────────────────────
    chassis_driver = Node(
        package="robot_platform_pkg",
        executable="chassis_driver_node",
        name="chassis_driver",
        output="screen",
        parameters=[PathJoinSubstitution([platform_config, "chassis_params.yaml"]),
                    {"use_sim": True}],
    )

    # ── 4. 里程计节点 ─────────────────────────────────────────────
    odometry = Node(
        package="robot_platform_pkg",
        executable="odometry_node",
        name="odometry",
        output="screen",
        parameters=[PathJoinSubstitution([platform_config, "odometry_params.yaml"])],
    )

    # ── 5. 平台管理节点（状态聚合） ──────────────────────────────
    platform_mgr = Node(
        package="robot_platform_pkg",
        executable="platform_manager_node",
        name="platform_manager",
        output="screen",
    )

    # ── 6. IMU 驱动（仿真模式，提供 /imu/data 给里程计） ────────
    imu_driver = Node(
        package="robot_platform_pkg",
        executable="imu_driver_node",
        name="imu_driver",
        output="screen",
        parameters=[PathJoinSubstitution([platform_config, "imu_params.yaml"]),
                    {"use_sim": True}],
    )

    # ── 7. RViz（可选） ───────────────────────────────────────────
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
        DeclareLaunchArgument("speed", default_value="0.3",
                              description="轨迹速度"),
        DeclareLaunchArgument("height", default_value="1.0",
                              description="目标高度 (m)"),
        DeclareLaunchArgument("radius", default_value="1.0",
                              description="轨迹半径 (m)"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="是否启动 RViz2"),

        camera_sim, target_sim, servo_manager, chassis_driver, odometry,
        imu_driver, platform_mgr, rviz,
    ])
