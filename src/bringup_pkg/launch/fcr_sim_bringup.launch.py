# bringup_pkg/launch/fcr_sim_bringup.launch.py
"""
FCR 仿真环境一键启动文件（Gazebo 3D 渲染 + 完整控制链）。

Gazebo 负责：物理引擎、机器人模型渲染、底盘驱动(planar_move)、IMU、深度相机
ROS2 控制层负责：目标生成、视觉伺服、平台状态聚合

数据流：
  target_sim → /perception/targets_3d → servo_manager → /cmd_vel → Gazebo planar_move 插件驱动机器人移动
                                                        → /cmd_gimbal → gimbal_driver(仿真)

用法：
  ros2 launch bringup_pkg fcr_sim_bringup.launch.py
  ros2 launch bringup_pkg fcr_sim_bringup.launch.py trajectory:=circle speed:=0.3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── 启动参数 ──────────────────────────────────────────────────
    controller_plugin = LaunchConfiguration("controller_plugin")
    allocation_ratio = LaunchConfiguration("allocation_ratio")
    trajectory = LaunchConfiguration("trajectory")
    speed = LaunchConfiguration("speed")
    height = LaunchConfiguration("height")
    radius = LaunchConfiguration("radius")

    sim_share = FindPackageShare("simulation_pkg")
    servo_share = FindPackageShare("servo_control_pkg")
    platform_share = FindPackageShare("robot_platform_pkg")

    servo_config = PathJoinSubstitution([servo_share, "config"])
    platform_config = PathJoinSubstitution([platform_share, "config"])

    # ── 阶段 1（t=0s）：Gazebo + 机器人模型 + RViz ────────────────
    # Gazebo 加载世界、生成机器人模型，URDF 内置的 planar_move / IMU / 深度相机插件自动启动
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([sim_share, "launch", "gazebo.launch.py"]),
    )
    rviz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([sim_share, "launch", "rviz.launch.py"]),
    )

    # ── 阶段 2（t=5s）：FCR 控制链 ────────────────────────────────

    # 仿真相机：发布 /camera/camera_info（TRANSIENT_LOCAL QoS），
    # 使 servo_manager 完成控制器标定初始化。
    # Gazebo 自带相机插件发布的内参话题是 /camera/camera/camera_info，
    # 路径不匹配，所以额外加这个作为标定源。
    camera_sim = Node(
        package="simulation_pkg",
        executable="camera_simulator.py",
        name="camera_simulator",
        output="screen",
    )

    # 目标仿真器：直出 3D 位姿 TargetArray（绕过感知管线）
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

    # 伺服管理器：IBVS 或 PBVS 控制器
    servo_manager = Node(
        package="servo_control_pkg",
        executable="servo_manager_node",
        name="servo_manager",
        output="screen",
        parameters=[
            PathJoinSubstitution([servo_config, "ibvs_params.yaml"]),
            PathJoinSubstitution([servo_config, "allocator_params.yaml"]),
            {"controller_plugin": controller_plugin,
             "allocation_ratio": allocation_ratio,
             "auto_start": True},
        ],
    )

    # 云台驱动（仿真模式）。
    # Gazebo gimbal_yaw_joint / gimbal_pitch_joint 由 libgimbal_controller_plugin.so
    # 驱动（在 URDF 中配置），/cmd_gimbal 可直接驱动 3D 模型中的云台关节。
    gimbal_driver = Node(
        package="robot_platform_pkg",
        executable="gimbal_driver_node",
        name="gimbal_driver",
        output="screen",
        parameters=[PathJoinSubstitution([platform_config, "gimbal_params.yaml"]),
                    {"use_sim": True}],
    )

    # 平台管理：订阅 Gazebo 发布的 /odom 和 /imu/data，
    # 聚合成 /platform/state 供 servo_manager 做控制分配
    platform_mgr = Node(
        package="robot_platform_pkg",
        executable="platform_manager_node",
        name="platform_manager",
        output="screen",
    )

    fcr_nodes = [
        camera_sim, target_sim, servo_manager,
        gimbal_driver, platform_mgr,
    ]

    return LaunchDescription([
        DeclareLaunchArgument("controller_plugin",
                              default_value="servo_control_pkg::PBVSController",
                              description="控制器插件: IBVSController, PBVSController"),
        DeclareLaunchArgument("allocation_ratio", default_value="0.5",
                              description="控制分配比例 (0=纯云台, 1=纯底盘)"),
        DeclareLaunchArgument("trajectory", default_value="circle",
                              description="目标轨迹类型: circle, line, figure8"),
        DeclareLaunchArgument("speed", default_value="0.3",
                              description="轨迹速度"),
        DeclareLaunchArgument("height", default_value="1.0",
                              description="目标高度 (m)"),
        DeclareLaunchArgument("radius", default_value="1.0",
                              description="轨迹半径 (m)"),

        # 阶段 1：Gazebo + RViz
        gazebo_launch,
        rviz_launch,

        # 阶段 2：控制链（延迟 5 秒等待 Gazebo 完全启动）
        TimerAction(period=5.0, actions=fcr_nodes),
    ])
