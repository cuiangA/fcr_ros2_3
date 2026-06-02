# simulation_pkg/launch/gazebo.launch.py
"""
@file gazebo.launch.py
@brief Gazebo 仿真环境启动文件 — 启动 Gazebo 服务端/客户端并加载机器人模型。

该 launch 文件完成四个核心任务：
  1. 启动 gzserver（Gazebo 物理引擎服务端），加载 FCR 世界文件
  2. 可选启动 gzclient（Gazebo GUI 客户端），受 gui 参数控制
  3. 运行 robot_state_publisher，从 xacro 生成机器人 URDF 并广播 TF
  4. 通过 spawn_entity.py 将机器人模型生成到 Gazebo 世界中

适用于：完整的 Gazebo 仿真环境一键启动，包含物理引擎、渲染和机器人模型。

@note 需要先安装 gazebo_ros_pkgs 和 xacro 包。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare("simulation_pkg")
    world_file = LaunchConfiguration("world")

    # ── Gazebo 服务端 ─────────────────────────────────────────────
    # gzserver 是 Gazebo 的无头(headless)模式，负责物理运算和传感器仿真
    gazebo_server = ExecuteProcess(
        cmd=["gzserver", "--verbose", "-s", "libgazebo_ros_factory.so", world_file],
        output="screen",
    )

    # ── Gazebo 客户端（GUI） ───────────────────────────────────────
    # 仅在 gui 参数为 true 时启动，服务器模式下可节省 GPU 资源
    gazebo_client = ExecuteProcess(
        cmd=["gzclient", "--verbose"],
        output="screen",
        condition=LaunchConfiguration("gui"),
    )

    # ── 机器人状态发布者 ──────────────────────────────────────────
    # robot_state_publisher 负责将 URDF 中的关节状态转换为 TF 变换树
    # use_sim_time=True 确保所有节点使用 Gazebo 的仿真时间而非系统时间
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": Command([
                "xacro ", PathJoinSubstitution([pkg_share, "urdf", "lekiwi_full.urdf.xacro"])
            ]),
            "use_sim_time": True,
        }],
    )

    # ── 生成机器人到 Gazebo 世界 ──────────────────────────────────
    # 使用 /robot_description 话题获取机器人 URDF，将其放置在原点上方 0.15m
    # 抬高避免模型在生成瞬间与地面碰撞导致物理爆炸
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_robot",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "lekiwi_fcr",
            "-x", "0", "-y", "0", "-z", "0.15",
        ],
    )

    # ── 组装 LaunchDescription ────────────────────────────────────
    # 声明参数放在列表最前，确保用户可以在命令行覆盖默认值
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution(
            [pkg_share, "worlds", "fcr_world.world"]),
            description="Gazebo 世界文件路径"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="是否启动 Gazebo GUI"),
        gazebo_server, gazebo_client,
        robot_state_pub, spawn_robot,
    ])
