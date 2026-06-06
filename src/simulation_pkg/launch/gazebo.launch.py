# simulation_pkg/launch/gazebo.launch.py
"""
@file gazebo.launch.py
@brief Gazebo 仿真环境启动文件 — 启动 Gazebo 服务端/客户端并加载机器人模型。

该 launch 文件完成四个核心任务：
  1. 预清理残留 Gazebo 进程（释放 11345 端口）
  2. 启动 gzserver（Gazebo 物理引擎服务端），加载 FCR 世界文件
  3. 可选启动 gzclient（Gazebo GUI 客户端），受 gui 参数控制
  4. 运行 robot_state_publisher + spawn_entity.py 生成机器人模型

@note 需要先安装 gazebo_ros_pkgs 和 xacro 包。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("simulation_pkg")
    world_file = LaunchConfiguration("world")

    # ── Gazebo 服务端 ─────────────────────────────────────────────
    # 用 bash -c 包装：先杀残留再启动，保证端口不冲突。
    # $0 = world_file 路径，由 LaunchConfiguration 在运行时替换
    # libgazebo_ros_init.so:  初始化 ROS API 节点
    # libgazebo_ros_state.so: 提供 /gazebo/set_entity_state 等状态话题
    # libgazebo_ros_factory.so: 提供 spawn_entity 服务
    gazebo_server = ExecuteProcess(
        cmd=["bash", "-c",
             "pkill -9 gzserver gzclient 2>/dev/null || true; "
             "sleep 1; "
             "exec gzserver --verbose "
             "-s libgazebo_ros_init.so "
             "-s libgazebo_ros_state.so "
             "-s libgazebo_ros_factory.so "
             "$0",
             world_file],
        output="screen",
    )

    # ── Gazebo 客户端（GUI） ───────────────────────────────────────
    gazebo_client = ExecuteProcess(
        cmd=["gzclient", "--verbose"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # ── 机器人状态发布者 ──────────────────────────────────────────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command([
                    "xacro ", PathJoinSubstitution([pkg_share, "urdf", "lekiwi_full.urdf.xacro"])
                ]), value_type=str
            ),
            "use_sim_time": True,
        }],
    )

    # ── 生成机器人到 Gazebo 世界 ──────────────────────────────────
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

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution(
            [pkg_share, "worlds", "fcr_world.world"]),
            description="Gazebo 世界文件路径"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="是否启动 Gazebo GUI"),
        gazebo_server, gazebo_client,
        robot_state_pub, spawn_robot,
    ])
