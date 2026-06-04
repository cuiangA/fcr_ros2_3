# simulation_pkg/launch/spawn_robot.launch.py
"""
@file spawn_robot.launch.py
@brief 机器人模型发布与关节状态启动文件 — 发布机器人 URDF 和关节状态。

该 launch 文件负责两项基础任务：
  1. 运行 robot_state_publisher，将 xacro 展开为 URDF 并通过 /robot_description 发布
  2. 运行 joint_state_publisher，维护并发布所有非固定关节的当前状态

这两项是启动 Gazebo 仿真或 RViz 可视化之前的必要前置步骤。
单独封装为 launch 文件的好处：可在仿真和真机之间复用同一套模型加载逻辑。

参数：
  - use_sim_time (bool, 默认 true):  是否使用仿真时间（/clock 话题）

@note 依赖 simulation_pkg 中的 urdf/lekiwi_full.urdf.xacro 文件。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("simulation_pkg")

    # ── 机器人状态发布者 ──────────────────────────────────────────
    # 使用 xacro 工具将 .xacro 宏文件展开为标准 URDF XML
    # use_sim_time 通过 LaunchConfiguration 外部传入，便于仿真/真机切换
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
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    # ── 关节状态发布者 ────────────────────────────────────────────
    # 在无物理仿真环境下提供默认关节状态（例如所有关节归零）
    # 在 Gazebo 中会被 /joint_states 控制器的输出覆盖
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # ── 组装 LaunchDescription ────────────────────────────────────
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="是否使用仿真时间（/clock 话题）"),
        robot_state_pub, joint_state_pub,
    ])
