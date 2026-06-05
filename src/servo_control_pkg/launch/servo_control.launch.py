# servo_control_pkg/launch/servo_control.launch.py
"""
@file servo_control.launch.py
@brief 伺服控制启动文件 — 启动伺服管理节点和速度指令节点。

参数：
  - controller_plugin (string, 默认 "IBVSController"): 控制器插件类名
  - allocation_ratio  (float,  默认 0.5): 控制分配比例 (0=纯云台, 1=纯底盘)
  - control_rate      (float,  默认 50.0): 控制回路频率 (Hz)

节点列表：
  - servo_manager：      主控制回路（加载插件 + 分配 + 发布指令）
  - velocity_commander：速度指令发布器（限幅 + 转发）

用法：
  ros2 launch servo_control_pkg servo_control.launch.py controller_plugin:=servo_control_pkg::PBVSController
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_plugin = LaunchConfiguration("controller_plugin")
    allocation_ratio = LaunchConfiguration("allocation_ratio")
    control_rate = LaunchConfiguration("control_rate")
    auto_start = LaunchConfiguration("auto_start")

    config_dir = PathJoinSubstitution([
        FindPackageShare("servo_control_pkg"), "config"
    ])

    # ── 伺服管理节点（主控制回路） ─────────────────────────────────
    servo_manager = Node(
        package="servo_control_pkg",
        executable="servo_manager_node",
        name="servo_manager",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir, "ibvs_params.yaml"]),
            PathJoinSubstitution([config_dir, "allocator_params.yaml"]),
            {"controller_plugin": controller_plugin,
             "allocation_ratio": allocation_ratio,
             "auto_start": auto_start},
        ],
        remappings=[
            ("/perception/targets_3d", "/perception/targets_3d"),  # 输入：3D 目标位姿
            ("/platform/state", "/platform/state"),                # 输入：平台状态
            ("/camera/camera_info", "/camera/camera_info"),        # 输入：相机内参
            ("/cmd_vel", "/cmd_vel"),                              # 输出：底盘速度指令
            ("/cmd_gimbal", "/cmd_gimbal"),                        # 输出：云台指令
            ("/servo/state", "/servo/state"),                      # 输出：伺服状态
        ],
    )

    # ── 速度指令节点（限幅 + 转发） ────────────────────────────────
    velocity_commander = Node(
        package="servo_control_pkg",
        executable="velocity_commander_node",
        name="velocity_commander",
        output="screen",
        parameters=[{
            "max_linear_velocity": 1.0,       # 最大线速度 (m/s)
            "max_angular_velocity": 2.0,       # 最大角速度 (rad/s)
            "gimbal_rate_limit": 3.14159,      # 云台速度限制 (rad/s)
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("controller_plugin",
                              default_value="servo_control_pkg::IBVSController",
                              description="控制器插件：IBVS, PBVS, MPC, RL"),
        DeclareLaunchArgument("allocation_ratio",
                              default_value="0.5",
                              description="控制分配比例 (0=纯云台, 1=纯底盘)"),
        DeclareLaunchArgument("control_rate",
                              default_value="50.0",
                              description="控制回路频率 (Hz)"),
        DeclareLaunchArgument("auto_start",
                              default_value="false",
                              description="是否在收到目标后自动启动闭环"),
        servo_manager,
        velocity_commander,
    ])
