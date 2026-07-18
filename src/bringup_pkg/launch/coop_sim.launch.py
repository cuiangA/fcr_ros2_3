# bringup_pkg/launch/coop_sim.launch.py
"""
FCR 协同控制仿真 — 真实伺服链路闭环。

数据流:
  coop_sim(感知) → /perception/targets_3d → servo_manager(IBVS/PBVS + ControlAllocator)
                 → /camera/camera_info
                                              ↓
                              /cmd_vel + /cmd_gimbal
                                              ↓
                                  coop_sim(执行) → /odom + /platform/state + /tf

测试参数:
  allocation_ratio : 0=纯云台, 0.5=协同, 1=纯底盘
  desired_depth    : 期望跟拍距离
  target_motion    : static | circle | line | figure8

用法:
  ros2 launch bringup_pkg coop_sim.launch.py
  ros2 launch bringup_pkg coop_sim.launch.py allocation_ratio:=0.0   # 纯云台
  ros2 launch bringup_pkg coop_sim.launch.py allocation_ratio:=1.0   # 纯底盘
  ros2 launch bringup_pkg coop_sim.launch.py target_motion:=circle target_speed:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── 启动参数 ──────────────────────────────────────────────────
    controller_plugin = LaunchConfiguration("controller_plugin")
    allocation_ratio = LaunchConfiguration("allocation_ratio")
    desired_depth = LaunchConfiguration("desired_depth")
    target_distance = LaunchConfiguration("target_distance")
    target_angle_deg = LaunchConfiguration("target_angle_deg")
    target_radius = LaunchConfiguration("target_radius")
    target_speed = LaunchConfiguration("target_speed")
    initial_robot_yaw_deg = LaunchConfiguration("initial_robot_yaw_deg")
    initial_gimbal_yaw_deg = LaunchConfiguration("initial_gimbal_yaw_deg")
    foxglove_port = LaunchConfiguration("foxglove_port")

    def f(v):
        return ParameterValue(v, value_type=float)
    def b(v):
        return ParameterValue(v, value_type=bool)

    sim_share = FindPackageShare("simulation_pkg")
    servo_share = FindPackageShare("servo_control_pkg")

    # ── 1. coop_sim: 感知 + 执行仿真 ─────────────────────────────
    coop_sim = Node(
        package="simulation_pkg",
        executable="coop_sim.py",
        name="coop_sim",
        output="screen",
        parameters=[{
            "target_topic": "/perception/targets_3d",
            "target_distance": f(target_distance),
            "target_angle_deg": f(target_angle_deg),
            "target_radius": f(target_radius),
            "target_speed": f(target_speed),
            "initial_robot_yaw_deg": f(initial_robot_yaw_deg),
            "initial_gimbal_yaw_deg": f(initial_gimbal_yaw_deg),
            "publish_markers": True,
            "publish_tf": True,
        }],
    )

    # ── 2. servo_manager: 真实 C++ 伺服控制 ─────────────────────
    servo_manager = Node(
        package="servo_control_pkg",
        executable="servo_manager_node",
        name="servo_manager",
        output="screen",
        parameters=[
            PathJoinSubstitution([servo_share, "config", "ibvs_params.yaml"]),
            PathJoinSubstitution([servo_share, "config", "allocator_params.yaml"]),
            {
                "controller_plugin": controller_plugin,
                "allocation_ratio": f(allocation_ratio),
                "desired_depth": f(desired_depth),
                "auto_start": True,
                "allow_chassis_translation": True,
                "publish_unstamped_cmd_vel": False,
                "use_sim_time": False,
            },
        ],
    )

    # ── 3. Foxglove WebSocket ────────────────────────────────────
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": ParameterValue(foxglove_port, value_type=int),
            "max_qos_depth": 10,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("controller_plugin",
                              default_value="servo_control_pkg::IBVSController",
                              description="伺服控制器: IBVSController | PBVSController"),
        DeclareLaunchArgument("allocation_ratio", default_value="0.5",
                              description="控制分配: 0=纯云台, 0.5=协同, 1=纯底盘"),
        DeclareLaunchArgument("desired_depth", default_value="2.0",
                              description="期望跟拍距离 (m)"),
        DeclareLaunchArgument("target_motion", default_value="figure8",
                              description="目标轨迹: static | circle | line | figure8"),
        DeclareLaunchArgument("target_distance", default_value="3.0",
                              description="目标初始距离 (m)"),
        DeclareLaunchArgument("target_angle_deg", default_value="20.0",
                              description="目标初始方位角 (°)"),
        DeclareLaunchArgument("target_radius", default_value="1.0",
                              description="circle/figure8 轨迹半径 (m)"),
        DeclareLaunchArgument("target_speed", default_value="0.3",
                              description="轨迹角速度"),
        DeclareLaunchArgument("initial_robot_yaw_deg", default_value="0.0",
                              description="机器人初始偏航角 (°)"),
        DeclareLaunchArgument("initial_gimbal_yaw_deg", default_value="0.0",
                              description="云台初始偏航角 (°)"),
        DeclareLaunchArgument("foxglove_port", default_value="8766",
                              description="Foxglove WebSocket 端口"),

        # 先启动 coop_sim (提供 CameraInfo + 执行反馈)，
        # 延时 1.5s 再启动 servo_manager (等待 CameraInfo TRANSIENT_LOCAL 就绪)
        coop_sim,
        TimerAction(period=1.5, actions=[servo_manager]),
        foxglove_bridge,
    ])
