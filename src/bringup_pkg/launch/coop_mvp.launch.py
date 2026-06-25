# bringup_pkg/launch/coop_mvp.launch.py
"""
FCR 协同控制仿真 — MVP 控制后端。

数据流:
  coop_sim(感知) → /target/current → mvp_follow_controller_node(三通道解耦 P + IBVS)
                 → /platform/state
                                       ↓
                         /cmd_vel + /cmd_gimbal
                                       ↓
                           coop_sim(执行) → /odom + /platform/state + /tf

与 servo_manager (IBVS/PBVS) 路径的对比:
  MVP:  三通道解耦 P 控制 + 云台可选 IBVS + 硬编码串级
  servo: 完整 6-DOF IBVS/PBVS + ControlAllocator PRIORITY 分配

用法:
  ros2 launch bringup_pkg coop_mvp.launch.py
  ros2 launch bringup_pkg coop_mvp.launch.py target_motion:=circle desired_distance:=3.0
  ros2 launch bringup_pkg coop_mvp.launch.py use_ibvs_gimbal:=false  # 纯 P 模式
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    target_motion = LaunchConfiguration("target_motion")
    target_distance = LaunchConfiguration("target_distance")
    target_angle_deg = LaunchConfiguration("target_angle_deg")
    target_radius = LaunchConfiguration("target_radius")
    target_speed = LaunchConfiguration("target_speed")
    desired_distance = LaunchConfiguration("desired_distance")
    initial_robot_yaw_deg = LaunchConfiguration("initial_robot_yaw_deg")
    initial_gimbal_yaw_deg = LaunchConfiguration("initial_gimbal_yaw_deg")

    # MVP 控制器参数
    use_ibvs_gimbal = LaunchConfiguration("use_ibvs_gimbal")
    K_gimbal_x = LaunchConfiguration("K_gimbal_x")
    K_base_yaw = LaunchConfiguration("K_base_yaw")
    K_base_z = LaunchConfiguration("K_base_z")
    foxglove_port = LaunchConfiguration("foxglove_port")

    def f(v):
        return ParameterValue(v, value_type=float)
    def b(v):
        return ParameterValue(v, value_type=bool)

    # ── 1. coop_sim: 感知 + 执行（target 发到 /target/current） ──
    coop_sim = Node(
        package="simulation_pkg",
        executable="coop_sim.py",
        name="coop_sim",
        output="screen",
        parameters=[{
            "target_topic": "/target/current",
            "target_motion": target_motion,
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

    # ── 2. mvp_follow_controller_node: 三通道解耦 P + IBVS ──────
    mvp_controller = Node(
        package="servo_control_pkg",
        executable="mvp_follow_controller_node",
        name="mvp_follow_controller_node",
        output="screen",
        parameters=[{
            "target_topic": "/target/current",
            "platform_state_topic": "/platform/state",
            "cmd_vel_topic": "/cmd_vel",
            "cmd_gimbal_topic": "/cmd_gimbal",
            "use_twist_stamped": True,
            "use_mock_gimbal_state": False,          # 从 /platform/state 读真实 gimbal 角度
            "use_ibvs_gimbal": b(use_ibvs_gimbal),
            "desired_distance": f(desired_distance),
            "K_gimbal_x": f(K_gimbal_x),
            "K_gimbal_y": 0.6,
            "K_base_z": f(K_base_z),
            "K_base_yaw": f(K_base_yaw),
            "control_rate_hz": 50.0,
            "max_gimbal_yaw_vel": 3.14,
            "max_gimbal_pitch_vel": 1.57,
            "max_base_vx": 1.0,
            "max_base_wz": 1.57,
            "q_yaw_deadband": 0.0873,
            "depth_deadband": 0.05,
            "target_timeout": 1.0,
        }],
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
        DeclareLaunchArgument("target_motion", default_value="static",
                              description="目标轨迹: static | circle | line | figure8"),
        DeclareLaunchArgument("target_distance", default_value="3.0",
                              description="目标初始距离 (m)"),
        DeclareLaunchArgument("target_angle_deg", default_value="20.0",
                              description="目标初始方位角 (°)"),
        DeclareLaunchArgument("target_radius", default_value="1.0",
                              description="circle/figure8 轨迹半径 (m)"),
        DeclareLaunchArgument("target_speed", default_value="0.3",
                              description="轨迹角速度"),
        DeclareLaunchArgument("desired_distance", default_value="2.0",
                              description="期望跟拍距离 (m)"),
        DeclareLaunchArgument("initial_robot_yaw_deg", default_value="0.0",
                              description="机器人初始偏航角 (°)"),
        DeclareLaunchArgument("initial_gimbal_yaw_deg", default_value="0.0",
                              description="云台初始偏航角 (°)"),
        DeclareLaunchArgument("use_ibvs_gimbal", default_value="true",
                              description="云台用 IBVS (true) 还是纯 P (false)"),
        DeclareLaunchArgument("K_gimbal_x", default_value="0.8",
                              description="云台偏航 P 增益"),
        DeclareLaunchArgument("K_base_yaw", default_value="0.5",
                              description="底盘偏航 P 增益 (追云台偏角)"),
        DeclareLaunchArgument("K_base_z", default_value="0.4",
                              description="底盘前向 P 增益 (追深度)"),
        DeclareLaunchArgument("foxglove_port", default_value="8766",
                              description="Foxglove WebSocket 端口"),

        coop_sim,
        mvp_controller,
        foxglove_bridge,
    ])
