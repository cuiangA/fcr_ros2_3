# simulation_pkg/launch/voice_gimbal_test.launch.py
"""
一键启动语音→云台仿真验证链路。

启动节点:
  1. gimbal_driver      — 仿真云台 (SimulatedGimbal, use_sim:=true)
  2. voice_gimbal_nudge — 语音意图→GimbalCmd 桥接

不启动 wake_up_node,跳过麦克风/sherpa-onnx/云ASR依赖。

用法:
  # 基本用法 (手动发 VoiceCommand 验证)
  ros2 launch simulation_pkg voice_gimbal_test.launch.py

  # 启动后自动发送测试命令
  ros2 launch simulation_pkg voice_gimbal_test.launch.py auto_test:=true

  # 验证命令:
  ros2 topic pub --once /external/voice_command external_control_pkg/msg/VoiceCommand \
    "{intents: ['gimbal_nudge_right'], confidences: [1.0], raw_text: '向右一点', \
      distance: -1.0, unit: '', speed: '', target_desc: '', follow: false}"
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, ExecuteProcess, RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_test = LaunchConfiguration("auto_test")

    gimbal = Node(
        package="robot_platform_pkg",
        executable="gimbal_driver_node",
        name="gimbal_driver",
        output="screen",
        parameters=[{"use_sim": True}],
    )

    nudge = Node(
        package="external_control_pkg",
        executable="voice_gimbal_nudge_node",
        name="voice_gimbal_nudge_node",
        output="screen",
        parameters=[{"right_yaw_sign": -1.0}],
    )

    # 可选: 等节点就绪后自动发一次测试命令
    auto_test_cmd = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "--once",
                    "/external/voice_command",
                    "external_control_pkg/msg/VoiceCommand",
                    "{intents: ['gimbal_nudge_right'], confidences: [1.0],"
                    " raw_text: '向右一点', distance: -1.0,"
                    " unit: '', speed: '', target_desc: '', follow: false}"
                ],
                name="auto_test_publisher",
                output="screen",
            )
        ],
        condition=IfCondition(auto_test),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "auto_test", default_value="false",
            description="启动 2 秒后自动发送 gimbal_nudge_right 测试指令",
        ),
        gimbal,
        nudge,
        auto_test_cmd,
    ])
