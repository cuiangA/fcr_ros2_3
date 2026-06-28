# external_control_pkg/launch/voice_control.launch.py
"""
@file voice_control.launch.py
@brief 语音控制启动文件 — 启动 C++ 唤醒节点（可与其他节点组合）。

用法：
  ros2 launch external_control_pkg voice_control.launch.py
  ros2 launch external_control_pkg voice_control.launch.py cloud_asr_url:=http://192.168.1.100:8080/asr
  ros2 launch external_control_pkg voice_control.launch.py model_dir:=/path/to/model
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cloud_asr_url = LaunchConfiguration("cloud_asr_url")
    cmd_topic = LaunchConfiguration("cmd_topic")
    model_dir = LaunchConfiguration("model_dir")
    energy_threshold = LaunchConfiguration("energy_threshold")
    silence_timeout = LaunchConfiguration("silence_timeout")
    mic_device = LaunchConfiguration("mic_device")
    start_wake_up_node = LaunchConfiguration("start_wake_up_node")
    cmd_gimbal_topic = LaunchConfiguration("cmd_gimbal_topic")
    nudge_yaw_rate = LaunchConfiguration("nudge_yaw_rate")
    nudge_duration = LaunchConfiguration("nudge_duration")
    right_yaw_sign = LaunchConfiguration("right_yaw_sign")
    min_confidence = LaunchConfiguration("min_confidence")

    nudge_config = PathJoinSubstitution([
        FindPackageShare("external_control_pkg"),
        "config",
        "voice_gimbal_nudge.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "cloud_asr_url",
            default_value="http://localhost:8080/asr",
            description="云端 ASR 服务地址（HTTP POST multipart/form-data）",
        ),
        DeclareLaunchArgument(
            "cmd_topic",
            default_value="/external/voice_command",
            description="发布的指令消息话题",
        ),
        DeclareLaunchArgument(
            "model_dir",
            default_value="",
            description="唤醒词模型目录（含 encoder.onnx / decoder.onnx / joiner.onnx / tokens.txt）",
        ),
        DeclareLaunchArgument(
            "energy_threshold",
            default_value="500.0",
            description="麦克风能量阈值（低于此值视为静音）",
        ),
        DeclareLaunchArgument(
            "silence_timeout",
            default_value="5.0",
            description="静音超时判定时间（秒）",
        ),
        DeclareLaunchArgument(
            "mic_device",
            default_value="-1",
            description="麦克风设备索引（-1 = 系统默认）",
        ),
        DeclareLaunchArgument(
            "start_wake_up_node",
            default_value="true",
            description="是否启动 wake_up_node；依赖未安装时可设为 false，仅验证语音指令桥接",
        ),
        DeclareLaunchArgument(
            "cmd_gimbal_topic",
            default_value="/cmd_gimbal",
            description="云台控制输出话题",
        ),
        DeclareLaunchArgument(
            "nudge_yaw_rate",
            default_value="0.25",
            description="语音云台短动作 yaw 角速度（rad/s）",
        ),
        DeclareLaunchArgument(
            "nudge_duration",
            default_value="0.4",
            description="语音云台短动作持续时间（秒）",
        ),
        DeclareLaunchArgument(
            "right_yaw_sign",
            default_value="-1.0",
            description="向右一点对应的 yaw 方向符号，实机方向反了改为 1.0",
        ),
        DeclareLaunchArgument(
            "min_confidence",
            default_value="0.5",
            description="接受语音 intent 的最低置信度",
        ),

        Node(
            package="external_control_pkg",
            executable="wake_up_node",       # ← 改为 C++ 编译目标（不再是 .py）
            name="wake_up_node",
            output="screen",
            condition=IfCondition(start_wake_up_node),
            parameters=[{
                "cloud_asr_url": cloud_asr_url,
                "cmd_topic": cmd_topic,
                "model_dir": model_dir,
                "energy_threshold": energy_threshold,
                "silence_timeout": silence_timeout,
                "mic_device": mic_device,
            }],
        ),

        Node(
            package="external_control_pkg",
            executable="voice_gimbal_nudge_node",
            name="voice_gimbal_nudge_node",
            output="screen",
            parameters=[
                nudge_config,
                {
                    "voice_command_topic": cmd_topic,
                    "cmd_gimbal_topic": cmd_gimbal_topic,
                    "yaw_step_rate": nudge_yaw_rate,
                    "step_duration_sec": nudge_duration,
                    "right_yaw_sign": right_yaw_sign,
                    "min_confidence": min_confidence,
                },
            ],
        ),
    ])
