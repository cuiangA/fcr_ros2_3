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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cloud_asr_url = LaunchConfiguration("cloud_asr_url")
    cmd_topic = LaunchConfiguration("cmd_topic")
    model_dir = LaunchConfiguration("model_dir")
    energy_threshold = LaunchConfiguration("energy_threshold")
    silence_timeout = LaunchConfiguration("silence_timeout")
    mic_device = LaunchConfiguration("mic_device")

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
            default_value="500",
            description="麦克风能量阈值（低于此值视为静音）",
        ),
        DeclareLaunchArgument(
            "silence_timeout",
            default_value="1.5",
            description="静音超时判定时间（秒）",
        ),
        DeclareLaunchArgument(
            "mic_device",
            default_value="-1",
            description="麦克风设备索引（-1 = 系统默认）",
        ),

        Node(
            package="external_control_pkg",
            executable="wake_up_node",       # ← 改为 C++ 编译目标（不再是 .py）
            name="wake_up_node",
            output="screen",
            parameters=[{
                "cloud_asr_url": cloud_asr_url,
                "cmd_topic": cmd_topic,
                "model_dir": model_dir,
                "energy_threshold": energy_threshold,
                "silence_timeout": silence_timeout,
                "mic_device": mic_device,
            }],
        ),
    ])
