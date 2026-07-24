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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cloud_asr_url = LaunchConfiguration("cloud_asr_url")
    cmd_topic = LaunchConfiguration("cmd_topic")
    text_topic = LaunchConfiguration("text_topic")
    model_dir = LaunchConfiguration("model_dir")
    energy_threshold = LaunchConfiguration("energy_threshold")
    silence_timeout = LaunchConfiguration("silence_timeout")
    mic_device = LaunchConfiguration("mic_device")
    start_wake_up_node = LaunchConfiguration("start_wake_up_node")
    start_dispatcher = LaunchConfiguration("start_dispatcher")
    start_intent_classifier = LaunchConfiguration("start_intent_classifier")
    publish_cloud_intents = LaunchConfiguration("publish_cloud_intents")
    classifier_model_root = LaunchConfiguration("classifier_model_root")
    embedding_model_dir = LaunchConfiguration("embedding_model_dir")
    start_command_router = LaunchConfiguration("start_command_router")
    start_chassis_control = LaunchConfiguration("start_chassis_control")
    start_keyboard_node = LaunchConfiguration("start_keyboard_node")
    cmd_gimbal_topic = LaunchConfiguration("cmd_gimbal_topic")
    manual_cmd_gimbal_topic = LaunchConfiguration("manual_cmd_gimbal_topic")
    autonomy_cmd_gimbal_topic = LaunchConfiguration("autonomy_cmd_gimbal_topic")
    router_output_cmd_topic = LaunchConfiguration("router_output_cmd_topic")
    nudge_yaw_rate = LaunchConfiguration("nudge_yaw_rate")
    nudge_pitch_rate = LaunchConfiguration("nudge_pitch_rate")
    nudge_duration = LaunchConfiguration("nudge_duration")
    right_yaw_sign = LaunchConfiguration("right_yaw_sign")
    up_pitch_sign = LaunchConfiguration("up_pitch_sign")
    min_confidence = LaunchConfiguration("min_confidence")
    gimbal_voice_command_topic = LaunchConfiguration(
        "gimbal_voice_command_topic"
    )
    chassis_voice_command_topic = LaunchConfiguration(
        "chassis_voice_command_topic"
    )
    voice_cmd_vel_topic = LaunchConfiguration("voice_cmd_vel_topic")
    manual_cmd_vel_topic = LaunchConfiguration("manual_cmd_vel_topic")
    autonomy_cmd_vel_topic = LaunchConfiguration("autonomy_cmd_vel_topic")
    chassis_output_cmd_topic = LaunchConfiguration(
        "chassis_output_cmd_topic"
    )
    chassis_linear_speed = LaunchConfiguration("chassis_linear_speed")
    chassis_angular_speed = LaunchConfiguration("chassis_angular_speed")
    chassis_nudge_duration = LaunchConfiguration("chassis_nudge_duration")

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
            "text_topic",
            default_value="/voice/text",
            description="ASR 文本输出和本地意图模型输入话题",
        ),
        DeclareLaunchArgument(
            "start_dispatcher",
            default_value="true",
            description="是否启动结构化语音意图分发器",
        ),
        DeclareLaunchArgument(
            "start_intent_classifier",
            default_value="false",
            description="是否启动本地双层意图分类节点",
        ),
        DeclareLaunchArgument(
            "publish_cloud_intents",
            default_value="true",
            description=(
                "兼容模式是否直发云端意图；使用本地双层模型时必须设为 false"
            ),
        ),
        DeclareLaunchArgument(
            "classifier_model_root",
            default_value="",
            description="双层分类模型根目录",
        ),
        DeclareLaunchArgument(
            "embedding_model_dir",
            default_value="",
            description="BGE 语义嵌入模型目录",
        ),
        DeclareLaunchArgument(
            "gimbal_voice_command_topic",
            default_value="/voice/gimbal_command",
            description="分发器输出给云台执行桥接的话题",
        ),
        DeclareLaunchArgument(
            "chassis_voice_command_topic",
            default_value="/voice/chassis_command",
            description="分发器输出给底盘语音桥接的话题",
        ),
        DeclareLaunchArgument(
            "start_chassis_control",
            default_value="true",
            description="是否启动语音底盘点动桥接和底盘命令仲裁",
        ),
        DeclareLaunchArgument(
            "voice_cmd_vel_topic",
            default_value="/voice/cmd_vel",
            description="语音底盘速度输入话题",
        ),
        DeclareLaunchArgument(
            "manual_cmd_vel_topic",
            default_value="/manual/cmd_vel",
            description="手动底盘速度输入话题",
        ),
        DeclareLaunchArgument(
            "autonomy_cmd_vel_topic",
            default_value="/autonomy/cmd_vel",
            description="自主底盘速度输入话题",
        ),
        DeclareLaunchArgument(
            "chassis_output_cmd_topic",
            default_value="/cmd_vel",
            description="底盘仲裁器输出到驱动的话题",
        ),
        DeclareLaunchArgument(
            "chassis_linear_speed",
            default_value="0.05",
            description="语音底盘点动线速度，单位 m/s",
        ),
        DeclareLaunchArgument(
            "chassis_angular_speed",
            default_value="0.20",
            description="语音底盘点动角速度，单位 rad/s",
        ),
        DeclareLaunchArgument(
            "chassis_nudge_duration",
            default_value="0.4",
            description="语音底盘点动持续时间，单位秒",
        ),
        DeclareLaunchArgument(
            "cmd_gimbal_topic",
            default_value="/voice/cmd_gimbal",
            description="语音云台控制输出话题，通常进入 command_router_node",
        ),
        DeclareLaunchArgument(
            "manual_cmd_gimbal_topic",
            default_value="/manual/cmd_gimbal",
            description="手动云台控制输入话题",
        ),
        DeclareLaunchArgument(
            "autonomy_cmd_gimbal_topic",
            default_value="/autonomy/cmd_gimbal",
            description="自主云台控制输入话题",
        ),
        DeclareLaunchArgument(
            "router_output_cmd_topic",
            default_value="/cmd_gimbal",
            description="command_router_node 输出到云台驱动的话题",
        ),
        DeclareLaunchArgument(
            "start_command_router",
            default_value="true",
            description="是否启动 command_router_node，使语音/键盘/自主控制统一仲裁",
        ),
        DeclareLaunchArgument(
            "start_keyboard_node",
            default_value="false",
            description="是否启动键盘云台控制节点",
        ),
        DeclareLaunchArgument(
            "nudge_yaw_rate",
            default_value="0.25",
            description="语音云台短动作 yaw 角速度（rad/s）",
        ),
        DeclareLaunchArgument(
            "nudge_pitch_rate",
            default_value="0.20",
            description="语音云台短动作 pitch 角速度（rad/s）",
        ),
        DeclareLaunchArgument(
            "nudge_duration",
            default_value="0.4",
            description="语音云台短动作持续时间（秒）",
        ),
        DeclareLaunchArgument(
            "right_yaw_sign",
            default_value="1.0",
            description="向右一点对应的 yaw 方向符号，实机方向反了改为 -1.0",
        ),
        DeclareLaunchArgument(
            "up_pitch_sign",
            default_value="1.0",
            description="向上一点对应的 pitch 方向符号，实机方向反了改为 -1.0",
        ),
        DeclareLaunchArgument(
            "min_confidence",
            default_value="0.5",
            description="接受语音 intent 的最低置信度",
        ),

        Node(
            package="external_control_pkg",
            executable="voice_command_dispatcher_node",
            name="voice_command_dispatcher_node",
            output="screen",
            condition=IfCondition(start_dispatcher),
            parameters=[{
                "input_topic": cmd_topic,
                "gimbal_topic": gimbal_voice_command_topic,
                "chassis_topic": chassis_voice_command_topic,
                "min_confidence": min_confidence,
            }],
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
                "text_topic": text_topic,
                "publish_cloud_intents": ParameterValue(
                    publish_cloud_intents, value_type=bool
                ),
                "model_dir": model_dir,
                "energy_threshold": energy_threshold,
                "silence_timeout": silence_timeout,
                "mic_device": mic_device,
            }],
        ),

        Node(
            package="voice_intent_pkg",
            executable="double_layer_console_voice_node",
            name="intent_classifier_node",
            output="screen",
            condition=IfCondition(start_intent_classifier),
            parameters=[{
                "model_root": classifier_model_root,
                "embedding_model_dir": embedding_model_dir,
                "text_input_topic": text_topic,
                "enable_console_input": False,
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
                    "voice_command_topic": gimbal_voice_command_topic,
                    "cmd_gimbal_topic": cmd_gimbal_topic,
                    "yaw_step_rate": nudge_yaw_rate,
                    "pitch_step_rate": nudge_pitch_rate,
                    "step_duration_sec": nudge_duration,
                    "right_yaw_sign": right_yaw_sign,
                    "up_pitch_sign": up_pitch_sign,
                    "min_confidence": min_confidence,
                    "enable_raw_text_fallback": False,
                },
            ],
        ),

        Node(
            package="external_control_pkg",
            executable="command_router_node",
            name="command_router_node",
            output="screen",
            condition=IfCondition(start_command_router),
            parameters=[{
                "manual_cmd_topic": manual_cmd_gimbal_topic,
                "voice_cmd_topic": cmd_gimbal_topic,
                "autonomy_cmd_topic": autonomy_cmd_gimbal_topic,
                "output_cmd_topic": router_output_cmd_topic,
            }],
        ),

        Node(
            package="external_control_pkg",
            executable="voice_chassis_nudge_node",
            name="voice_chassis_nudge_node",
            output="screen",
            condition=IfCondition(start_chassis_control),
            parameters=[{
                "voice_command_topic": chassis_voice_command_topic,
                "cmd_vel_topic": voice_cmd_vel_topic,
                "linear_speed": chassis_linear_speed,
                "angular_speed": chassis_angular_speed,
                "step_duration_sec": chassis_nudge_duration,
                "min_confidence": min_confidence,
            }],
        ),

        Node(
            package="external_control_pkg",
            executable="chassis_command_router_node",
            name="chassis_command_router_node",
            output="screen",
            condition=IfCondition(start_chassis_control),
            parameters=[{
                "manual_cmd_topic": manual_cmd_vel_topic,
                "voice_cmd_topic": voice_cmd_vel_topic,
                "autonomy_cmd_topic": autonomy_cmd_vel_topic,
                "output_cmd_topic": chassis_output_cmd_topic,
            }],
        ),

        Node(
            package="external_control_pkg",
            executable="keyboard_gimbal_control_node",
            name="keyboard_gimbal_control_node",
            output="screen",
            condition=IfCondition(start_keyboard_node),
            parameters=[{
                "cmd_gimbal_topic": manual_cmd_gimbal_topic,
                "yaw_step_rate": nudge_yaw_rate,
                "pitch_step_rate": nudge_pitch_rate,
                "right_yaw_sign": right_yaw_sign,
                "up_pitch_sign": up_pitch_sign,
            }],
        ),
    ])
