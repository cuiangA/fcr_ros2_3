from voice_intent_pkg.intent_rules import (
    is_explicit_gimbal_home,
    resolve_control_intent,
    resolve_gimbal_intent,
)


def test_resolves_supported_gimbal_commands():
    assert resolve_gimbal_intent("向右一点") == "gimbal_nudge_right"
    assert resolve_gimbal_intent("请把云台向左一下") == "gimbal_nudge_left"
    assert resolve_gimbal_intent("抬高一点") == "gimbal_nudge_up"
    assert resolve_gimbal_intent("降低一点") == "gimbal_nudge_down"
    assert resolve_gimbal_intent("速度快一点") == "gimbal_speed_up"
    assert resolve_gimbal_intent("慢一些") == "gimbal_speed_down"
    assert resolve_gimbal_intent("云台停止") == "gimbal_stop"
    assert resolve_gimbal_intent("云台回中") == "gimbal_home"


def test_tolerates_non_chinese_noise_inside_gimbal_phrases():
    assert resolve_gimbal_intent("向上一i点") == "gimbal_nudge_up"
    assert resolve_gimbal_intent("云台往右移x动一点") == "gimbal_nudge_right"
    assert resolve_gimbal_intent("云台回i中") == "gimbal_home"


def test_requires_explicit_home_language():
    assert is_explicit_gimbal_home("请让云台回中")
    assert is_explicit_gimbal_home("镜头回i正")
    assert not is_explicit_gimbal_home("向上一i点")


def test_resolves_camera_commands():
    assert resolve_control_intent("帮我拍一张照片") == "camera_take_photo"
    assert resolve_control_intent("开始录像") == "camera_start_recording"
    assert resolve_control_intent("停止录像") == "camera_stop_recording"


def test_ignores_unrelated_text():
    assert resolve_gimbal_intent("跟上我") is None
    assert resolve_gimbal_intent("今天天气怎么样") is None
