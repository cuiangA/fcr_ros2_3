from voice_intent_pkg.intent_rules import resolve_gimbal_intent


def test_resolves_supported_gimbal_commands():
    assert resolve_gimbal_intent("向右一点") == "gimbal_nudge_right"
    assert resolve_gimbal_intent("请把云台向左一下") == "gimbal_nudge_left"
    assert resolve_gimbal_intent("抬高一点") == "gimbal_nudge_up"
    assert resolve_gimbal_intent("降低一点") == "gimbal_nudge_down"
    assert resolve_gimbal_intent("速度快一点") == "gimbal_speed_up"
    assert resolve_gimbal_intent("慢一些") == "gimbal_speed_down"
    assert resolve_gimbal_intent("云台停止") == "gimbal_stop"


def test_ignores_unrelated_text():
    assert resolve_gimbal_intent("跟上我") is None
    assert resolve_gimbal_intent("今天天气怎么样") is None
