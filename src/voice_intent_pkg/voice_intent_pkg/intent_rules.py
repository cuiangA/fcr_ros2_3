"""Deterministic control intent rules used around the current BERT model."""

from typing import Optional, Sequence, Tuple


Rule = Tuple[str, Sequence[str]]

GIMBAL_HOME_PHRASES: Sequence[str] = (
    "云台回中",
    "云台归位",
    "镜头回正",
    "恢复云台中位",
    "回到中间",
)


# Safety and stop commands are evaluated before motion/start commands.
CONTROL_RULES: Sequence[Rule] = (
    (
        "camera_stop_recording",
        (
            "停止录像",
            "结束录像",
            "停止录制",
            "结束录制",
            "停止拍摄视频",
        ),
    ),
    (
        "camera_take_photo",
        (
            "相机拍照",
            "拍一张照片",
            "拍张照片",
            "拍一张",
            "拍照",
        ),
    ),
    (
        "camera_start_recording",
        (
            "开始录像",
            "开始录制",
            "开始拍摄视频",
            "录一段视频",
            "录像",
        ),
    ),
    (
        "gimbal_stop",
        (
            "云台停止",
            "停止云台",
            "停一下",
            "别动",
        ),
    ),
    ("gimbal_home", GIMBAL_HOME_PHRASES),
    (
        "gimbal_nudge_right",
        (
            "向右一点",
            "向右一下",
            "右转一点",
            "右移一点",
            "往右一点",
            "向右移动一点",
            "往右移动一点",
            "向右转一点",
            "往右转一点",
        ),
    ),
    (
        "gimbal_nudge_left",
        (
            "向左一点",
            "向左一下",
            "左转一点",
            "左移一点",
            "往左一点",
            "向左移动一点",
            "往左移动一点",
            "向左转一点",
            "往左转一点",
        ),
    ),
    (
        "gimbal_nudge_up",
        (
            "向上一点",
            "向上一下",
            "抬高一点",
            "往上一点",
            "上移一点",
            "向上移动一点",
            "往上移动一点",
        ),
    ),
    (
        "gimbal_nudge_down",
        (
            "向下一点",
            "向下一下",
            "降低一点",
            "往下一点",
            "下移一点",
            "向下移动一点",
            "往下移动一点",
        ),
    ),
    (
        "gimbal_speed_up",
        (
            "速度快一点",
            "快一点",
            "快一些",
        ),
    ),
    (
        "gimbal_speed_down",
        (
            "速度慢一点",
            "慢一点",
            "慢一些",
        ),
    ),
)


def _rule_text_variants(text: str) -> Sequence[str]:
    normalized = "".join(text.strip().split())
    chinese_only = "".join(
        character
        for character in normalized
        if "\u4e00" <= character <= "\u9fff"
    )
    return (normalized, chinese_only)


def resolve_control_intent(text: str) -> Optional[str]:
    """Return the supported deterministic control intent found in text."""
    variants = _rule_text_variants(text)
    for intent, phrases in CONTROL_RULES:
        if any(phrase in variant for variant in variants for phrase in phrases):
            return intent
    return None


def resolve_gimbal_intent(text: str) -> Optional[str]:
    """Backward-compatible alias for callers using the original function name."""
    return resolve_control_intent(text)


def is_explicit_gimbal_home(text: str) -> bool:
    """Return true only when the text explicitly asks the gimbal to return home."""
    return any(
        phrase in variant
        for variant in _rule_text_variants(text)
        for phrase in GIMBAL_HOME_PHRASES
    )
