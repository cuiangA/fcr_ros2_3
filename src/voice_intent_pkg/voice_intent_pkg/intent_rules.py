"""Deterministic gimbal intent rules used around the current BERT model."""

from typing import Optional, Sequence, Tuple


Rule = Tuple[str, Sequence[str]]


# Safety and stop commands are evaluated before motion commands.
GIMBAL_RULES: Sequence[Rule] = (
    (
        "gimbal_stop",
        (
            "云台停止",
            "停止云台",
            "停一下",
            "别动",
        ),
    ),
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


def resolve_gimbal_intent(text: str) -> Optional[str]:
    """Return the supported gimbal intent found in text, if any."""
    normalized = "".join(text.strip().split())
    for intent, phrases in GIMBAL_RULES:
        if any(phrase in normalized for phrase in phrases):
            return intent
    return None
