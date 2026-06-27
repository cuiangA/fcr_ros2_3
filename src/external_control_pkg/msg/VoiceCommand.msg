# VoiceCommand.msg
# 语音控制节点解析后的指令消息
# 支持单指令和多指令（如"跟上我，走近点"→ start_following + distance_adjust）

std_msgs/Header header

# ── 意图列表 ─────────────────────────────────────────────────
# 可能同时包含多个意图（多指令场景）
# 可选值: start_following, stop_following, emergency_stop,
#         switch_target, distance_adjust, gimbal_home, status_query
string[] intents

# ── 各意图的置信度（与 intents 一一对应） ─────────────────────
float32[] confidences

# ── ASR 原始文本 ─────────────────────────────────────────────
string raw_text

# ── 参数（各意图的细化参数，无对应参数时填默认值） ──────────────
# 距离调整相关
float32 distance         # 距离值（米），-1 表示未指定
string unit              # 单位: "m" / "cm"，空字符串表示未指定

# 速度设定
string speed             # "fast" / "mid" / "slow"，空字符串表示未指定

# 切换目标相关
string target_desc       # 目标描述，如"穿红衣服的""高个子"，空字符串表示未指定

# 是否跟随模式
bool follow              # true=开启跟随，false=关闭
