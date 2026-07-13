# BERT 控制台云台验证节点

这个包用于跳过麦克风和 ASR，直接从终端输入中文文本。每条文本都会先经过
BERT 推理，节点会打印 BERT 原始意图和置信度，然后发布现有的
`/external/voice_command`，接入已经验证过的云台控制链路。

当前 BERT 只有 7 个训练类别，不包含云台左、右、上、下、停止和速度调整。
因此节点会明确区分两类输出：

- `[BERT]`：模型真实输出；
- `[CONTROL]`：最终发布到 ROS 2 的控制意图。

方向类指令由 BERT 推理后的云台规则层补齐，`control_source` 会显示为
`gimbal_rule`，不会把规则结果伪装成 BERT 分类结果。

完整实机步骤见 `docs/bert_console_gimbal_test.md`。
