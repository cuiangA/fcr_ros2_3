# 语音指令数据集设计方案

## 1. 目标

本数据集用于训练 FCR ROS 2 项目的语音指令理解模型，使模型能够区分：

- 控制对象：系统、云台、底盘、相机、自主任务；
- 动作类型：移动、旋转、停止、回中、拍照、录像、跟随、运镜等；
- 动作参数：方向、距离、速度、目标 ID 和目标描述；
- 无关对话、歧义指令、否定指令和异常输入。

数据集应同时满足两个目标：

1. 兼容当前单标签 BERT 分类器；
2. 支持未来升级为“控制对象 + 动作 + 参数”的分层模型。

本文档只定义生成与标注方案，不包含实际生成的数据。

## 2. 数据格式

推荐使用 JSON Lines，每行存放一条独立样本。

```json
{
  "id": "gimbal_right_0001",
  "text": "把云台向右转一点",
  "flat_intent": "gimbal_nudge_right",
  "target": "gimbal",
  "action": "nudge",
  "direction": "right",
  "amount": "small",
  "distance": null,
  "unit": null,
  "speed": null,
  "target_id": null,
  "target_desc": null,
  "executable": true,
  "source": "manual",
  "noise_type": "clean",
  "template_group": "gimbal_direction_polite_01"
}
```

### 2.1 核心字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | string | 样本唯一编号 |
| `text` | string | 用户原始语句 |
| `flat_intent` | string | 兼容当前 BERT 的扁平意图标签 |
| `target` | string | 控制对象 |
| `action` | string | 动作类型 |
| `executable` | bool | 当前语句是否允许直接执行 |
| `source` | string | 样本来源，如人工、扩写、ASR、噪声增强 |
| `noise_type` | string | 输入噪声类型 |
| `template_group` | string | 语义模板组，用于防止数据划分泄漏 |

### 2.2 参数字段

| 字段 | 可选值或格式 | 默认值 |
|---|---|---|
| `direction` | `left/right/up/down/forward/backward` | `null` |
| `amount` | `small/medium/large` | `null` |
| `distance` | 浮点数 | `null` |
| `unit` | `m/cm` | `null` |
| `speed` | `slow/mid/fast` | `null` |
| `target_id` | 整数 | `null` |
| `target_desc` | 字符串 | `null` |

## 3. 分层标签体系

### 3.1 控制对象标签

```text
system
gimbal
chassis
camera
autonomy
unknown
```

### 3.2 动作标签

```text
emergency_stop
stop
cancel
query_status
nudge
translate
rotate
home
speed_adjust
take_photo
start_recording
stop_recording
start_following
stop_following
pause_following
resume_following
distance_adjust
switch_target
start_cinematic
stop_cinematic
orbit_target
push_in
pull_out
track_target
chat
clarify
```

### 3.3 扁平意图标签

当前单标签 BERT 使用 `flat_intent`。建议至少包含以下集合。

#### 安全与系统

```text
emergency_stop
stop_all
cancel_action
status_query
```

#### 云台

```text
gimbal_nudge_left
gimbal_nudge_right
gimbal_nudge_up
gimbal_nudge_down
gimbal_stop
gimbal_home
gimbal_speed_up
gimbal_speed_down
```

#### 底盘

```text
chassis_move_forward
chassis_move_backward
chassis_move_left
chassis_move_right
chassis_turn_left
chassis_turn_right
chassis_stop
chassis_speed_up
chassis_speed_down
```

#### 相机

```text
camera_take_photo
camera_start_recording
camera_stop_recording
```

#### 跟随任务

```text
start_following
stop_following
pause_following
resume_following
distance_adjust
speed_adjust
switch_target
```

#### 运镜任务

```text
start_cinematic
stop_cinematic
orbit_target
push_in
pull_out
track_target
```

#### 拒绝与对话

```text
clarification_required
chat
```

## 4. 样本类型

每个核心意图都应覆盖多种自然语言形式。

| 类型 | 示例 | 目的 |
|---|---|---|
| 标准表达 | 云台向右一点 | 建立基础语义 |
| 口语表达 | 镜头往右挪一下 | 适配真实说话方式 |
| 礼貌表达 | 请把云台向右移动一点 | 避免礼貌词干扰 |
| 简略表达 | 云台右一点 | 适配短指令 |
| 带参数 | 底盘向前移动半米 | 训练参数抽取 |
| 带上下文词 | 帮我拍一张照片 | 适配自然句式 |
| 错别字 | 云台回钟 | 适配 ASR 和输入错误 |
| 字母夹杂 | 向上一i点 | 适配终端和 ASR 噪声 |
| 否定表达 | 不要向右转 | 防止反向执行 |
| 相似反例 | 底盘向右一点 | 学习控制对象差异 |
| 无关对话 | 今天天气怎么样 | 抑制误触发 |
| 歧义表达 | 向右一点 | 学习拒绝执行 |

## 5. 最小对照样本

数据集必须包含大量只改变一个关键词的最小对照组。

### 5.1 控制对象对照

```text
云台向右一点 -> gimbal_nudge_right
底盘向右一点 -> chassis_move_right
```

```text
云台停止 -> gimbal_stop
底盘停止 -> chassis_stop
停止录像 -> camera_stop_recording
停止跟随 -> stop_following
全部停止 -> stop_all
紧急停止 -> emergency_stop
```

### 5.2 动作对照

```text
底盘向右移动 -> chassis_move_right
底盘向右转动 -> chassis_turn_right
云台向右转动 -> gimbal_nudge_right
```

```text
镜头回正 -> gimbal_home
开始录像 -> camera_start_recording
开始运镜 -> start_cinematic
```

### 5.3 否定对照

```text
云台向右转 -> gimbal_nudge_right
云台不要向右转 -> cancel_action 或 clarification_required
```

```text
开始录像 -> camera_start_recording
不要开始录像 -> cancel_action
```

## 6. 歧义与拒绝样本

涉及硬件运动时，无法确定控制对象的语句不能默认执行。

```text
向右一点
快一点
停一下
靠近一些
```

建议标注为：

```json
{
  "flat_intent": "clarification_required",
  "target": "unknown",
  "action": "clarify",
  "executable": false
}
```

以下情况也应标注为不可执行：

- 语义不完整；
- 同时出现冲突方向；
- 缺少必要目标；
- 带有明确否定；
- 与机器人控制无关；
- 模型无法确认控制对象。

## 7. 复合指令

项目的 `VoiceCommand` 消息支持意图列表，因此数据结构应预留多意图能力。

```json
{
  "id": "compound_0001",
  "text": "云台向右一点然后拍照",
  "intents": [
    {
      "flat_intent": "gimbal_nudge_right",
      "order": 1
    },
    {
      "flat_intent": "camera_take_photo",
      "order": 2
    }
  ],
  "executable": true
}
```

典型复合指令：

```text
开始跟随并开始录像
云台向右一点然后拍照
切换到二号目标并靠近一些
停止跟随并结束录像
```

第一版单标签模型可以只训练单意图子集，但原始数据不应丢失复合意图结构。

## 8. 数据规模

### 8.1 初版建议

| 数据类型 | 建议数量 |
|---|---:|
| 每个核心意图 | 150-300 条 |
| 每个低频规划意图 | 80-150 条 |
| `chat/unknown` | 总量的 15%-20% |
| 最小对照与相似反例 | 总量的至少 20% |
| 噪声样本 | 总量的约 10% |
| 否定和歧义样本 | 总量的约 10%-15% |

预计初版总量：

```text
6000-10000 条
```

### 8.2 类别平衡

不要求所有类别绝对相等，但应避免：

- `chat` 数量远高于控制类；
- 云台样本远多于底盘和相机；
- `stop` 类只包含少量固定句式；
- 安全指令召回率因样本过少而降低；
- 方向词数量不平衡。

## 9. 数据生成流程

1. 固化标签清单和字段定义。
2. 每个意图由人工编写 20-30 条种子语句。
3. 基于种子扩写口语、礼貌、简略和带参数表达。
4. 为每个动作生成控制对象最小对照组。
5. 增加否定、歧义和无关对话。
6. 增加错别字、同音字、字母夹杂和 ASR 噪声。
7. 对生成样本进行去重和冲突检测。
8. 由人工复核控制对象、方向和安全标签。
9. 按模板组划分训练、验证和测试集。
10. 导出单标签训练集和结构化完整版。

## 10. 数据来源标记

建议使用以下 `source`：

```text
manual_seed
manual_test
llm_paraphrase
template_generated
asr_collected
noise_augmented
counterfactual
```

生成数据必须保留来源，便于后续分析模型是否只适应合成表达。

## 11. 数据清洗

清洗步骤包括：

- Unicode 规范化；
- 去除首尾空白；
- 保留原始文本副本；
- 完全重复文本去重；
- 近似重复语句聚类；
- 同一句文本多标签冲突检查；
- 方向、对象和动作逻辑检查；
- 参数单位统一；
- 不安全指令人工复核。

不能在清洗阶段直接删除所有字母、数字和标点，因为目标 ID、距离和噪声样本可能依赖这些字符。可以额外生成规范化字段，但必须保留原始文本。

## 12. 数据集划分

推荐比例：

```text
train: 70%
validation: 15%
test: 15%
```

划分必须以 `template_group` 为单位，不能逐条随机划分。

错误示例：

```text
训练集：请把云台向右移动一点
测试集：把云台向右移动一下
```

这两句来自同一个模板，如果分别进入训练集和测试集，会造成测试指标虚高。

测试集建议满足：

- 以人工原创语句为主；
- 不参与扩写；
- 包含未见过的句式；
- 包含控制对象对照；
- 包含否定和歧义表达；
- 包含真实 ASR 输出。

## 13. 质量控制

### 13.1 自动检查

- ID 唯一性；
- 必填字段完整性；
- 标签是否在白名单中；
- `target/action/flat_intent` 是否一致；
- 距离和单位是否匹配；
- 方向是否适用于目标对象；
- 完全重复和跨集合重复；
- 同文本冲突标签。

### 13.2 人工复核

重点复核：

- `emergency_stop`；
- 各类 `stop`；
- 云台回中；
- 底盘移动与转向；
- 否定表达；
- 复合命令顺序；
- 控制对象不明确的语句。

建议安全相关样本全部人工复核，普通样本采用抽样复核。

## 14. 训练方式

### 14.1 短期方案

```text
结构化 JSONL
  -> 导出 text + flat_intent
  -> 训练单标签 BERT
  -> 参数继续由规则或独立抽取器处理
```

优点：改动小，可以复用当前 `bert_predict.py` 和控制链路。

### 14.2 长期方案

```text
共享中文 BERT 编码器
  |- target 分类头
  |- action 分类头
  |- direction/amount/speed 分类头
  |- 距离与目标 ID 抽取
  `- 目标描述序列标注
```

该方案可以自然区分：

```text
云台向右一点
底盘向右一点
相机拍一张照片
```

同时避免为所有控制对象、动作和参数组合建立无限增长的扁平标签。

## 15. 验收指标

除总体准确率外，应单独统计：

| 指标 | 关注点 |
|---|---|
| `target_accuracy` | 是否识别正确控制对象 |
| `intent_macro_f1` | 各意图是否均衡有效 |
| `direction_accuracy` | 左右上下是否正确 |
| `slot_f1` | 距离、速度、目标等参数 |
| `unknown_recall` | 歧义和未知语句是否被拒绝 |
| `emergency_stop_recall` | 急停是否不漏检 |
| `false_execution_rate` | 不应执行的语句被执行的比例 |

机器人控制场景中，`false_execution_rate` 比普通分类准确率更重要。模型高置信度但语义错误时，必须由规则、安全门或澄清机制阻止执行。

## 16. 建议实施顺序

| 阶段 | 工作内容 |
|---|---|
| 1 | 固化标签、字段和歧义规则 |
| 2 | 人工编写种子语句和测试集 |
| 3 | 生成扩写、对照、噪声和否定样本 |
| 4 | 人工审核安全类和运动类样本 |
| 5 | 导出单标签数据并训练新版 BERT |
| 6 | 评估控制对象、动作方向和误执行率 |
| 7 | 接入规则安全门并进行 ROS 2 仿真验证 |
| 8 | 小幅度实机验证后逐步开放执行能力 |
