# 双层意图模型接入与测试

## 1. 目录约定

模型不提交到 Git。Jetson 上应准备如下目录：

```text
~/ros2_ws/models/
├── classifier_v2/
│   ├── bert_intent_output/
│   │   ├── config.json
│   │   ├── model.safetensors
│   │   └── tokenizer*.json
│   └── council/
│       ├── 0_move_command/
│       ├── 1_vision_command/
│       ├── 2_camera_mode_command/
│       ├── 3_recording_command/
│       ├── 4_stop_command/
│       ├── 5_status_query_command/
│       └── 6_distractor_command/
└── bge-base-zh-v1.5/
```

`bge-base-zh-v1.5` 是细分类语义 SVM 的特征提取模型。缺少它时，只传输
`classifier_v2` 仍不能启动完整的双委员会分类器。

## 2. Windows 向 Jetson 传输新模型

在 Windows PowerShell 中执行：

```powershell
ssh nvidia@192.168.50.35 "mkdir -p ~/ros2_ws/models/classifier_v2"
sftp -o ServerAliveInterval=15 -o ServerAliveCountMax=6 nvidia@192.168.50.35
```

进入 SFTP 后执行：

```text
lcd D:/code/fcr_ros2/classifier_
cd /home/nvidia/ros2_ws/models/classifier_v2
put -r bert_intent_output
put -r council
exit
```

若大文件中断，重新进入 SFTP，定位到两端对应目录后续传：

```text
lcd D:/code/fcr_ros2/classifier_/bert_intent_output
cd /home/nvidia/ros2_ws/models/classifier_v2/bert_intent_output
reput model.safetensors
```

## 3. 准备 Python 依赖和语义模型

```bash
source ~/venvs/fcr_bert/bin/activate
python -m pip install "setuptools<80" sentence-transformers scikit-learn joblib
python -m pip check
```

下载语义模型到外置模型目录：

```bash
hf download \
  BAAI/bge-base-zh-v1.5 \
  --local-dir ~/ros2_ws/models/bge-base-zh-v1.5
```

若 Jetson 下载速度慢，可在 Windows 下载该模型后按第 2 节的方法传输。

## 4. 更新并构建项目

```bash
cd ~/ros2_ws/src/fcr_ros2_3
git pull --ff-only origin main

source /opt/ros/humble/setup.bash
source ~/venvs/fcr_bert/bin/activate
VENV_SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
export PYTHONPATH="$VENV_SITE${PYTHONPATH:+:$PYTHONPATH}"
export CUDA_VISIBLE_DEVICES=""

colcon build \
  --packages-select voice_intent_pkg external_control_pkg \
  --allow-overriding voice_intent_pkg external_control_pkg \
  --cmake-args -DBUILD_WAKE_UP_NODE=OFF
source install/setup.bash
```

确认新入口已经安装：

```bash
ros2 pkg executables voice_intent_pkg
```

输出中应包含：

```text
voice_intent_pkg double_layer_console_voice_node
```

## 5. 仅验证模型输出

终端 1：

```bash
source ~/venvs/fcr_bert/bin/activate
VENV_SITE=$(python -c 'import site; print(site.getsitepackages()[0])')
source /opt/ros/humble/setup.bash
source ~/ros2_ws/src/fcr_ros2_3/install/setup.bash
export PYTHONPATH="$VENV_SITE${PYTHONPATH:+:$PYTHONPATH}"
export CUDA_VISIBLE_DEVICES=""

ros2 run voice_intent_pkg double_layer_console_voice_node --ros-args \
  -p model_root:=$HOME/ros2_ws/models/classifier_v2 \
  -p embedding_model_dir:=$HOME/ros2_ws/models/bge-base-zh-v1.5 \
  -p device:=cpu
```

终端 2：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/src/fcr_ros2_3/install/setup.bash
ros2 topic echo /external/intent_result std_msgs/msg/String
```

依次在终端 1 输入：

```text
云台向右一点
底盘向右移动一点
云台向上看一点
云台回中
给我拍一张照片
开始录像
跟随前面的人
查询云台状态
今天天气怎么样
```

重点检查：

- `云台向右一点`：大类为视野朝向，最终意图为 `gimbal_nudge_right`。
- `底盘向右移动一点`：大类为移动，最终意图为 `chassis_move_right`。
- 拍照与录像分别输出 `camera_take_photo` 和
  `camera_start_recording`。
- 干扰项的 `published` 必须为 `false`。

## 6. 接入已验证的真实云台链路

先按项目既有步骤启动 `can0`、`gimbal_driver`、语音桥接和
`command_router_node`，然后启动第 5 节的双层模型节点。

先只输入以下三个已存在执行链路的动作：

```text
云台向右一点
云台向左一点
云台回中
```

观察：

```bash
ros2 topic echo /external/voice_command
ros2 topic echo /voice/cmd_gimbal
ros2 topic echo /cmd_gimbal
ros2 topic echo /gimbal/status
```

当前模型还能识别底盘、相机、运镜和状态查询意图，但其中没有对应执行节点的
动作只完成“识别并发布 VoiceCommand”，不代表硬件动作已经实现。

## 7. 输出含义

节点每次输入会输出：

- `coarse`：BERT 判断的大类和置信度；
- `fine`：双 SVM 判断的细类、困惑度和最终采用的委员会；
- `control_intent`：映射后的 ROS 英文意图；
- `published`：是否通过门限并发布到 `/external/voice_command`。

默认门限：

- BERT 大类置信度不低于 `0.60`；
- 至少一个细分类器困惑度不高于 `0.05`；
- 干扰项永远不发布控制消息。
