# perception_pkg

当前阶段只实现 Sony 主 RGB 图像的目标检测和跟踪。奥比中光深度融合、避障、
建图和 Nav2 暂不进入本轮代码范围。

## 当前数据流

```text
Sony image
  -> detection_node (YOLO ONNX)
  -> /perception/detections
  -> tracking_node (Kalman + Hungarian IoU association)
  -> /perception/tracks
```

`Target.bbox` 和 `Target.center` 始终使用 Sony 输入图像的像素坐标。检测和跟踪
结果保留 Sony 原始时间戳及 `frame_id`。

仓库原有 `depth_estimator_node` 和 `PerceptionPipeline` 目前仅为历史仿真兼容
代码，默认既不构建也不启动。只有显式设置
`-DPERCEPTION_BUILD_LEGACY_FUSION=ON` 才会生成它们；生产环境不得使用该选项。
双相机融合阶段开始时应单独重新设计，不能直接把 Sony bbox 像素用于索引
奥比中光深度图。

## detection_node

- 支持常见 YOLOv8 `[1, 84, 8400]` 和 YOLOv5 `[1, N, 85]` ONNX 输出。
- 启动时严格检查模型文件、输入尺寸、输出形状和类别数量，并执行模型预热。
- 实现 letterbox、归一化、OpenCV DNN 前向、坐标还原和按类别 NMS。
- 推理在后台线程运行，待处理缓存只有一帧；过载时丢弃旧帧，避免延迟累积。
- `labels_path` 为空时使用 COCO 80 类，默认只输出 `person`。
- 模型二进制不进 Git；下载脚本按 SHA-256 校验后，CMake 才将其安装到包内。
- `device=cuda_fp16` 会先检查 OpenCV CUDA DNN target；不可用时明确退出，不静默
  回退。Jetson 生产路径使用 `device=tensorrt` 和本机生成的 `.engine`。
- 每隔 `performance_log_period` 秒输出处理帧率、平均/最大推理耗时和累计丢帧数。

## tracking_node

- 8 维恒速 Kalman 状态：`[cx, cy, w, h, vx, vy, vw, vh]`。
- 按实际消息时间间隔预测，使用 IoU 代价矩阵和 Hungarian 全局关联。
- 默认自动选择面积与置信度综合得分最高的 `person`。
- 目标选择服务：`/tracking_node/set_tracking_target`。
- `/diagnostics` 报告输入超时和最近端到端延迟，并周期输出平均、P95、最大延迟。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
python3 src/perception_pkg/scripts/download_model.py
colcon build --symlink-install --packages-up-to perception_pkg
source install/setup.bash
colcon test --packages-select perception_pkg
colcon test-result --verbose
```

## 启动

默认加载随 `perception_pkg` 安装的 `yolov8n.onnx`：

```bash
ros2 launch perception_pkg perception.launch.py \
  sony_image_topic:=/sony/image_raw
```

也可以覆盖模型路径和推理设备：

```bash
ros2 launch perception_pkg perception.launch.py \
  model_path:=/absolute/path/to/yolov8n.onnx \
  device:=cpu
```

Jetson 上的 TensorRT 10 路径（engine 只能由目标机可信 ONNX 本地生成）：

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=$HOME/fcr_models/yolov8n.onnx \
  --saveEngine=$HOME/fcr_models/yolov8n_fp16.engine \
  --fp16 --memPoolSize=workspace:1024 --skipInference

ros2 launch perception_pkg perception.launch.py \
  model_path:=$HOME/fcr_models/yolov8n_fp16.engine \
  device:=tensorrt sony_image_topic:=/sony/image_raw
```

TensorRT 支持仅在检测到 `NvInfer.h`、`libnvinfer` 和 CUDA runtime 开发文件时
条件编译；普通 Humble CI 自动只构建 ONNX/OpenCV 路径。

完成 SDK 本地安装后，可在 Jetson 工作空间根目录执行可重复构建与测试：

```bash
bash src/perception_pkg/scripts/jetson_build_perception.sh "$PWD"
```

启动 Sony、TensorRT 检测和跟踪链后，执行五分钟接口采样：

```bash
ACCEPTANCE_DURATION_SECONDS=300 \
  bash src/perception_pkg/scripts/jetson_acceptance.sh \
  "$HOME/fcr_models/yolov8n_fp16.engine"
```

脚本只自动检查 topic 存在性、消息可达性、采样频率以及融合前不得出现的 3D
topic；相机 30 分钟稳定性、诊断状态和人物 ID 切换仍需实机观察确认。

也可以只调试其中一段：

```bash
# 只运行 YOLO
ros2 launch perception_pkg perception.launch.py enable_tracking:=false

# 使用外部/mock detections，只运行跟踪
ros2 launch perception_pkg perception.launch.py enable_detection:=false
```

检查接口：

```bash
ros2 topic hz /perception/detections
ros2 topic hz /perception/tracks
ros2 topic echo /perception/tracks --once
ros2 topic info /perception/detections -v
```

实机接入前先确认 Sony 发布图像的分辨率、编码、时间戳和光学坐标系稳定，再
进行模型速度与跟踪稳定性评估。
