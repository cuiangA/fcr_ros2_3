# T01 2D 跟踪基线 v2.5

## 基准信息

| 条目 | 值 |
|------|-----|
| 标签 | `baseline-v2.5-tracking` |
| 分支 | `zhou_mi` |
| Commit | `a3e16038d0934ad16266b9074deb1e307d40ca73` |
| 日期 | 2026-07-21 |

## 配置快照

所有参数文件位于 `output/baseline-v2.5/config/`：

| 文件 | 说明 |
|------|------|
| `tracking_params.yaml` | ByteTrack V2 完整 42 参数（阈值/门控/代价权重/生命周期） |
| `detection_params.yaml` | YOLO 检测参数（置信度 0.10 / NMS 0.45 / 输入 640x640） |
| `manifest.json` | 模型清单（ONNX SHA-256, opset 12, COCO-80） |

## 模型哈希

| 模型 | 哈希 | 路径 |
|------|------|------|
| yolov8n.onnx | `e374e187b211da8b9db2a0863ceac5ff11d903cbf59a36e32b34aa8d178f527b` | `src/perception_pkg/models/yolov8n.onnx` |
| yolov8n_fp16.engine | **TODO**（需在 Jetson 上执行 `sha256sum ~/fcr_models/yolov8n_fp16.engine`） | `$HOME/fcr_models/yolov8n_fp16.engine` |

## 性能基线

- **TODO**：在 Jetson 上运行 `scripts/jetson_baseline_benchmark.sh` 采集
- **开发机 CPU 基线**：使用时收集，记录于 `performance_baseline.json`

采集指标：
- 处理帧率 (FPS)：平均 / 最大
- 推理延迟 (ms)：平均 / 最大 / P95
- 端到端延迟 (ms)：平均 / 最大 / P95
- 丢帧数

## 测试 Rosbag

| 场景 | 文件 | 时长 | 状态 |
|------|------|------|------|
| 单人 | `bags/t01_single_person/` | ~30s | **TODO** |
| 遮挡 | `bags/t01_occlusion/` | ~30s | **TODO** |
| 双人交叉 | `bags/t01_crossing/` | ~30s | **TODO** |
| 云台转动 | `bags/t01_gimbal_rotate/` | ~30s | **TODO** |

### Bag 录制方法

```bash
# 1. 启动视频回放
ros2 run perception_pkg video_publisher --ros-args \
  -p video_path:=/path/to/your_video.mp4 \
  -p loop:=false

# 2. 启动感知管线（不启动控制）
ros2 launch bringup_pkg fcr_perception_observe.launch.py

# 3. 录制 rosbag
ros2 bag record \
  /sony/image_raw /perception/detections /perception/tracks \
  -o bags/t01_<scene_name>
```

## 回滚方法

```bash
# 回退到基线版本
git checkout baseline-v2.5-tracking

# 若同时需恢复配置（live 替换）
cp output/baseline-v2.5/config/tracking_params.yaml \
   src/perception_pkg/config/tracking_params.yaml
cp output/baseline-v2.5/config/detection_params.yaml \
   src/perception_pkg/config/detection_params.yaml
```

## 追踪清单

- [x] Git 标签 `baseline-v2.5-tracking`
- [x] 配置文件快照
- [x] ONNX 模型哈希
- [ ] TensorRT engine 哈希（需 Jetson）
- [ ] 性能基线（需 Jetson）
- [ ] 单人 rosbag
- [ ] 遮挡 rosbag
- [ ] 双人交叉 rosbag
- [ ] 云台转动 rosbag
