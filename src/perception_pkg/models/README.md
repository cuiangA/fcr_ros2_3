# YOLO model artifacts

`yolov8n.onnx` is the runtime model used by `perception_pkg`. It was exported
from the COCO-pretrained Ultralytics `yolov8n.pt` model with a fixed `640x640` input,
ONNX opset 12, simplification enabled, and dynamic shapes disabled.

Expected tensors:

```text
input:  [1, 3, 640, 640]
output: [1, 84, 8400]
```

Runtime ONNX SHA-256:

```text
e374e187b211da8b9db2a0863ceac5ff11d903cbf59a36e32b34aa8d178f527b
```

The model binaries are intentionally ignored by Git. After the
`perception-models-v1` GitHub Release asset has been uploaded, stage and verify the
ONNX model before building:

```bash
python3 src/perception_pkg/scripts/download_model.py
```

CMake installs the ONNX file when it is present, but a missing model no longer
breaks compilation or CI. `yolov8n.pt` is a local source artifact only. TensorRT
engines are generated on the target Jetson and are never distributed because they
depend on the TensorRT/CUDA/GPU environment.
