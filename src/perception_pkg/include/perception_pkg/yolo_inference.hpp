/**
 * @file yolo_inference.hpp
 * @brief YOLOv8 ONNX 推理封装（基于 ONNX Runtime 1.24+）。
 *
 * 把「图像 -> YOLOv8 -> TargetArray(像素坐标)」这一段从 DetectionNode 和
 * PerceptionPipeline 中抽出来，做到只实现一份、两处复用。
 *
 * 约定（与下游 DepthEstimator / PerceptionPipeline 对齐）：
 *   - 输出的 bbox 为像素坐标 [x_min, y_min, x_max, y_max]
 *   - center 为像素坐标 [cx, cy]
 *   - 输入模型为 Ultralytics 默认导出（nms=False, simplify=True）的 YOLOv8 ONNX：
 *       输入  [1, 3, 640, 640] (RGB, /255)
 *       输出  [1, 84, 8400]    (4 框坐标 + 80 类分数, 分数已 sigmoid ∈ [0,1])
 *
 * 注：原先使用 OpenCV DNN 后端，但 OpenCV 4.5.4 对 YOLOv8 ONNX 的 broadcast
 *     Add 节点支持不全（onnx_importer parseBias 失败）。改用 ONNX Runtime 后稳定。
 */

#pragma once

#include <opencv2/core.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>
#include <onnxruntime_cxx_api.h>
#include <string>
#include <vector>
#include <memory>

namespace perception_pkg {

class YoloInference {
public:
  /**
   * @brief 加载 YOLOv8 ONNX 模型并配置推理参数。
   * @param model_path      .onnx 文件路径（不可为空）
   * @param conf_threshold  置信度阈值
   * @param nms_threshold   NMS IoU 阈值
   * @param class_whitelist 类别白名单（为空表示输出全部 COCO 类）
   * @param device          推理设备："cuda:0" / "cpu"（无 CUDA 时自动回退 CPU）
   */
  YoloInference(const std::string& model_path,
                float conf_threshold,
                float nms_threshold,
                const std::vector<std::string>& class_whitelist = {},
                const std::string& device = "cpu");

  /// 输入 BGR 图像，输出像素坐标的检测结果（header 由调用方填充）
  vision_servo_msgs::msg::TargetArray detect(const cv::Mat& frame);

private:
  // ── ONNX Runtime 资源（用 unique_ptr / Env 包裹） ───────────────
  std::unique_ptr<Ort::Env>      ort_env_;
  std::unique_ptr<Ort::Session>  ort_session_;
  Ort::AllocatorWithDefaultOptions allocator_;

  // 输入/输出节点信息
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<int64_t>     input_shape_;  // [1, 3, 640, 640]

  // ── 配置 ────────────────────────────────────────────────────────
  float conf_threshold_;
  float nms_threshold_;
  std::vector<std::string> class_whitelist_;
  std::vector<std::string> coco_names_;
  bool use_whitelist_ = false;
  int input_w_ = 640;
  int input_h_ = 640;

  static const std::vector<std::string>& coco_names();
};

}  // namespace perception_pkg
