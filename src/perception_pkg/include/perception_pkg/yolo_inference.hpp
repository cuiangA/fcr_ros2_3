/**
 * @file yolo_inference.hpp
 * @brief YOLOv8 ONNX 推理封装（基于 OpenCV DNN 后端）。
 *
 * 把「图像 -> YOLOv8 -> TargetArray(像素坐标)」这一段从 DetectionNode 和
 * PerceptionPipeline 中抽出来，做到只实现一份、两处复用，也方便后续平滑
 * 切换到 ONNX Runtime / TensorRT（只需替换本类内部实现）。
 *
 * 约定（与下游 DepthEstimator / PerceptionPipeline 对齐）：
 *   - 输出的 bbox 为像素坐标 [x_min, y_min, x_max, y_max]
 *   - center 为像素坐标 [cx, cy]
 *   - 输入模型为 Ultralytics 默认导出（nms=False）的 YOLOv8 ONNX：
 *       输入  [1, 3, 640, 640] (RGB, /255)
 *       输出  [1, 84, 8400]    (4 框坐标 + 80 类分数, 分数已 sigmoid ∈ [0,1])
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>
#include <string>
#include <vector>
#include <memory>

namespace perception_pkg {

class YoloInference {
public:
  /**
   * @brief 加载 YOLOv8 ONNX 模型并配置推理参数。
   * @param model_path     .onnx 文件路径（不可为空）
   * @param conf_threshold 置信度阈值
   * @param nms_threshold  NMS IoU 阈值
   * @param class_whitelist 类别白名单（为空表示输出全部 COCO 类）
   * @param device         推理设备："cuda:0" / "cpu"（无 CUDA 时自动回退 CPU）
   */
  YoloInference(const std::string& model_path,
                float conf_threshold,
                float nms_threshold,
                const std::vector<std::string>& class_whitelist = {},
                const std::string& device = "cpu");

  /// 输入 BGR 图像，输出像素坐标的检测结果（header 由调用方填充）
  vision_servo_msgs::msg::TargetArray detect(const cv::Mat& frame);

private:
  cv::dnn::Net net_;
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
