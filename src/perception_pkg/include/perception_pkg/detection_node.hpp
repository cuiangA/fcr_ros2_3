/**
 * @file detection_node.hpp
 * @brief YOLO 目标检测节点 — 对输入图像执行 DNN 推理，输出 2D 目标边界框。
 *
 * 支持多种推理后端（YOLOv5 / YOLOv8 / TensorRT），通过 model_type 参数切换。
 * 输入为 sensor_msgs/Image（通过 image_transport），输出为自定义 TargetArray 消息。
 *
 * 设计要点：
 *   - 支持 GPU（CUDA）和 CPU 推理，通过 device 参数切换
 *   - 预分配推理缓冲区（preprocess_buffer_）减少动态内存分配
 *   - NMS 后处理过滤重叠检测框，确保每个物体只输出一个最优框
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include "perception_pkg/yolo_inference.hpp"
#include <memory>
#include <string>

namespace perception_pkg {

class DetectionNode : public rclcpp::Node {
public:
  explicit DetectionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /// 声明并读取所有 ROS 参数
  void declare_parameters();
  /// 加载 DNN 模型（ONNX / TensorRT 引擎）
  void load_model();
  /// 图像回调 — 对每一帧执行预处理 → 推理 → 后处理 → 发布
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  /// 执行 DNN 推理（预处理 + 前向传递 + NMS 后处理）
  vision_servo_msgs::msg::TargetArray infer(const cv::Mat& frame);

  // ── 配置参数 ────────────────────────────────────────────────────
  std::string model_path_;              ///< ONNX/TensorRT 模型文件路径
  std::string model_type_;              ///< 模型类型："yolov8" / "yolov5" / "tensorrt"
  std::string device_;                  ///< 推理设备："cuda:0" / "cpu"
  float confidence_threshold_;          ///< 置信度阈值（低于此值被滤除）
  float nms_threshold_;                 ///< NMS IoU 阈值
  std::vector<std::string> class_names_; ///< 目标类别名称列表
  std::string camera_frame_;            ///< 相机坐标系名称

  // ── 发布者与订阅者 ─────────────────────────────────────────────
  rclcpp::Publisher<vision_servo_msgs::msg::TargetArray>::SharedPtr det_pub_;
  image_transport::Subscriber image_sub_;

  // ── 预分配缓冲区（避免每次推理时重新分配内存） ─────────────────
  cv::Mat preprocess_buffer_;

  // ── YOLOv8 推理封装（OpenCV DNN 后端） ────────────────────────
  std::unique_ptr<YoloInference> yolo_;
};

}  // namespace perception_pkg
