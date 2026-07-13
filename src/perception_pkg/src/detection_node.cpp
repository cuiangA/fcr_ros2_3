/**
 * @file detection_node.cpp
 * @brief YOLO 目标检测节点实现。
 *
 * 当前实现为占位代码（placeholder），模型加载和推理逻辑待填充。
 * 架构已完整：图像回调 → 预处理 → DNN 前向传递 → NMS 后处理 → 发布。
 *
 * 支持的推理路径：
 *   - ONNX Runtime（通用，CPU/GPU）
 *   - TensorRT（NVIDIA GPU 专属，FP16/INT8 量化）
 *   - OpenCV DNN（轻量，适合原型验证）
 */

#include "perception_pkg/detection_node.hpp"
#include "perception_pkg/qos.hpp"
#include <cv_bridge/cv_bridge.h>

namespace perception_pkg {

DetectionNode::DetectionNode(const rclcpp::NodeOptions& options)
  : Node("detection_node", options)
{
  declare_parameters();

  // ── 发布者：2D 检测框（使用私有命名空间 ~/detections） ───────────
  det_pub_ = this->create_publisher<vision_servo_msgs::msg::TargetArray>(
    "/perception/detections", qos::detections());

  // ── 订阅者：原始图像（通过 image_transport 获取） ─────────────────
  image_sub_ = image_transport::create_subscription(
    this, "/camera/image_raw",
    std::bind(&DetectionNode::image_callback, this, std::placeholders::_1),
    "raw");  // "raw" = 不压缩，保证图像质量

  load_model();
  RCLCPP_INFO(get_logger(), "检测节点已启动 (model=%s, device=%s)",
              model_type_.c_str(), device_.c_str());
}

void DetectionNode::declare_parameters() {
  // 声明所有可运行时配置的参数
  this->declare_parameter("model_path", "");
  this->declare_parameter("model_type", "yolov8");
  this->declare_parameter("device", "cuda:0");
  this->declare_parameter("confidence_threshold", 0.5);
  this->declare_parameter("nms_threshold", 0.45);
  this->declare_parameter("class_names", std::vector<std::string>{});
  this->declare_parameter("camera_frame", "camera_link");

  // 读取参数值到成员变量
  model_path_ = this->get_parameter("model_path").as_string();
  model_type_ = this->get_parameter("model_type").as_string();
  device_ = this->get_parameter("device").as_string();
  confidence_threshold_ = static_cast<float>(this->get_parameter("confidence_threshold").as_double());
  nms_threshold_ = static_cast<float>(this->get_parameter("nms_threshold").as_double());
  class_names_ = this->get_parameter("class_names").as_string_array();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
}

void DetectionNode::load_model() {
  // [占位] 实际模型加载逻辑（示例）：
  //   cv::dnn::Net net = cv::dnn::readNetFromONNX(model_path_);
  //   if (device_ != "cpu") { net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA); }
  //   if (device_ == "cuda:0") { net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16); }
  RCLCPP_INFO(get_logger(), "模型加载：[占位] %s", model_path_.c_str());
}

void DetectionNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  try {
    // 将 ROS Image 消息转换为 OpenCV BGR 格式
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    // 执行推理
    auto targets = infer(frame);
    // 保持原始时间戳和坐标系信息
    targets.header.stamp = msg->header.stamp;
    targets.header.frame_id = camera_frame_;
    det_pub_->publish(targets);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
  }
}

vision_servo_msgs::msg::TargetArray DetectionNode::infer(const cv::Mat& frame) {
  if (!yolo_) {
    return vision_servo_msgs::msg::TargetArray{};
  }
  return yolo_->detect(frame);
}

}  // namespace perception_pkg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception_pkg::DetectionNode)

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::DetectionNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}