/**
 * @file depth_estimator_node.cpp
 * @brief 深度估计节点实现 — 从深度图和 2D 检测框反投影 3D 位置。
 *
 * 异步融合策略：
 * 深度图和检测结果以不同频率到达，采用双缓存模式——
 * 任意一路新数据到达时，若另一路缓存非空则触发融合计算。
 * 这避免了严格的时间同步带来的延迟，适用于帧率不匹配的传感器配置。
 *
 * 反投影使用标准针孔相机模型：
 *   X = (u - cx) / fx * depth
 *   Y = (v - cy) / fy * depth
 *   Z = depth（mm → m 转换后）
 */

#include "perception_pkg/depth_estimator.hpp"
#include "perception_pkg/qos.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace perception_pkg {

DepthEstimatorNode::DepthEstimatorNode(const rclcpp::NodeOptions& options)
  : Node("depth_estimator_node", options), fx_(0), fy_(0), cx_(0), cy_(0)
{
  // ── 声明参数 ────────────────────────────────────────────────────
  this->declare_parameter("camera_frame", "camera_link");
  camera_frame_ = this->get_parameter("camera_frame").as_string();

  // ── 订阅者：检测结果（可靠 QoS） ─────────────────────────────────
  det_sub_ = this->create_subscription<vision_servo_msgs::msg::TargetArray>(
    "detections", qos::detections(),
    std::bind(&DepthEstimatorNode::detection_callback, this, std::placeholders::_1));

  // ── 订阅者：深度图像（通过 image_transport 获取高效传输） ────────
  depth_sub_ = image_transport::create_subscription(
    this, "depth/image_raw",
    std::bind(&DepthEstimatorNode::depth_callback, this, std::placeholders::_1),
    "raw");  // "raw" = 不压缩，保证深度值精度

  // ── 发布者：3D 目标位姿 ─────────────────────────────────────────
  target3d_pub_ = this->create_publisher<vision_servo_msgs::msg::TargetArray>(
    "~/targets_3d", qos::detections());

  RCLCPP_INFO(get_logger(), "深度估计节点已启动");
}

void DepthEstimatorNode::depth_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
  last_depth_ = depth_msg;       // 缓存最新深度帧
  if (last_detections_) estimate_and_publish();  // 检测结果已就绪 → 触发融合
}

void DepthEstimatorNode::detection_callback(
    const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& det_msg)
{
  last_detections_ = det_msg;     // 缓存最新检测结果
  if (last_depth_) estimate_and_publish();  // 深度图已就绪 → 触发融合
}

void DepthEstimatorNode::estimate_and_publish() {
  if (!last_depth_ || !last_detections_) return;

  // 将 ROS 深度图像消息转换为 OpenCV 格式（16UC1 = 16 位无符号单通道，单位毫米）
  cv::Mat depth_frame = cv_bridge::toCvShare(last_depth_, "16UC1")->image;
  auto targets_3d = *last_detections_;  // 拷贝检测结果，填充 3D 信息

  // 对每个检测目标执行深度反投影
  for (auto& target : targets_3d.targets) {
    float depth = estimate_depth(depth_frame, target.bbox);
    // depth_confidence：有效深度 = 0.8，无效深度（0） = 0.0
    target.depth_confidence = (depth > 0) ? 0.8f : 0.0f;
    compute_3d_position(target, depth);
  }

  target3d_pub_->publish(targets_3d);
}

float DepthEstimatorNode::estimate_depth(const cv::Mat& depth_frame, const float bbox[4]) {
  // 取边界框中心点作为深度采样位置
  float cx_bbox = (bbox[0] + bbox[2]) / 2.0f;
  float cy_bbox = (bbox[1] + bbox[3]) / 2.0f;
  // 钳位到图像边界内，防止越界访问
  int px = std::clamp(static_cast<int>(cx_bbox), 0, depth_frame.cols - 1);
  int py = std::clamp(static_cast<int>(cy_bbox), 0, depth_frame.rows - 1);
  // 深度图存储单位为毫米，除以 1000 转换为米
  return depth_frame.at<uint16_t>(py, px) / 1000.0f;
}

void DepthEstimatorNode::compute_3d_position(
    vision_servo_msgs::msg::Target& target, float depth)
{
  // 未标定（fx_ == 0）时无法反投影，直接跳过
  if (fx_ == 0) return;
  // 针孔模型反投影：
  //   X = (u - cx) / fx * Z   →  target.position[0]
  //   Y = (v - cy) / fy * Z   →  target.position[1]
  //   Z = depth               →  target.position[2]
  target.position[0] = (target.center[0] - cx_) / fx_ * depth;  // X（右方向）
  target.position[1] = (target.center[1] - cy_) / fy_ * depth;  // Y（下方向）
  target.position[2] = depth;                                     // Z（前方向）
}

}  // namespace perception_pkg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception_pkg::DepthEstimatorNode)
