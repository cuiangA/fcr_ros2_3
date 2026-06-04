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
#include <opencv2/core/mat.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace perception_pkg {

DepthEstimatorNode::DepthEstimatorNode(const rclcpp::NodeOptions& options)
  : Node("depth_estimator_node", options), fx_(0), fy_(0), cx_(0), cy_(0)
{
  // ── 声明参数 ────────────────────────────────────────────────────
  this->declare_parameter("camera_frame", "camera_link");
  this->declare_parameter("use_depth_camera", false);
  this->declare_parameter("real_target_width", 0.3);
  this->declare_parameter("real_target_height", 0.3);

  camera_frame_ = this->get_parameter("camera_frame").as_string();
  use_depth_camera_ = this->get_parameter("use_depth_camera").as_bool();
  real_target_width_ = this->get_parameter("real_target_width").as_double();
  real_target_height_ = this->get_parameter("real_target_height").as_double();

  // ── 订阅者：检测结果（可靠 QoS） ─────────────────────────────────
  det_sub_ = this->create_subscription<vision_servo_msgs::msg::TargetArray>(
    "/perception/detections", qos::detections(),
    std::bind(&DepthEstimatorNode::detection_callback, this, std::placeholders::_1));

  // ── 订阅者：相机内参（TRANSIENT_LOCAL，迟加入也能拿到最后一次标定） ─────
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", qos::camera_info(),
    std::bind(&DepthEstimatorNode::camera_info_callback, this, std::placeholders::_1));

  // ── 订阅者：深度图像（仅在 use_depth_camera=true 时订阅） ────────
  if (use_depth_camera_) {
    depth_sub_ = image_transport::create_subscription(
      this, "/camera/depth/image_raw",
      std::bind(&DepthEstimatorNode::depth_callback, this, std::placeholders::_1),
      "raw");  // "raw" = 不压缩，保证深度值精度
  }

  // ── 发布者：3D 目标位姿 ─────────────────────────────────────────
  target3d_pub_ = this->create_publisher<vision_servo_msgs::msg::TargetArray>(
    "/perception/targets_3d", qos::detections());

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
  // 有深度图时按双缓存策略触发；没有深度相机时直接走 bbox 面积推算
  if (last_depth_ || !use_depth_camera_) estimate_and_publish();
}

void DepthEstimatorNode::camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
{
  fx_ = info_msg->k[0];
  fy_ = info_msg->k[4];
  cx_ = info_msg->k[2];
  cy_ = info_msg->k[5];
  width_ = static_cast<int>(info_msg->width);
  height_ = static_cast<int>(info_msg->height);
  if (!info_msg->header.frame_id.empty()) {
    camera_frame_ = info_msg->header.frame_id;
  }
}

void DepthEstimatorNode::estimate_and_publish() {
  if (!last_detections_) return;

  auto targets_3d = *last_detections_;  // 拷贝检测结果，填充 3D 信息

  for (auto& target : targets_3d.targets) {
    float depth = 0.0f;
    float confidence = 0.0f;

    // 方法 1：深度图采样（有 Gazebo 深度相机时优先使用）
    if (last_depth_) {
      cv::Mat depth_frame = cv_bridge::toCvShare(last_depth_, "16UC1")->image;
      depth = estimate_depth(depth_frame, target.bbox);
      confidence = (depth > 0.0f) ? 0.8f : 0.0f;
    }

    // 方法 2：bbox 面积推算（无需深度图）
    if (depth <= 0.0f && target.bbox[2] > target.bbox[0] && target.bbox[3] > target.bbox[1]) {
      depth = estimate_depth_from_bbox_area(
          target.bbox,
          std::max(fx_, 600.0), std::max(fy_, 600.0),
          real_target_width_, real_target_height_);
      confidence = (depth > 0.0f) ? 0.5f : 0.0f;
    }

    target.depth_confidence = confidence;
    compute_3d_position(target, depth);
  }

  target3d_pub_->publish(targets_3d);
}

float DepthEstimatorNode::estimate_depth(const cv::Mat& depth_frame, const std::array<float, 4>& bbox) {
  float cx_bbox = (bbox[0] + bbox[2]) / 2.0f;
  float cy_bbox = (bbox[1] + bbox[3]) / 2.0f;
  int px = std::clamp(static_cast<int>(cx_bbox), 0, depth_frame.cols - 1);
  int py = std::clamp(static_cast<int>(cy_bbox), 0, depth_frame.rows - 1);
  return depth_frame.at<uint16_t>(py, px) / 1000.0f;
}

float DepthEstimatorNode::estimate_depth_from_bbox_area(
    const std::array<float, 4>& bbox, double fx, double fy,
    double real_width, double real_height) {
  // 针孔模型二维几何平均：
  //   depth_h = (fx * real_width) / bbox_width_px
  //   depth_v = (fy * real_height) / bbox_height_px
  //   depth = sqrt(depth_h * depth_v)
  //         = sqrt((fx * fy * real_width * real_height) / area_px)
  double bbox_width_px = std::max(static_cast<double>(bbox[2] - bbox[0]), 1.0);
  double bbox_height_px = std::max(static_cast<double>(bbox[3] - bbox[1]), 1.0);
  double area_px = bbox_width_px * bbox_height_px;
  double real_area = real_width * real_height;
  double depth = std::sqrt((fx * fy * real_area) / area_px);
  return static_cast<float>(std::clamp(depth, 0.1, 50.0));
}

void DepthEstimatorNode::compute_3d_position(
    vision_servo_msgs::msg::Target& target, float depth)
{
  // 未标定（fx_ == 0）时无法反投影，直接跳过
  if (fx_ == 0 || fy_ == 0 || depth <= 0.0f) return;
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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::DepthEstimatorNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
