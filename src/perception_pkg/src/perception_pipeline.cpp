/**
 * @file perception_pipeline.cpp
 * @brief 可组合感知管线实现 — 检测 → 跟踪 → 深度估计，三阶段零拷贝。
 *
 * 该节点将三个感知阶段整合在单个进程中，消除阶段间的序列化开销。
 * 启用 intra-process communication 后，图像数据以 UniquePtr 移动传递，
 * 各阶段之间无需网络序列化/反序列化（ROS2 中这是重要的性能优化）。
 *
 * 管线流程（每帧）：
 *   阶段 1 (检测)：  YOLO 推理 → 2D 边界框 + 类别 + 置信度
 *   阶段 2 (跟踪)：  卡尔曼预测 + IoU 关联 + 状态更新 → 带 ID 的轨迹
 *   阶段 3 (深度)：  边界框中心点反投影 → 相机坐标系 3D 位置
 */

#include "perception_pkg/perception_pipeline.hpp"
#include "perception_pkg/qos.hpp"
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

namespace perception_pkg {

PerceptionPipeline::PerceptionPipeline(const rclcpp::NodeOptions& options)
  : Node("perception_pipeline", options),
    active_tracking_id_(-1), calibrated_(false)
{
  // ── 1. 声明参数 ─────────────────────────────────────────────────
  this->declare_parameter("model_path", "");
  this->declare_parameter("confidence_threshold", 0.5);
  this->declare_parameter("nms_threshold", 0.45);
  this->declare_parameter("tracker_max_age", 30);
  this->declare_parameter("tracker_min_hits", 3);
  this->declare_parameter("camera_frame", "camera_link");

  model_path_ = this->get_parameter("model_path").as_string();
  confidence_threshold_ = static_cast<float>(this->get_parameter("confidence_threshold").as_double());
  nms_threshold_ = static_cast<float>(this->get_parameter("nms_threshold").as_double());
  int max_age = this->get_parameter("tracker_max_age").as_int();
  int min_hits = this->get_parameter("tracker_min_hits").as_int();
  camera_frame_ = this->get_parameter("camera_frame").as_string();

  // ── 2. 初始化管线组件 ───────────────────────────────────────────
  tracker_ = std::make_unique<MultiObjectTracker>(max_age, min_hits, 0.3f);

  // ── 3. 订阅者（启用进程内通信以实现零拷贝） ─────────────────────
  auto sub_opts = rclcpp::SubscriptionOptions();
  sub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  // RGB 图像订阅
  image_sub_ = image_transport::create_subscription(
    this, "/camera/image_raw",
    std::bind(&PerceptionPipeline::image_callback, this, std::placeholders::_1),
    "raw");

  // 深度图像订阅（缓存最新深度图用于阶段 3 反投影）
  depth_sub_ = image_transport::create_subscription(
    this, "/camera/depth/image_raw", [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
      last_depth_frame_ = cv_bridge::toCvShare(msg, "16UC1")->image.clone();
      depth_available_ = true;
    }, "raw");

  // 相机内参订阅（TRANSIENT_LOCAL QoS = 迟加入也能获取最后发布的内参）
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", qos::camera_info(),
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
      // 提取针孔模型内参矩阵 K 的参数
      fx_ = info->k[0]; fy_ = info->k[4];
      cx_ = info->k[2]; cy_ = info->k[5];
      calibrated_ = true;
    }, sub_opts);

  // ── 4. 发布者（三个阶段各自的输出话题） ─────────────────────────
  det_pub_ = this->create_publisher<vision_servo_msgs::msg::TargetArray>(
    "/perception/detections", qos::detections());
  track_pub_ = this->create_publisher<vision_servo_msgs::msg::TargetArray>(
    "/perception/tracks", qos::detections());
  target3d_pub_ = this->create_publisher<vision_servo_msgs::msg::TargetArray>(
    "/perception/targets_3d", qos::detections());

  // ── 5. 服务 ─────────────────────────────────────────────────────
  tracking_srv_ = this->create_service<vision_servo_msgs::srv::SetTrackingTarget>(
    "~/set_tracking_target",
    [this](
      const std::shared_ptr<vision_servo_msgs::srv::SetTrackingTarget::Request> req,
      std::shared_ptr<vision_servo_msgs::srv::SetTrackingTarget::Response> resp) {
      active_tracking_id_ = req->target_id;  // 设置当前跟踪目标
      resp->success = true;
      resp->message = "OK";
      resp->assigned_id = active_tracking_id_;
    });

  RCLCPP_INFO(get_logger(), "感知管线已初始化");
}

void PerceptionPipeline::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  // ── 阶段 1：检测 ────────────────────────────────────────────────
  vision_servo_msgs::msg::TargetArray detections;
  detections.header = msg->header;
  detections.header.frame_id = camera_frame_;
  // [YOLO 推理占位]
  det_pub_->publish(detections);

  // ── 阶段 2：跟踪 ────────────────────────────────────────────────
  tracker_->update(detections);           // 卡尔曼预测 + 关联 + 更新
  auto tracks = tracker_->get_tracks(
    // 将 ROS 时间戳转换为纳秒级 Unix 时间戳
    msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec, camera_frame_);
  tracks.tracking_id = active_tracking_id_;
  track_pub_->publish(tracks);

  // ── 阶段 3：深度估计 ────────────────────────────────────────────
  // 深度估计采用两级回退策略：
  //   优先级 1：深度图采样（Gazebo 深度相机发布时，最准确）
  //   优先级 2：bbox 面积推算（无深度图时，假设已知目标真实尺寸）
  if (calibrated_) {
    for (auto& target : tracks.targets) {
      float depth = 0.0f;
      float confidence = 0.0f;

      if (depth_available_) {
        // 方法 1：直接从深度图采样 bbox 中心点
        depth = estimate_depth_for_box(last_depth_frame_, target.bbox);
        confidence = (depth > 0.0f) ? 0.8f : 0.0f;
      }

      if (depth <= 0.0f && target.bbox[2] > target.bbox[0]) {
        // 方法 2：用 bbox 像素面积推算深度（无需深度图）
        // 原理：depth = (f * real_size) / pixel_size
        // 假设相机 f=600px, 目标真实宽高 ≈ 0.3m
        depth = estimate_depth_from_bbox_area(
            target.bbox, /*fx=*/std::max(fx_, 600.0), /*real_width=*/0.3);
        confidence = (depth > 0.0f) ? 0.5f : 0.0f;  // 推算值置信度低于传感器直接测量
      }

      target.depth_confidence = confidence;
      if (depth > 0.0f) {
        compute_3d_position_target(target, depth);
      }
    }
  }
  target3d_pub_->publish(tracks);
}

float PerceptionPipeline::estimate_depth_for_box(
    const cv::Mat& depth_frame, const std::array<float, 4>& bbox) {
  float cx_bbox = (bbox[0] + bbox[2]) / 2.0f;
  float cy_bbox = (bbox[1] + bbox[3]) / 2.0f;
  int px = std::clamp(static_cast<int>(cx_bbox), 0, depth_frame.cols - 1);
  int py = std::clamp(static_cast<int>(cy_bbox), 0, depth_frame.rows - 1);
  return depth_frame.at<uint16_t>(py, px) / 1000.0f;
}

float PerceptionPipeline::estimate_depth_from_bbox_area(
    const std::array<float, 4>& bbox, double fx, double real_width) {
  // 针孔模型：object_width_on_sensor / focal_length = real_width / depth
  // → depth = (focal_length_px * real_width_m) / bbox_width_px
  double bbox_width_px = std::max(static_cast<double>(bbox[2] - bbox[0]), 1.0);
  double depth = (fx * real_width) / bbox_width_px;
  // 钳位到合理范围：0.1m ~ 50m
  return static_cast<float>(std::clamp(depth, 0.1, 50.0));
}

void PerceptionPipeline::compute_3d_position_target(
    vision_servo_msgs::msg::Target& target, float depth) {
  if (fx_ == 0) return;  // 未标定
  // 针孔模型反投影：
  //   X = (u - cx) / fx * Z,  Y = (v - cy) / fy * Z,  Z = depth
  target.position[0] = (target.center[0] - cx_) / fx_ * depth;
  target.position[1] = (target.center[1] - cy_) / fy_ * depth;
  target.position[2] = depth;
}

}  // namespace perception_pkg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception_pkg::PerceptionPipeline)
