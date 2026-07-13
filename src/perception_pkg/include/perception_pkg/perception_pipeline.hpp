/**
 * @file perception_pipeline.hpp
 * @brief 可组合感知管线 — 检测 → 跟踪 → 深度估计，三阶段零拷贝。
 *
 * 将三个独立节点整合为单个 ComposableNode，在同一个进程内
 * 以零拷贝方式串联。启用 intra-process communication 后，
 * 图像数据在各阶段间以 UniquePtr 移动传递，无需序列化/反序列化。
 *
 * 管线流程：
 *   1. 接收原始图像 → YOLO 推理 → 发布 2D 检测框（TargetArray）
 *   2. 检测框送入卡尔曼跟踪器 → 发布带 ID 的跟踪轨迹
 *   3. 跟踪结果结合深度图 → 发布 3D 目标位姿
 *
 * 同时订阅 camera_info 话题获取内参，完成相机标定初始化。
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>
#include <vision_servo_msgs/srv/set_tracking_target.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/mat.hpp>
#include <memory>
#include <string>
#include <vector>
#include <array>

#include "perception_pkg/detection_node.hpp"
#include "perception_pkg/tracking_node.hpp"
#include "perception_pkg/depth_estimator.hpp"

namespace perception_pkg {

class PerceptionPipeline : public rclcpp::Node {
public:
  explicit PerceptionPipeline(const rclcpp::NodeOptions& options);

private:
  /**
   * @brief 图像回调 — 三阶段管线的主入口。
   *
   * 每帧依次执行：
   *   阶段 1：YOLO 目标检测
   *   阶段 2：多目标跟踪（卡尔曼滤波预测 + 匈牙利关联）
   *   阶段 3：深度估计反投影
   *
   * 三个阶段均为进程内调用（按顺序同步执行），零拷贝。
   */
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  /**
   * @brief 从深度图的边界框中心点估计深度值（需要深度图）。
   */
  float estimate_depth_for_box(const cv::Mat& depth_frame, const std::array<float, 4>& bbox);
  /**
   * @brief 从 bbox 像素宽度推算深度（无需深度图，无 Gazebo 可工作）。
   * depth = (focal_length × real_width) / bbox_width_px
   */
  float estimate_depth_from_bbox_area(
      const std::array<float, 4>& bbox, double fx, double real_width);
  /**
   * @brief 针孔模型反投影：图像坐标 → 相机系 3D 位置。
   */
  void compute_3d_position_target(vision_servo_msgs::msg::Target& target, float depth);

  // ── 管线输入 ────────────────────────────────────────────────────
  image_transport::Subscriber image_sub_;       ///< RGB 图像输入
  image_transport::Subscriber depth_sub_;       ///< 深度图像输入
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_; ///< 相机内参

  // ── 管线输出（各阶段的中间/最终结果） ────────────────────────────
  rclcpp::Publisher<vision_servo_msgs::msg::TargetArray>::SharedPtr det_pub_;     ///< 阶段 1 输出：2D 检测框
  rclcpp::Publisher<vision_servo_msgs::msg::TargetArray>::SharedPtr track_pub_;   ///< 阶段 2 输出：带 ID 的跟踪轨迹
  rclcpp::Publisher<vision_servo_msgs::msg::TargetArray>::SharedPtr target3d_pub_;///< 阶段 3 输出：3D 目标位姿

  // ── 服务 ────────────────────────────────────────────────────────
  rclcpp::Service<vision_servo_msgs::srv::SetTrackingTarget>::SharedPtr tracking_srv_; ///< 设置跟踪目标

  // ── 配置参数 ────────────────────────────────────────────────────
  std::string model_path_;
  float confidence_threshold_;
  float nms_threshold_;
  int tracker_max_age_;
  int tracker_min_hits_;
  std::string camera_frame_;

  // ── 管线组件（进程内） ──────────────────────────────────────────
  std::unique_ptr<MultiObjectTracker> tracker_; ///< 卡尔曼滤波多目标跟踪器
  std::unique_ptr<YoloInference> yolo_;         ///< YOLOv8 推理封装
  int active_tracking_id_;                      ///< 当前活跃的跟踪目标 ID（-1=未选中）

  // ── 相机标定状态 ────────────────────────────────────────────────
  double fx_, fy_, cx_, cy_;  ///< 针孔内参矩阵参数（来自 camera_info）
  bool calibrated_;            ///< 是否已完成标定初始化

  // ── 深度图缓存 ──────────────────────────────────────────────────
  cv::Mat last_depth_frame_;   ///< 最新深度帧缓存
  bool depth_available_ = false;  ///< 是否已收到深度帧
};

}  // namespace perception_pkg
