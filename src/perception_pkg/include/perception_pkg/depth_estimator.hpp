/**
 * @file depth_estimator.hpp
 * @brief 深度估计节点 — 将 2D 检测框反投影到 3D 空间。
 *
 * 该节点订阅：
 *   - 深度图像（/depth/image_raw）
 *   - 2D 检测结果（detections）
 *
 * 当两路数据均到达时，对每个检测框取其中心点在深度图中对应的深度值，
 * 利用针孔相机模型反投影为相机坐标系下的 3D 位置。
 *
 * 采用双缓存（last_depth_ / last_detections_）策略：
 * 深度图和检测结果以异步频率到达，收到任意一路数据时触发融合计算。
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/mat.hpp>
#include <memory>
#include <string>

namespace perception_pkg {

class DepthEstimatorNode : public rclcpp::Node {
public:
  explicit DepthEstimatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /// 深度图像回调 — 缓存最新深度帧，触发融合估算
  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);
  /// 检测结果回调 — 缓存最新检测结果，触发融合估算
  void detection_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& det_msg);
  /// 实际的深度反投影与发布逻辑（双路数据均到达时执行）
  void estimate_and_publish();

  /**
   * @brief 从深度图中估计指定边界框中心点的深度值。
   * @param depth_frame 深度图（16UC1 格式，单位毫米）
   * @param bbox        边界框 [x_min, y_min, x_max, y_max]，单位像素
   * @return 深度值，单位米
   */
  float estimate_depth(const cv::Mat& depth_frame, const std::array<float, 4>& bbox);

  /**
   * @brief 从 bbox 像素宽度推算深度（无需深度图）。
   *
   * 基于针孔模型推导：depth = (focal_length × real_width) / bbox_width_px
   * 假设目标真实宽度已知（默认 0.3m）。作为深度图不可用时的回退方案。
   *
   * @param bbox        边界框 [x_min, y_min, x_max, y_max]
   * @param fx          相机焦距 (px)
   * @param real_width  目标真实宽度 (m)，默认 0.3
   * @return 估算深度值 (m)
   */
  float estimate_depth_from_bbox_area(
      const std::array<float, 4>& bbox, double fx, double real_width = 0.3);

  /**
   * @brief 根据深度值和图像坐标计算相机坐标系下的 3D 位置。
   *
   * 使用针孔模型反投影公式：
   *   X = (u - cx) / fx * Z
   *   Y = (v - cy) / fy * Z
   *   Z = depth
   *
   * @param target 目标对象（in/out），会填充 position 字段
   * @param depth  深度值，单位米
   */
  void compute_3d_position(vision_servo_msgs::msg::Target& target, float depth);

  // ── 订阅者与发布者 ─────────────────────────────────────────────
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr det_sub_;
  image_transport::Subscriber depth_sub_;
  rclcpp::Publisher<vision_servo_msgs::msg::TargetArray>::SharedPtr target3d_pub_;

  // ── 相机内参（来自 CameraInfo 话题） ────────────────────────────
  double fx_, fy_, cx_, cy_;     ///< 针孔模型内参矩阵参数
  int width_, height_;            ///< 图像分辨率
  std::string camera_frame_;      ///< 相机坐标系名称

  // ── 双缓存（异步融合策略） ──────────────────────────────────────
  sensor_msgs::msg::Image::ConstSharedPtr last_depth_;       ///< 最新深度帧
  vision_servo_msgs::msg::TargetArray::ConstSharedPtr last_detections_; ///< 最新检测结果
};

}  // namespace perception_pkg
