/**
 * @file tracking_node.hpp
 * @brief 多目标跟踪 — ByteTrack 默认实现与 legacy IoU 回退实现。
 *
 * 默认核心算法（ByteTracker）：
 *   1. 预测：XYAH 卡尔曼预测，可叠加稀疏光流全局相机运动补偿
 *   2. 分状态关联：CONFIRMED 和 LOST 使用不同门限及复合几何代价
 *   3. 低分恢复：未匹配活跃轨迹与低置信度检测二次匹配
 *   4. 管理：匿名候选延迟分配 ID、确认、丢失、删除与重复轨迹清理
 *
 * TrackingNode 将跟踪器封装为 ROS2 节点，提供：
 *   - 检测结果订阅 → 跟踪轨迹发布
 *   - SetTrackingTarget 服务（设置/切换跟踪目标）
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>
#include <vision_servo_msgs/srv/set_tracking_target.hpp>

#include "perception_pkg/camera_motion_estimator.hpp"
#include "perception_pkg/target_selector.hpp"
#include "perception_pkg/tracker_interface.hpp"

namespace perception_pkg {

/**
 * @class MultiObjectTracker
 * @brief 原有单阶段 IoU 跟踪器，仅用于对比和回退。
 *
 * 使用恒速运动模型的 8 维卡尔曼滤波器：
 *   状态向量 [x, y, w, h, vx, vy, vw, vh]
 *   观测向量 [x, y, w, h]（由检测器提供）
 *
 * 轨迹在连续 max_age 帧未被检测到时被删除；
 * 检测结果需连续命中 min_hits 帧后才被认为是稳定轨迹（防止假阳性）。
 */
class MultiObjectTracker final : public TrackerInterface {
public:
  /// 单条跟踪轨迹的内部表示
  struct Track {
    int id = -1;                      ///< 轨迹唯一标识
    std::string class_name;           ///< 目标类别
    cv::KalmanFilter kf;              ///< 卡尔曼滤波器（8 维状态，4 维观测）
    int age = 0;                       ///< 自最近一次检测以来的帧数
    int consecutive_visible_count = 0;///< 连续可见帧数
    int consecutive_invisible_count = 0; ///< 连续不可见帧数
    float confidence = 0.0F;          ///< 最近一次匹配检测的置信度
    bool confirmed = false;           ///< 是否曾达到连续 min_hits 次命中
    bool visible_in_current_frame = false; ///< 当前帧是否匹配到真实检测
  };

  explicit MultiObjectTracker(int max_age = 30, int min_hits = 3, float iou_threshold = 0.3);

  /// 对所有活跃轨迹执行卡尔曼预测步（在关联检测结果前调用）
  void predict(float dt_seconds);
  /// 用当前帧检测结果更新跟踪器状态（预测 + 关联 + 更新 + 管理）
  void update(const vision_servo_msgs::msg::TargetArray& detections) override;
  /// 获取所有已确认的跟踪轨迹（包括 max_age 窗口内的 LOST 轨迹）
  vision_servo_msgs::msg::TargetArray get_tracks(
      uint64_t timestamp, const std::string& frame_id) override;
  const char* name() const noexcept override { return "legacy_iou"; }

private:
  /// 计算两个目标边界框的 IoU（交并比）
  float compute_iou(const vision_servo_msgs::msg::Target& a,
                    const vision_servo_msgs::msg::Target& b);
  /// 执行检测结果与轨迹的全局关联（基于 IoU + Hungarian）
  void associate_detections_to_tracks(
    const vision_servo_msgs::msg::TargetArray& detections,
    std::map<int, int>& matches,
    std::set<int>& unmatched_detections,
    std::set<int>& unmatched_tracks);

  std::map<int, Track> tracks_;  ///< 所有活跃轨迹
  int next_id_;                   ///< 下一个分配的轨迹 ID
  int max_age_;                   ///< 轨迹最大存活帧数
  int min_hits_;                  ///< 轨迹确认所需最小命中帧数
  float iou_threshold_;           ///< 关联 IoU 阈值
  uint64_t last_timestamp_ns_ = 0; ///< 上一帧时间戳，用于时间尺度正确的预测
};

// ── ROS2 节点封装 ──────────────────────────────────────────────────

class TrackingNode : public rclcpp::Node {
public:
  explicit TrackingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /// 检测结果回调 — 驱动跟踪器更新并发布跟踪轨迹
  void detection_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg);
  /// RGB image callback used only for optional global camera-motion estimation.
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper& status);
  void record_latency(double elapsed_ms);
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr det_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_servo_msgs::msg::TargetArray>::SharedPtr track_pub_;
  rclcpp::Service<vision_servo_msgs::srv::SetTrackingTarget>::SharedPtr tracking_srv_;

  std::unique_ptr<TrackerInterface> tracker_; ///< Startup-selected 2D tracker
  std::unique_ptr<CameraMotionEstimator> camera_motion_estimator_;
  TargetSelector target_selector_;    ///< 目标锁定状态机
  std::string tracker_type_;          ///< Diagnostics and startup log name
  std::string camera_frame_;          ///< 相机坐标系名称
  mutable std::mutex state_mutex_;    ///< 保护服务与检测回调共享的目标选择状态
  diagnostic_updater::Updater diagnostics_;
  double input_timeout_seconds_ = 2.0;
  double performance_log_period_ = 5.0;
  bool enable_camera_motion_compensation_ = false;
  uint64_t previous_detection_timestamp_ns_ = 0;
  std::atomic<uint64_t> camera_motion_frames_{0};
  std::atomic<uint64_t> valid_camera_motion_frames_{0};
  std::atomic<uint64_t> received_frames_{0};
  std::atomic<int64_t> last_input_steady_ns_{0};
  std::chrono::steady_clock::time_point stats_window_start_;
  uint64_t stats_frame_count_ = 0;
  double stats_total_latency_ms_ = 0.0;
  double stats_max_latency_ms_ = 0.0;
  std::vector<double> stats_latency_samples_ms_;
  double last_latency_ms_ = -1.0;
};

}  // namespace perception_pkg
