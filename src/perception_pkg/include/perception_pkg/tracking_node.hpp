/**
 * @file tracking_node.hpp
 * @brief 多目标跟踪 — 基于卡尔曼滤波的 SORT/ByteTrack 风格跟踪器。
 *
 * 核心算法（MultiObjectTracker）：
 *   1. 预测：对所有活跃轨迹执行卡尔曼滤波预测步
 *   2. 关联：通过 IoU 与当前检测结果进行匈牙利式贪心匹配
 *   3. 更新：用匹配到的检测结果校正卡尔曼滤波器状态
 *   4. 管理：为新检测创建轨迹，清除超龄轨迹
 *
 * TrackingNode 将跟踪器封装为 ROS2 节点，提供：
 *   - 检测结果订阅 → 跟踪轨迹发布
 *   - SetTrackingTarget 服务（设置/切换跟踪目标）
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>
#include <memory>
#include <deque>
#include <map>

namespace perception_pkg {

/**
 * @class MultiObjectTracker
 * @brief 基于卡尔曼滤波的多目标跟踪器（SORT/ByteTrack 风格）。
 *
 * 使用恒速运动模型的 8 维卡尔曼滤波器：
 *   状态向量 [x, y, w, h, vx, vy, vw, vh]
 *   观测向量 [x, y, w, h]（由检测器提供）
 *
 * 轨迹在连续 max_age 帧未被检测到时被删除；
 * 检测结果需连续命中 min_hits 帧后才被认为是稳定轨迹（防止假阳性）。
 */
class MultiObjectTracker {
public:
  /// 单条跟踪轨迹的内部表示
  struct Track {
    int id;                           ///< 轨迹唯一标识
    std::string class_name;           ///< 目标类别
    cv::KalmanFilter kf;              ///< 卡尔曼滤波器（8 维状态，4 维观测）
    int age;                          ///< 自最近一次检测以来的帧数
    int total_visible_count;          ///< 累计可见帧数
    int consecutive_invisible_count;  ///< 连续不可见帧数
    float max_confidence;             ///< 历史最高置信度
  };

  explicit MultiObjectTracker(int max_age = 30, int min_hits = 3, float iou_threshold = 0.3);

  /// 对所有活跃轨迹执行卡尔曼预测步（在关联检测结果前调用）
  void predict();
  /// 用当前帧检测结果更新跟踪器状态（预测 + 关联 + 更新 + 管理）
  void update(const vision_servo_msgs::msg::TargetArray& detections);
  /// 获取所有已确认的跟踪轨迹（total_visible_count >= min_hits_）
  vision_servo_msgs::msg::TargetArray get_tracks(uint64_t timestamp, const std::string& frame_id);

private:
  /// 计算两个目标边界框的 IoU（交并比）
  float compute_iou(const vision_servo_msgs::msg::Target& a,
                    const vision_servo_msgs::msg::Target& b);
  /// 执行检测结果与轨迹的贪心关联（基于 IoU）
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
};

// ── ROS2 节点封装 ──────────────────────────────────────────────────

class TrackingNode : public rclcpp::Node {
public:
  explicit TrackingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /// 检测结果回调 — 驱动跟踪器更新并发布跟踪轨迹
  void detection_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg);
  /// 设置当前跟踪目标（由 SetTrackingTarget 服务触发）
  void set_tracking_target(int target_id, const std::string& class_name, bool enable);

  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr det_sub_;
  rclcpp::Publisher<vision_servo_msgs::msg::TargetArray>::SharedPtr track_pub_;
  rclcpp::Service<vision_servo_msgs::srv::SetTrackingTarget>::SharedPtr tracking_srv_;

  MultiObjectTracker tracker_;       ///< 卡尔曼滤波跟踪器实例
  int active_tracking_id_;            ///< 当前活跃跟踪目标 ID（-1=未选中）
  std::string target_class_filter_;   ///< 目标类别过滤（空=不过滤）
  bool tracking_enabled_;             ///< 是否启用跟踪
  std::string camera_frame_;          ///< 相机坐标系名称
};

}  // namespace perception_pkg
