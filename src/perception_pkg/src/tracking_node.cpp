/**
 * @file tracking_node.cpp
 * @brief 多目标跟踪节点实现 — 基于卡尔曼滤波的 SORT/ByteTrack 风格跟踪器。
 *
 * MultiObjectTracker 核心流程：
 *   1. predict()：对每条轨迹执行卡尔曼预测，更新先验状态估计
 *   2. update()：  与当前检测结果进行 IoU 关联匹配
 *   3. 更新已匹配轨迹的卡尔曼状态（后验校正）
 *   4. 为未匹配检测创建新轨迹
 *   5. 移除超龄（consecutive_invisible_count > max_age）的死轨迹
 *
 * 卡尔曼滤波器配置：
 *   - 8 维状态：[x, y, w, h, vx, vy, vw, vh]（位置 + 速度）
 *   - 4 维观测：[x, y, w, h]（来自检测器）
 *   - 恒速运动模型（transitionMatrix 中对角位置-速度耦合）
 *
 * TrackingNode 是 MultiObjectTracker 的 ROS2 节点封装，
 * 提供检测订阅、轨迹发布和 SetTrackingTarget 服务。
 */

#include "perception_pkg/tracking_node.hpp"
#include "perception_pkg/qos.hpp"

namespace perception_pkg {

// ── MultiObjectTracker 实现 ────────────────────────────────────────

MultiObjectTracker::MultiObjectTracker(int max_age, int min_hits, float iou_threshold)
  : next_id_(0), max_age_(max_age), min_hits_(min_hits), iou_threshold_(iou_threshold)
{}

void MultiObjectTracker::predict() {
  for (auto& [id, track] : tracks_) {
    track.kf.predict();  // 卡尔曼预测步：先验估计 x̂ₖ|ₖ₋₁
    track.age_++;         // 递增未更新计数（关联成功后会被重置）
  }
}

void MultiObjectTracker::update(const vision_servo_msgs::msg::TargetArray& detections) {
  // 步骤 1：对所有轨迹执行预测
  predict();

  // 步骤 2：关联检测结果与轨迹（基于 IoU 的贪心匹配）
  std::map<int, int> matches;
  std::set<int> unmatched_detections, unmatched_tracks;
  associate_detections_to_tracks(detections, matches, unmatched_detections, unmatched_tracks);

  // 步骤 3：更新已匹配的轨迹（卡尔曼校正 + 重置不可见计数）
  for (auto& [track_idx, det_idx] : matches) {
    auto& det = detections.targets[det_idx];
    auto& track = tracks_[track_idx];
    // 用检测器输出构造 4 维观测向量 [x, y, w, h]
    cv::Mat measurement = (cv::Mat_<float>(4, 1)
      << det.center[0], det.center[1],
         det.bbox[2] - det.bbox[0], det.bbox[3] - det.bbox[1]);
    track.kf.correct(measurement);  // 卡尔曼校正步：后验估计 x̂ₖ|ₖ
    // 重置老化计数器（轨迹被确认存活）
    track.age_ = 0;
    track.consecutive_invisible_count = 0;
    track.total_visible_count++;
    // 更新历史最佳置信度和类别
    if (det.confidence > track.max_confidence) {
      track.max_confidence = det.confidence;
      track.class_name = det.class_name;
    }
  }

  // 步骤 4：为未匹配的检测结果创建新轨迹
  for (int det_idx : unmatched_detections) {
    auto& det = detections.targets[det_idx];
    Track new_track;
    new_track.id = next_id_++;
    new_track.class_name = det.class_name;
    new_track.age_ = 0;
    new_track.total_visible_count = 1;
    new_track.consecutive_invisible_count = 0;
    new_track.max_confidence = det.confidence;

    // 初始化 8 维状态、4 维观测的卡尔曼滤波器
    new_track.kf.init(8, 4, 0);
    // 恒速运动模型的状态转移矩阵（dt=1）：位置 = 前一位置 + 速度
    new_track.kf.transitionMatrix = (cv::Mat_<float>(8, 8) <<
      1,0,0,0,1,0,0,0, 0,1,0,0,0,1,0,0, 0,0,1,0,0,0,1,0, 0,0,0,1,0,0,0,1,
      0,0,0,0,1,0,0,0, 0,0,0,0,0,1,0,0, 0,0,0,0,0,0,1,0, 0,0,0,0,0,0,0,1);
    // 观测矩阵：直接观测位置分量（忽略速度分量）
    new_track.kf.measurementMatrix = cv::Mat::eye(4, 8, CV_32F);
    // TODO：根据检测结果设置卡尔曼滤波器的初始状态
    tracks_[new_track.id] = new_track;
  }

  // 步骤 5：移除超过最大存活帧数的死轨迹
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (it->second.consecutive_invisible_count > max_age_) {
      it = tracks_.erase(it);  // 轨迹已死，清理资源
    } else {
      ++it;
    }
  }
}

vision_servo_msgs::msg::TargetArray MultiObjectTracker::get_tracks(
    uint64_t timestamp, const std::string& frame_id)
{
  vision_servo_msgs::msg::TargetArray result;
  // 仅返回已确认的轨迹（可见次数达到 min_hits_ 阈值）
  // 这可以过滤掉短暂的虚假检测
  for (auto& [id, track] : tracks_) {
    if (track.total_visible_count >= min_hits_) {
      vision_servo_msgs::msg::Target t;
      t.id = id;
      t.class_name = track.class_name;
      t.confidence = track.max_confidence;
      // TODO：从卡尔曼状态估计填充中心点和边界框
      result.targets.push_back(t);
    }
  }
  return result;
}

float MultiObjectTracker::compute_iou(
    const vision_servo_msgs::msg::Target& a,
    const vision_servo_msgs::msg::Target& b)
{
  // 计算两个矩形框的交集区域
  float x1 = std::max(a.bbox[0], b.bbox[0]);
  float y1 = std::max(a.bbox[1], b.bbox[1]);
  float x2 = std::min(a.bbox[2], b.bbox[2]);
  float y2 = std::min(a.bbox[3], b.bbox[3]);
  float intersection = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);

  // 计算并集面积 = A面积 + B面积 - 交集面积
  float area_a = (a.bbox[2] - a.bbox[0]) * (a.bbox[3] - a.bbox[1]);
  float area_b = (b.bbox[2] - b.bbox[0]) * (b.bbox[3] - b.bbox[1]);

  // 分母加 1e-6 防止除零
  return intersection / (area_a + area_b - intersection + 1e-6f);
}

void MultiObjectTracker::associate_detections_to_tracks(
    const vision_servo_msgs::msg::TargetArray& detections,
    std::map<int, int>& matches,
    std::set<int>& unmatched_detections,
    std::set<int>& unmatched_tracks)
{
  // 贪心匈牙利式关联：对每条轨迹寻找 IoU 最大的检测结果
  // 当前为简化实现——基于阈值匹配。实际部署时可替换为
  // 完整的匈牙利算法（或 lapjv）以获得全局最优匹配。

  // 初始时所有检测和轨迹均未匹配
  for (size_t d = 0; d < detections.targets.size(); ++d) {
    unmatched_detections.insert(static_cast<int>(d));
  }
  for (auto& [id, track] : tracks_) {
    unmatched_tracks.insert(id);
  }

  // 对每条未匹配轨迹，寻找最佳 IoU 检测匹配
  for (int tid : unmatched_tracks) {
    float best_iou = iou_threshold_;
    int best_det = -1;
    for (int did : unmatched_detections) {
      float iou = compute_iou(detections.targets[did],
        // TODO：从轨迹的卡尔曼状态构造边界框用于 IoU 计算
        vision_servo_msgs::msg::Target{});
      if (iou > best_iou) { best_iou = iou; best_det = did; }
    }
    // 找到高于阈值的匹配 → 从未匹配集合中移除
    if (best_det >= 0) {
      matches[tid] = best_det;
      unmatched_detections.erase(best_det);
      unmatched_tracks.erase(tid);
    }
  }
}

// ── TrackingNode 实现 ──────────────────────────────────────────────

TrackingNode::TrackingNode(const rclcpp::NodeOptions& options)
  : Node("tracking_node", options),
    tracker_(/*max_age=*/30, /*min_hits=*/3, /*iou_threshold=*/0.3),
    active_tracking_id_(-1),
    tracking_enabled_(true)
{
  // ── 声明参数 ────────────────────────────────────────────────────
  this->declare_parameter("max_age", 30);
  this->declare_parameter("min_hits", 3);
  this->declare_parameter("iou_threshold", 0.3);
  this->declare_parameter("camera_frame", "camera_link");

  camera_frame_ = this->get_parameter("camera_frame").as_string();
  int max_age = this->get_parameter("max_age").as_int();
  int min_hits = this->get_parameter("min_hits").as_int();
  float iou_threshold = static_cast<float>(this->get_parameter("iou_threshold").as_double());
  // 使用参数重新构造跟踪器（覆盖成员初始化列表中的默认值）
  tracker_ = MultiObjectTracker(max_age, min_hits, iou_threshold);

  // ── 订阅者：检测结果 ─────────────────────────────────────────────
  det_sub_ = this->create_subscription<vision_servo_msgs::msg::TargetArray>(
    "detections", qos::detections(),
    std::bind(&TrackingNode::detection_callback, this, std::placeholders::_1));

  // ── 发布者：跟踪轨迹 ─────────────────────────────────────────────
  track_pub_ = this->create_publisher<vision_servo_msgs::msg::TargetArray>(
    "~/tracks", qos::detections());

  // ── 服务：设置跟踪目标 ───────────────────────────────────────────
  tracking_srv_ = this->create_service<vision_servo_msgs::srv::SetTrackingTarget>(
    "~/set_tracking_target",
    [this](const auto& req, auto& resp) {
      set_tracking_target(req->target_id, req->class_name, req->enable);
      resp->success = true;
      resp->message = "OK";
      resp->assigned_id = active_tracking_id_;
    });

  RCLCPP_INFO(get_logger(), "跟踪节点已启动 (max_age=%d, min_hits=%d)",
              max_age, min_hits);
}

void TrackingNode::detection_callback(
    const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg)
{
  // 驱动跟踪器更新（预测 + 关联 + 管理）
  tracker_.update(*msg);
  // 获取所有已确认的跟踪轨迹
  auto tracks = tracker_.get_tracks(msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec,
                                    camera_frame_);

  // 自动选择跟踪目标：若未手动指定且存在轨迹，选第一个（或置信度最高的）
  if (tracking_enabled_ && active_tracking_id_ < 0 && !tracks.targets.empty()) {
    active_tracking_id_ = tracks.targets[0].id;
  }
  tracks.tracking_id = active_tracking_id_;
  tracks.header = msg->header;
  track_pub_->publish(tracks);
}

void TrackingNode::set_tracking_target(int target_id, const std::string& class_name, bool enable) {
  tracking_enabled_ = enable;
  active_tracking_id_ = target_id;
  target_class_filter_ = class_name;
  RCLCPP_INFO(get_logger(), "设置跟踪目标: id=%d, class=%s, enable=%d",
              target_id, class_name.c_str(), enable);
}

}  // namespace perception_pkg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception_pkg::TrackingNode)
