/**
 * @file tracking_node.cpp
 * @brief 启动时选择 ByteTrack 或 legacy IoU 实现的 ROS 2 跟踪节点。
 */

#include "perception_pkg/tracking_node.hpp"
#include "perception_pkg/byte_tracker.hpp"
#include "perception_pkg/qos.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace perception_pkg {
namespace {

rcl_interfaces::msg::ParameterDescriptor descriptor(const std::string& description)
{
  rcl_interfaces::msg::ParameterDescriptor result;
  result.description = description;
  result.read_only = true;
  return result;
}

rcl_interfaces::msg::ParameterDescriptor integer_descriptor(
    const std::string& description, int64_t minimum, int64_t maximum)
{
  auto result = descriptor(description);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = minimum;
  range.to_value = maximum;
  range.step = 1;
  result.integer_range.push_back(range);
  return result;
}

rcl_interfaces::msg::ParameterDescriptor floating_descriptor(
    const std::string& description, double minimum, double maximum)
{
  auto result = descriptor(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = minimum;
  range.to_value = maximum;
  result.floating_point_range.push_back(range);
  return result;
}

double percentile_95(std::vector<double> samples)
{
  if (samples.empty()) {
    return -1.0;
  }
  std::sort(samples.begin(), samples.end());
  const size_t index = static_cast<size_t>(
      std::ceil(0.95 * static_cast<double>(samples.size()))) - 1;
  return samples[std::min(index, samples.size() - 1)];
}

}  // namespace

TrackingNode::TrackingNode(const rclcpp::NodeOptions& options)
  : Node("tracking_node", options),
    target_selector_(true, "person"),
    diagnostics_(this)
{
  this->declare_parameter("tracker_type", "bytetrack",
      descriptor("Startup tracker implementation: bytetrack or legacy_iou."));
  // Legacy tracker parameters are kept for A/B comparison and rollback.
  this->declare_parameter("max_age", 30,
      integer_descriptor("Legacy tracker missed frames retained as LOST.", 0, 10000));
  this->declare_parameter("min_hits", 3,
      integer_descriptor("Legacy tracker consecutive hits required for confirmation.", 1, 10000));
  this->declare_parameter("iou_threshold", 0.3,
      floating_descriptor("Legacy tracker minimum association IoU.", 0.0, 1.0));

  this->declare_parameter("track_high_threshold", 0.50,
      floating_descriptor("ByteTrack high-confidence split threshold.", 0.0, 1.0));
  this->declare_parameter("track_low_threshold", 0.10,
      floating_descriptor("ByteTrack lowest score accepted for recovery matching.", 0.0, 1.0));
  this->declare_parameter("new_track_threshold", 0.60,
      floating_descriptor("Minimum score allowed to create a new track.", 0.0, 1.0));
  this->declare_parameter("first_match_min_iou", 0.30,
      floating_descriptor("Minimum IoU for primary high-score association.", 0.0, 1.0));
  this->declare_parameter("second_match_min_iou", 0.50,
      floating_descriptor("Minimum IoU for low-score recovery association.", 0.0, 1.0));
  this->declare_parameter("unconfirmed_match_min_iou", 0.30,
      floating_descriptor("Minimum IoU used to confirm tentative tracks.", 0.0, 1.0));
  this->declare_parameter("duplicate_iou_threshold", 0.85,
      floating_descriptor("IoU at which confirmed/lost tracks are considered duplicates.", 0.0, 1.0));
  this->declare_parameter("lost_timeout_seconds", 1.0,
      floating_descriptor("Seconds a confirmed track remains recoverable as LOST.", 0.0, 3600.0));
  this->declare_parameter("min_confirm_hits", 3,
      integer_descriptor("Consecutive high-confidence hits required for confirmation.", 1, 10000));
  this->declare_parameter("fuse_detection_score", true,
      descriptor("Fuse high-box confidence into the primary association cost."));
  this->declare_parameter("publish_tentative_tracks", true,
      descriptor("Publish tentative tracks for observation without target selection."));
  this->declare_parameter("camera_frame", "",
      descriptor("Optional frame override; empty preserves the detector frame."));
  this->declare_parameter("auto_select", true,
      descriptor("Automatically select the best confirmed target when none is locked."));
  this->declare_parameter("class_filter", "person",
      descriptor("Class used for automatic target selection; empty accepts every class."));
  this->declare_parameter("input_timeout_seconds", 2.0,
      floating_descriptor("Warn after this many seconds without detections messages.", 0.1, 3600.0));
  this->declare_parameter("performance_log_period", 5.0,
      floating_descriptor("Seconds between tracking latency reports; 0 disables logs.", 0.0, 3600.0));

  camera_frame_ = this->get_parameter("camera_frame").as_string();
  tracker_type_ = this->get_parameter("tracker_type").as_string();
  std::transform(
      tracker_type_.begin(), tracker_type_.end(), tracker_type_.begin(),
      [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
  if (tracker_type_ == "bytetrack") {
    ByteTrackerConfig config;
    config.track_high_threshold = static_cast<float>(
        this->get_parameter("track_high_threshold").as_double());
    config.track_low_threshold = static_cast<float>(
        this->get_parameter("track_low_threshold").as_double());
    config.new_track_threshold = static_cast<float>(
        this->get_parameter("new_track_threshold").as_double());
    config.first_match_min_iou = static_cast<float>(
        this->get_parameter("first_match_min_iou").as_double());
    config.second_match_min_iou = static_cast<float>(
        this->get_parameter("second_match_min_iou").as_double());
    config.unconfirmed_match_min_iou = static_cast<float>(
        this->get_parameter("unconfirmed_match_min_iou").as_double());
    config.duplicate_iou_threshold = static_cast<float>(
        this->get_parameter("duplicate_iou_threshold").as_double());
    config.lost_timeout_seconds =
        this->get_parameter("lost_timeout_seconds").as_double();
    config.min_confirm_hits = this->get_parameter("min_confirm_hits").as_int();
    config.fuse_detection_score =
        this->get_parameter("fuse_detection_score").as_bool();
    config.publish_tentative_tracks =
        this->get_parameter("publish_tentative_tracks").as_bool();
    tracker_ = std::make_unique<ByteTracker>(config);
  } else if (tracker_type_ == "legacy_iou") {
    tracker_ = std::make_unique<MultiObjectTracker>(
        this->get_parameter("max_age").as_int(),
        this->get_parameter("min_hits").as_int(),
        static_cast<float>(this->get_parameter("iou_threshold").as_double()));
  } else {
    throw std::invalid_argument("tracker_type must be bytetrack or legacy_iou");
  }
  const bool auto_select = this->get_parameter("auto_select").as_bool();
  const std::string target_class_filter = this->get_parameter("class_filter").as_string();
  input_timeout_seconds_ = this->get_parameter("input_timeout_seconds").as_double();
  performance_log_period_ = this->get_parameter("performance_log_period").as_double();
  target_selector_ = TargetSelector(auto_select, target_class_filter);

  det_sub_ = this->create_subscription<vision_servo_msgs::msg::TargetArray>(
    "detections", qos::perception(),
    std::bind(&TrackingNode::detection_callback, this, std::placeholders::_1));

  track_pub_ = this->create_publisher<vision_servo_msgs::msg::TargetArray>(
    "tracks", qos::perception());

  tracking_srv_ = this->create_service<vision_servo_msgs::srv::SetTrackingTarget>(
    "~/set_tracking_target",
    [this](
      const std::shared_ptr<vision_servo_msgs::srv::SetTrackingTarget::Request> req,
      std::shared_ptr<vision_servo_msgs::srv::SetTrackingTarget::Response> resp) {
      std::string message;
      std::lock_guard<std::mutex> lock(state_mutex_);
      resp->success = target_selector_.request(
          req->target_id, req->class_name, req->enable, message);
      resp->message = message;
      resp->assigned_id = target_selector_.active_id();
    });

  diagnostics_.setHardwareID(tracker_type_);
  diagnostics_.add("input_and_latency", this, &TrackingNode::diagnostic_callback);
  stats_window_start_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(
      get_logger(), "跟踪节点已启动 (tracker=%s)", tracker_->name());
}

void TrackingNode::detection_callback(
    const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg)
{
  received_frames_.fetch_add(1, std::memory_order_relaxed);
  last_input_steady_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count(),
      std::memory_order_relaxed);
  tracker_->update(*msg);
  const std::string output_frame = camera_frame_.empty() ? msg->header.frame_id : camera_frame_;
  const uint64_t timestamp_ns = msg->header.stamp.sec >= 0
      ? static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(msg->header.stamp.nanosec)
      : 0;
  auto tracks = tracker_->get_tracks(
      timestamp_ns, output_frame);

  tracks.header = msg->header;
  if (!camera_frame_.empty()) {
    tracks.header.frame_id = camera_frame_;
  }
  for (auto& target : tracks.targets) {
    target.header = tracks.header;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    tracks.tracking_id = target_selector_.update(tracks);
  }
  track_pub_->publish(tracks);

  double latency_ms = -1.0;
  const rclcpp::Time input_stamp(msg->header.stamp, get_clock()->get_clock_type());
  const rclcpp::Time current_time = now();
  if (input_stamp.nanoseconds() > 0 && current_time >= input_stamp) {
    latency_ms = static_cast<double>((current_time - input_stamp).nanoseconds()) * 1.0e-6;
  }
  record_latency(latency_ms);
}

void TrackingNode::record_latency(double elapsed_ms)
{
  if (elapsed_ms >= 0.0) {
    last_latency_ms_ = elapsed_ms;
    ++stats_frame_count_;
    stats_total_latency_ms_ += elapsed_ms;
    stats_max_latency_ms_ = std::max(stats_max_latency_ms_, elapsed_ms);
    stats_latency_samples_ms_.push_back(elapsed_ms);
  }
  if (performance_log_period_ <= 0.0) {
    return;
  }
  const auto current = std::chrono::steady_clock::now();
  const double window_seconds =
      std::chrono::duration<double>(current - stats_window_start_).count();
  if (window_seconds < performance_log_period_) {
    return;
  }
  const double average_ms = stats_frame_count_ > 0
      ? stats_total_latency_ms_ / static_cast<double>(stats_frame_count_) : -1.0;
  RCLCPP_INFO(
      get_logger(),
      "Tracking performance: processed=%.1f FPS end_to_end_avg=%.1f ms "
      "end_to_end_p95=%.1f ms end_to_end_max=%.1f ms",
      static_cast<double>(stats_frame_count_) / window_seconds,
      average_ms, percentile_95(stats_latency_samples_ms_), stats_max_latency_ms_);
  stats_window_start_ = current;
  stats_frame_count_ = 0;
  stats_total_latency_ms_ = 0.0;
  stats_max_latency_ms_ = 0.0;
  stats_latency_samples_ms_.clear();
}

void TrackingNode::diagnostic_callback(
    diagnostic_updater::DiagnosticStatusWrapper& status)
{
  const int64_t last_input_ns = last_input_steady_ns_.load(std::memory_order_relaxed);
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const double input_age = last_input_ns > 0
      ? static_cast<double>(now_ns - last_input_ns) * 1.0e-9
      : std::numeric_limits<double>::infinity();
  if (input_age > input_timeout_seconds_) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Detections input timeout");
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Tracker is receiving detections");
  }
  status.add("received_frames", received_frames_.load(std::memory_order_relaxed));
  status.add("tracker_type", tracker_type_);
  status.add("input_age_seconds", std::isfinite(input_age) ? input_age : -1.0);
  status.add("last_end_to_end_latency_ms", last_latency_ms_);
}

}  // namespace perception_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::TrackingNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
