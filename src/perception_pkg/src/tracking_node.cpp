/**
 * @file tracking_node.cpp
 * @brief 多目标跟踪节点实现；MultiObjectTracker 核心逻辑在 multi_object_tracker.cpp 中共享。
 */

#include "perception_pkg/tracking_node.hpp"
#include "perception_pkg/qos.hpp"

#include <algorithm>
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
    tracker_(/*max_age=*/30, /*min_hits=*/3, /*iou_threshold=*/0.3),
    target_selector_(true, "person"),
    diagnostics_(this)
{
  this->declare_parameter("max_age", 30,
      integer_descriptor("Maximum consecutive missed frames retained as LOST.", 0, 10000));
  this->declare_parameter("min_hits", 3,
      integer_descriptor("Consecutive detections required to confirm a track.", 1, 10000));
  this->declare_parameter("iou_threshold", 0.3,
      floating_descriptor("Minimum IoU used to associate a detection and track.", 0.0, 1.0));
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
  const int max_age = this->get_parameter("max_age").as_int();
  const int min_hits = this->get_parameter("min_hits").as_int();
  const float iou_threshold =
      static_cast<float>(this->get_parameter("iou_threshold").as_double());
  const bool auto_select = this->get_parameter("auto_select").as_bool();
  const std::string target_class_filter = this->get_parameter("class_filter").as_string();
  input_timeout_seconds_ = this->get_parameter("input_timeout_seconds").as_double();
  performance_log_period_ = this->get_parameter("performance_log_period").as_double();
  tracker_ = MultiObjectTracker(max_age, min_hits, iou_threshold);
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

  diagnostics_.setHardwareID("multi_object_tracker");
  diagnostics_.add("input_and_latency", this, &TrackingNode::diagnostic_callback);
  stats_window_start_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(get_logger(), "跟踪节点已启动 (max_age=%d, min_hits=%d)",
              max_age, min_hits);
}

void TrackingNode::detection_callback(
    const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg)
{
  received_frames_.fetch_add(1, std::memory_order_relaxed);
  last_input_steady_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count(),
      std::memory_order_relaxed);
  tracker_.update(*msg);
  const std::string output_frame = camera_frame_.empty() ? msg->header.frame_id : camera_frame_;
  const uint64_t timestamp_ns = msg->header.stamp.sec >= 0
      ? static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(msg->header.stamp.nanosec)
      : 0;
  auto tracks = tracker_.get_tracks(
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
