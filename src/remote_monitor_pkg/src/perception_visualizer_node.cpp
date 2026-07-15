/**
 * @file perception_visualizer_node.cpp
 * @brief Read-only image annotation and structured telemetry for remote monitoring.
 */

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_servo_msgs/msg/gimbal_status.hpp>
#include <vision_servo_msgs/msg/perception_monitor.hpp>
#include <vision_servo_msgs/msg/target.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

namespace remote_monitor_pkg {
namespace {

using SteadyClock = std::chrono::steady_clock;
using Monitor = vision_servo_msgs::msg::PerceptionMonitor;
using Target = vision_servo_msgs::msg::Target;
using TargetArray = vision_servo_msgs::msg::TargetArray;

rcl_interfaces::msg::ParameterDescriptor descriptor(const std::string& description)
{
  rcl_interfaces::msg::ParameterDescriptor result;
  result.description = description;
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
  range.step = 0.0;
  result.floating_point_range.push_back(range);
  return result;
}

int64_t stamp_key(const builtin_interfaces::msg::Time& stamp)
{
  if (stamp.sec < 0) {
    return -1;
  }
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
      static_cast<int64_t>(stamp.nanosec);
}

std::string lowercase(std::string value)
{
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
  return value;
}

bool parse_bool(const std::string& value)
{
  const std::string normalized = lowercase(value);
  return normalized == "true" || normalized == "1" || normalized == "yes";
}

std::string tracking_state_name(uint8_t state)
{
  switch (state) {
    case Target::TRACKING_STATE_UNTRACKED:
      return "UNTRACKED";
    case Target::TRACKING_STATE_TENTATIVE:
      return "TENTATIVE";
    case Target::TRACKING_STATE_CONFIRMED:
      return "CONFIRMED";
    case Target::TRACKING_STATE_LOST:
      return "LOST";
    default:
      return "UNKNOWN";
  }
}

std::string health_name(uint8_t health)
{
  switch (health) {
    case Monitor::HEALTH_OK:
      return "OK";
    case Monitor::HEALTH_WARN:
      return "WARN";
    case Monitor::HEALTH_ERROR:
      return "ERROR";
    case Monitor::HEALTH_STALE:
      return "STALE";
    default:
      return "UNKNOWN";
  }
}

cv::Scalar tracking_color(uint8_t state)
{
  switch (state) {
    case Target::TRACKING_STATE_TENTATIVE:
      return cv::Scalar(0, 190, 255);       // amber
    case Target::TRACKING_STATE_CONFIRMED:
      return cv::Scalar(255, 170, 0);       // blue/cyan
    case Target::TRACKING_STATE_LOST:
      return cv::Scalar(150, 150, 150);     // gray
    default:
      return cv::Scalar(0, 220, 0);         // green
  }
}

class RateEstimator {
public:
  void tick(SteadyClock::time_point current)
  {
    if (last_.time_since_epoch().count() > 0) {
      const double dt = std::chrono::duration<double>(current - last_).count();
      if (dt > 1.0e-6 && dt < 5.0) {
        const double instant = 1.0 / dt;
        fps_ = fps_ <= 0.0 ? instant : 0.9 * fps_ + 0.1 * instant;
      }
    }
    last_ = current;
  }

  double fps() const { return fps_; }

private:
  SteadyClock::time_point last_{};
  double fps_ = 0.0;
};

struct ComponentHealth {
  uint8_t level = Monitor::HEALTH_UNKNOWN;
  std::string message{"No diagnostics received"};
  SteadyClock::time_point last_update{};
};

template<typename MessageT>
void trim_cache(std::map<int64_t, std::shared_ptr<const MessageT>>& cache, size_t maximum_size)
{
  while (cache.size() > maximum_size) {
    cache.erase(cache.begin());
  }
}

void draw_dashed_rectangle(
    cv::Mat& image, const cv::Rect& rectangle, const cv::Scalar& color, int thickness)
{
  constexpr int dash = 10;
  constexpr int gap = 6;
  for (int x = rectangle.x; x < rectangle.x + rectangle.width; x += dash + gap) {
    const int end_x = std::min(x + dash, rectangle.x + rectangle.width);
    cv::line(image, {x, rectangle.y}, {end_x, rectangle.y}, color, thickness, cv::LINE_AA);
    cv::line(
        image, {x, rectangle.y + rectangle.height},
        {end_x, rectangle.y + rectangle.height}, color, thickness, cv::LINE_AA);
  }
  for (int y = rectangle.y; y < rectangle.y + rectangle.height; y += dash + gap) {
    const int end_y = std::min(y + dash, rectangle.y + rectangle.height);
    cv::line(image, {rectangle.x, y}, {rectangle.x, end_y}, color, thickness, cv::LINE_AA);
    cv::line(
        image, {rectangle.x + rectangle.width, y},
        {rectangle.x + rectangle.width, end_y}, color, thickness, cv::LINE_AA);
  }
}

cv::Rect clipped_box(const Target& target, const cv::Size& image_size)
{
  if (image_size.width <= 0 || image_size.height <= 0) {
    return {};
  }
  const int x1 = std::clamp(
      static_cast<int>(std::floor(target.bbox[0])), 0, image_size.width - 1);
  const int y1 = std::clamp(
      static_cast<int>(std::floor(target.bbox[1])), 0, image_size.height - 1);
  const int x2 = std::clamp(
      static_cast<int>(std::ceil(target.bbox[2])), 0, image_size.width);
  const int y2 = std::clamp(
      static_cast<int>(std::ceil(target.bbox[3])), 0, image_size.height);
  if (x2 <= x1 || y2 <= y1) {
    return {};
  }
  return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

void draw_label(
    cv::Mat& image, const std::string& text, const cv::Point& anchor,
    const cv::Scalar& color, double scale = 0.5)
{
  int baseline = 0;
  const cv::Size size = cv::getTextSize(
      text, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &baseline);
  const int x = std::clamp(anchor.x, 0, std::max(0, image.cols - size.width - 6));
  const int y = std::clamp(anchor.y, size.height + 6, std::max(size.height + 6, image.rows - 2));
  cv::rectangle(
      image, cv::Rect(x, y - size.height - 6, size.width + 6, size.height + baseline + 6),
      cv::Scalar(20, 20, 20), cv::FILLED);
  cv::putText(
      image, text, {x + 3, y - 3}, cv::FONT_HERSHEY_SIMPLEX,
      scale, color, 1, cv::LINE_AA);
}

}  // namespace

class PerceptionVisualizerNode final : public rclcpp::Node {
public:
  PerceptionVisualizerNode()
      : Node("perception_visualizer_node")
  {
    declare_parameters();

    auto remote_image_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    remote_image_qos.best_effort();
    remote_image_qos.durability_volatile();
    remote_image_qos.lifespan(
        rclcpp::Duration(std::chrono::milliseconds(max_frame_age_ms_)));
    tracking_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "tracking_image/compressed", remote_image_qos);
    monitor_pub_ = create_publisher<Monitor>(
        "monitor_status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "image", sensor_qos,
        std::bind(&PerceptionVisualizerNode::image_callback, this, std::placeholders::_1));
    detections_sub_ = create_subscription<TargetArray>(
        "detections", sensor_qos,
        std::bind(&PerceptionVisualizerNode::detections_callback, this, std::placeholders::_1));
    tracks_sub_ = create_subscription<TargetArray>(
        "tracks", sensor_qos,
        std::bind(&PerceptionVisualizerNode::tracks_callback, this, std::placeholders::_1));
    diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&PerceptionVisualizerNode::diagnostics_callback, this, std::placeholders::_1));

    if (enable_future_inputs_) {
      target_3d_sub_ = create_subscription<TargetArray>(
          "target_3d", sensor_qos,
          std::bind(&PerceptionVisualizerNode::target_3d_callback, this, std::placeholders::_1));
      cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
          "cmd_vel", sensor_qos,
          std::bind(&PerceptionVisualizerNode::cmd_vel_callback, this, std::placeholders::_1));
      gimbal_sub_ = create_subscription<vision_servo_msgs::msg::GimbalStatus>(
          "gimbal_state", sensor_qos,
          std::bind(&PerceptionVisualizerNode::gimbal_callback, this, std::placeholders::_1));
    }

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / status_publish_rate_hz_));
    status_timer_ = create_wall_timer(
        period, std::bind(&PerceptionVisualizerNode::publish_monitor_status, this));
    const auto render_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / remote_publish_rate_hz_));
    render_timer_ = create_wall_timer(
        render_period, std::bind(&PerceptionVisualizerNode::publish_latest_frame, this));

    RCLCPP_INFO(
        get_logger(),
        "Remote perception visualizer ready: model=%s backend=%s cache=%zu "
        "remote=%.1fHz/%dpx/jpeg%d/max_age=%dms future_inputs=%s",
        yolo_model_.c_str(), inference_backend_.c_str(), cache_size_,
        remote_publish_rate_hz_, remote_max_width_, jpeg_quality_, max_frame_age_ms_,
        enable_future_inputs_ ? "enabled" : "disabled");
  }

private:
  struct RenderBundle {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    TargetArray::ConstSharedPtr detections;
    TargetArray::ConstSharedPtr tracks;
    uint64_t sequence = 0;
  };

  void declare_parameters()
  {
    cache_size_ = static_cast<size_t>(declare_parameter<int>(
        "sync_cache_size", 12,
        integer_descriptor("Maximum timestamp-matched image/detection frames retained.", 3, 120)));
    status_publish_rate_hz_ = declare_parameter<double>(
        "status_publish_rate_hz", 5.0,
        floating_descriptor("Structured monitor status publication rate.", 0.2, 30.0));
    remote_publish_rate_hz_ = declare_parameter<double>(
        "remote_publish_rate_hz", 5.0,
        floating_descriptor(
            "Maximum annotated image rate; detection and tracking remain unthrottled.",
            0.5, 30.0));
    remote_max_width_ = declare_parameter<int>(
        "remote_max_width", 640,
        integer_descriptor(
             "Maximum annotated image width; 0 preserves the source resolution.", 0, 7680));
    jpeg_quality_ = declare_parameter<int>(
        "jpeg_quality", 55,
        integer_descriptor("JPEG quality for the direct remote image stream.", 20, 95));
    max_frame_age_ms_ = declare_parameter<int>(
        "max_frame_age_ms", 300,
        integer_descriptor(
            "Drop an annotated frame instead of publishing it after this source age.",
            50, 5000));
    diagnostic_timeout_sec_ = declare_parameter<double>(
        "diagnostic_timeout_sec", 3.0,
        floating_descriptor("Age after which component diagnostics are marked stale.", 0.5, 60.0));
    optional_input_timeout_sec_ = declare_parameter<double>(
        "optional_input_timeout_sec", 2.0,
        floating_descriptor("Age after which optional future inputs are marked unavailable.", 0.1, 60.0));
    draw_detections_ = declare_parameter<bool>(
        "draw_detections", true, descriptor("Draw raw detector boxes."));
    draw_tracks_ = declare_parameter<bool>(
        "draw_tracks", true, descriptor("Draw tracked boxes, IDs, and lifecycle state."));
    draw_status_overlay_ = declare_parameter<bool>(
        "draw_status_overlay", true, descriptor("Draw component and performance telemetry."));
    render_without_subscribers_ = declare_parameter<bool>(
        "render_without_subscribers", false,
        descriptor("Render frames even when no remote/local image subscriber exists."));
    yolo_model_ = declare_parameter<std::string>(
        "yolo_model", "yolov8n", descriptor("Human-readable model name."));
    model_path_ = declare_parameter<std::string>(
        "model_path", "", descriptor("Deployment model/engine path shown in telemetry."));
    inference_backend_ = declare_parameter<std::string>(
        "inference_backend", "tensorrt", descriptor("Configured inference backend."));
    enable_future_inputs_ = declare_parameter<bool>(
        "enable_future_inputs", false,
        descriptor("Observe target_3d, cmd_vel, and gimbal_state without publishing commands."));
  }

  double observed_latency_ms(const builtin_interfaces::msg::Time& stamp) const
  {
    const rclcpp::Time source(stamp, get_clock()->get_clock_type());
    const rclcpp::Time current = now();
    if (source.nanoseconds() <= 0 || current < source) {
      return -1.0;
    }
    return static_cast<double>((current - source).nanoseconds()) * 1.0e-6;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& message)
  {
    const int64_t key = stamp_key(message->header.stamp);
    if (key < 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    image_rate_.tick(SteadyClock::now());
    latest_frame_id_ = message->header.frame_id;
    image_cache_[key] = message;
    trim_cache(image_cache_, cache_size_);
  }

  void detections_callback(const TargetArray::ConstSharedPtr& message)
  {
    const int64_t key = stamp_key(message->header.stamp);
    if (key < 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    detection_rate_.tick(SteadyClock::now());
    detection_latency_ms_ = observed_latency_ms(message->header.stamp);
    inference_time_estimate_ms_ = detection_latency_ms_;
    detection_count_ = static_cast<uint32_t>(message->targets.size());
    detection_cache_[key] = message;
    trim_cache(detection_cache_, cache_size_);
  }

  void tracks_callback(const TargetArray::ConstSharedPtr& message)
  {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    TargetArray::ConstSharedPtr detections;
    uint64_t unmatched_track_frames = 0;
    const int64_t key = stamp_key(message->header.stamp);
    if (key < 0) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      tracking_rate_.tick(SteadyClock::now());
      tracking_latency_ms_ = observed_latency_ms(message->header.stamp);
      track_count_ = static_cast<uint32_t>(message->targets.size());
      tracking_target_id_ = message->tracking_id;
      tracking_state_ = Target::TRACKING_STATE_UNTRACKED;
      tracking_state_name_ = message->tracking_id >= 0 ? "MISSING" : "NONE";
      const auto selected = std::find_if(
          message->targets.begin(), message->targets.end(),
          [message](const auto& target) { return target.id == message->tracking_id; });
      if (selected != message->targets.end()) {
        tracking_state_ = selected->tracking_state;
        tracking_state_name_ = tracking_state_name(selected->tracking_state);
      }

      const auto image_it = image_cache_.find(key);
      if (image_it != image_cache_.end()) {
        image = image_it->second;
      } else {
        ++unmatched_track_frames_;
      }
      unmatched_track_frames = unmatched_track_frames_;
      const auto detection_it = detection_cache_.find(key);
      if (detection_it != detection_cache_.end()) {
        detections = detection_it->second;
      }

      // All three streams preserve the source stamp. Once tracks arrive, older
      // frames can no longer produce a more complete visualization.
      image_cache_.erase(image_cache_.begin(), image_cache_.upper_bound(key));
      detection_cache_.erase(detection_cache_.begin(), detection_cache_.upper_bound(key));

      if (image) {
        latest_render_bundle_ = RenderBundle{
            image, detections, message, ++next_render_sequence_};
      }
    }

    if (!image) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No timestamp-matched Sony image for track frame (unmatched=%llu)",
          static_cast<unsigned long long>(unmatched_track_frames));
      return;
    }
  }

  void publish_latest_frame()
  {
    if (!render_without_subscribers_ &&
        tracking_image_pub_->get_subscription_count() == 0) {
      return;
    }

    std::optional<RenderBundle> bundle;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!latest_render_bundle_ ||
          latest_render_bundle_->sequence == last_render_sequence_) {
        return;
      }
      bundle = latest_render_bundle_;
      last_render_sequence_ = bundle->sequence;
    }

    const double source_age_ms = observed_latency_ms(bundle->image->header.stamp);
    if (source_age_ms > static_cast<double>(max_frame_age_ms_)) {
      ++stale_frame_drop_count_;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping stale remote frame: age=%.1fms limit=%dms dropped=%llu",
          source_age_ms, max_frame_age_ms_,
          static_cast<unsigned long long>(stale_frame_drop_count_));
      return;
    }

    render_and_publish(bundle->image, bundle->detections, bundle->tracks);
  }

  void diagnostics_callback(
      const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const auto& status : message->status) {
      const std::string identity = lowercase(status.name + " " + status.hardware_id);
      ComponentHealth* component = nullptr;
      enum class Kind { None, Camera, Detector, Tracker } kind = Kind::None;
      if (identity.find("sony_camera") != std::string::npos ||
          identity.find("sony_zv") != std::string::npos) {
        component = &camera_health_;
        kind = Kind::Camera;
      } else if (identity.find("yolo_detector") != std::string::npos ||
                 identity.find("image_and_inference") != std::string::npos) {
        component = &detector_health_;
        kind = Kind::Detector;
      } else if (identity.find("multi_object_tracker") != std::string::npos ||
                 identity.find("input_and_latency") != std::string::npos) {
        component = &tracker_health_;
        kind = Kind::Tracker;
      }
      if (component == nullptr) {
        continue;
      }
      component->level = diagnostic_level(status.level);
      component->message = status.message;
      component->last_update = SteadyClock::now();
      for (const auto& value : status.values) {
        if (kind == Kind::Camera && value.key == "connected") {
          camera_connected_ = parse_bool(value.value);
        } else if (kind == Kind::Detector && value.key == "model_ready") {
          model_ready_ = parse_bool(value.value);
        } else if (kind == Kind::Detector && value.key == "backend") {
          inference_backend_ = value.value;
        }
      }
    }
  }

  static uint8_t diagnostic_level(uint8_t level)
  {
    switch (level) {
      case diagnostic_msgs::msg::DiagnosticStatus::OK:
        return Monitor::HEALTH_OK;
      case diagnostic_msgs::msg::DiagnosticStatus::WARN:
        return Monitor::HEALTH_WARN;
      case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
        return Monitor::HEALTH_ERROR;
      case diagnostic_msgs::msg::DiagnosticStatus::STALE:
        return Monitor::HEALTH_STALE;
      default:
        return Monitor::HEALTH_UNKNOWN;
    }
  }

  void target_3d_callback(const TargetArray::ConstSharedPtr& message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const Target* selected = nullptr;
    for (const auto& target : message->targets) {
      if (target.depth_confidence <= 0.0F) {
        continue;
      }
      if (target.id == message->tracking_id) {
        selected = &target;
        break;
      }
      if (selected == nullptr || target.depth_confidence > selected->depth_confidence) {
        selected = &target;
      }
    }
    if (selected != nullptr) {
      target_3d_position_ = selected->position;
      target_3d_last_update_ = SteadyClock::now();
    }
  }

  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cmd_vel_ = {
      static_cast<float>(message->twist.linear.x),
      static_cast<float>(message->twist.linear.y),
      static_cast<float>(message->twist.angular.z)};
    cmd_vel_last_update_ = SteadyClock::now();
  }

  void gimbal_callback(
      const vision_servo_msgs::msg::GimbalStatus::ConstSharedPtr& message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    gimbal_yaw_ = message->yaw;
    gimbal_pitch_ = message->pitch;
    gimbal_last_update_ = SteadyClock::now();
  }

  uint8_t health_with_timeout(const ComponentHealth& component, SteadyClock::time_point current) const
  {
    if (component.last_update.time_since_epoch().count() == 0) {
      return Monitor::HEALTH_UNKNOWN;
    }
    if (std::chrono::duration<double>(current - component.last_update).count() >
        diagnostic_timeout_sec_) {
      return Monitor::HEALTH_STALE;
    }
    return component.level;
  }

  bool fresh(SteadyClock::time_point timestamp, SteadyClock::time_point current) const
  {
    return timestamp.time_since_epoch().count() > 0 &&
        std::chrono::duration<double>(current - timestamp).count() <= optional_input_timeout_sec_;
  }

  Monitor make_monitor_message(const std_msgs::msg::Header& header)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto current = SteadyClock::now();
    Monitor result;
    result.header = header;
    result.camera_health = health_with_timeout(camera_health_, current);
    result.detector_health = health_with_timeout(detector_health_, current);
    result.tracker_health = health_with_timeout(tracker_health_, current);
    result.camera_status = camera_health_.message;
    result.detector_status = detector_health_.message;
    result.tracker_status = tracker_health_.message;
    result.camera_connected = camera_connected_;
    result.model_ready = model_ready_;
    result.yolo_model = yolo_model_;
    result.inference_backend = inference_backend_;
    result.tensor_rt_engine = model_path_.empty()
        ? std::string{} : std::filesystem::path(model_path_).filename().string();
    result.image_fps = static_cast<float>(image_rate_.fps());
    result.detection_fps = static_cast<float>(detection_rate_.fps());
    result.tracking_fps = static_cast<float>(tracking_rate_.fps());
    result.detection_latency_ms = static_cast<float>(detection_latency_ms_);
    result.tracking_latency_ms = static_cast<float>(tracking_latency_ms_);
    result.visualization_latency_ms = static_cast<float>(visualization_latency_ms_);
    result.inference_time_ms_estimate = static_cast<float>(inference_time_estimate_ms_);
    result.detection_count = detection_count_;
    result.track_count = track_count_;
    result.tracking_target_id = tracking_target_id_;
    result.tracking_state = tracking_state_;
    result.tracking_state_name = tracking_state_name_;

    result.target_3d_available = fresh(target_3d_last_update_, current);
    result.target_3d_position = target_3d_position_;
    result.cmd_vel_available = fresh(cmd_vel_last_update_, current);
    result.cmd_vel = cmd_vel_;
    result.gimbal_state_available = fresh(gimbal_last_update_, current);
    result.gimbal_yaw = gimbal_yaw_;
    result.gimbal_pitch = gimbal_pitch_;
    return result;
  }

  void publish_monitor_status()
  {
    std_msgs::msg::Header header;
    header.stamp = now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      header.frame_id = latest_frame_id_;
    }
    monitor_pub_->publish(make_monitor_message(header));
  }

  void render_and_publish(
      const sensor_msgs::msg::Image::ConstSharedPtr& image,
      const TargetArray::ConstSharedPtr& detections,
      const TargetArray::ConstSharedPtr& tracks)
  {
    try {
      const auto cv_image = cv_bridge::toCvShare(image, "bgr8");
      cv::Mat canvas = cv_image->image.clone();

      if (draw_detections_ && detections) {
        for (const auto& detection : detections->targets) {
          const cv::Rect box = clipped_box(detection, canvas.size());
          if (box.empty()) {
            continue;
          }
          const cv::Scalar color(0, 210, 0);
          cv::rectangle(canvas, box, color, 1, cv::LINE_AA);
          std::ostringstream label;
          label << "UNTRACKED " << detection.class_name << ' ' << std::fixed
                << std::setprecision(2) << detection.confidence;
          draw_label(canvas, label.str(), {box.x, box.y}, color, 0.45);
        }
      }

      if (draw_tracks_) {
        for (const auto& track : tracks->targets) {
          const cv::Rect box = clipped_box(track, canvas.size());
          if (box.empty()) {
            continue;
          }
          const bool is_target = track.id == tracks->tracking_id;
          const cv::Scalar color = is_target
              ? cv::Scalar(255, 0, 255) : tracking_color(track.tracking_state);
          const int thickness = is_target ? 4 : 2;
          if (track.tracking_state == Target::TRACKING_STATE_LOST) {
            draw_dashed_rectangle(canvas, box, color, thickness);
          } else {
            cv::rectangle(canvas, box, color, thickness, cv::LINE_AA);
          }
          std::ostringstream label;
          if (is_target) {
            label << "TARGET ";
          }
          label << "ID:" << track.id << ' ' << track.class_name << ' '
                << tracking_state_name(track.tracking_state) << ' '
                << std::fixed << std::setprecision(2) << track.confidence;
          draw_label(canvas, label.str(), {box.x, box.y - 20}, color, 0.5);
        }
      }

      if (draw_status_overlay_) {
        draw_status(canvas, make_monitor_message(image->header));
      }

      cv::Mat remote_canvas;
      if (remote_max_width_ > 0 && canvas.cols > remote_max_width_) {
        const double scale = static_cast<double>(remote_max_width_) /
            static_cast<double>(canvas.cols);
        const int output_height = std::max(
            1, static_cast<int>(std::lround(static_cast<double>(canvas.rows) * scale)));
        cv::resize(
            canvas, remote_canvas, cv::Size(remote_max_width_, output_height),
            0.0, 0.0, cv::INTER_AREA);
      } else {
        remote_canvas = canvas;
      }

      sensor_msgs::msg::CompressedImage output;
      output.header = image->header;
      output.format = "jpeg";
      const std::vector<int> encode_parameters{
          cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      if (!cv::imencode(".jpg", remote_canvas, output.data, encode_parameters)) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Remote visualizer failed to encode a JPEG frame");
        return;
      }

      // Encoding time is part of end-to-end age. Never hand an already stale
      // frame to the bridge, even if it was fresh when this timer started.
      const double publish_age_ms = observed_latency_ms(image->header.stamp);
      if (publish_age_ms > static_cast<double>(max_frame_age_ms_)) {
        ++stale_frame_drop_count_;
        return;
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        visualization_latency_ms_ = publish_age_ms;
      }
      tracking_image_pub_->publish(output);
      ++published_remote_frame_count_;
    } catch (const cv_bridge::Exception& error) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Remote visualizer cv_bridge conversion failed: %s", error.what());
    } catch (const cv::Exception& error) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Remote visualizer OpenCV drawing failed: %s", error.what());
    }
  }

  static void draw_status(cv::Mat& image, const Monitor& status)
  {
    if (image.empty()) {
      return;
    }
    std::vector<std::string> lines;
    {
      std::ostringstream line;
      line << "Camera:" << health_name(status.camera_health)
           << " YOLO:" << health_name(status.detector_health)
           << " Tracker:" << health_name(status.tracker_health);
      lines.push_back(line.str());
    }
    {
      std::ostringstream line;
      line << std::fixed << std::setprecision(1)
           << "FPS img/det/trk: " << status.image_fps << '/'
           << status.detection_fps << '/' << status.tracking_fps;
      lines.push_back(line.str());
    }
    {
      std::ostringstream line;
      line << std::fixed << std::setprecision(1)
           << "Latency det/trk/vis: " << status.detection_latency_ms << '/'
           << status.tracking_latency_ms << '/' << status.visualization_latency_ms << " ms";
      lines.push_back(line.str());
    }
    {
      std::ostringstream line;
      line << "Detections:" << status.detection_count << " Tracks:" << status.track_count
           << " Target:" << status.tracking_target_id << ' ' << status.tracking_state_name;
      lines.push_back(line.str());
    }
    lines.push_back("YOLO: " + status.yolo_model + " Backend: " + status.inference_backend);
    {
      std::ostringstream line;
      line << std::fixed << std::setprecision(1)
           << "Inference(est): " << status.inference_time_ms_estimate << " ms";
      if (!status.tensor_rt_engine.empty()) {
        line << " Engine: " << status.tensor_rt_engine;
      }
      lines.push_back(line.str());
    }

    constexpr int margin = 10;
    constexpr int line_height = 24;
    const int panel_width = std::min(620, std::max(1, image.cols - 2 * margin));
    const int panel_height = std::min(
        static_cast<int>(lines.size()) * line_height + 16,
        std::max(1, image.rows - 2 * margin));
    if (panel_width <= 1 || panel_height <= 1) {
      return;
    }
    cv::Mat roi = image(cv::Rect(margin, margin, panel_width, panel_height));
    cv::Mat dark(roi.size(), roi.type(), cv::Scalar(10, 10, 10));
    cv::addWeighted(dark, 0.68, roi, 0.32, 0.0, roi);
    for (size_t index = 0; index < lines.size(); ++index) {
      const int y = margin + 23 + static_cast<int>(index) * line_height;
      if (y >= image.rows - margin) {
        break;
      }
      cv::putText(
          image, lines[index], {margin + 10, y}, cv::FONT_HERSHEY_SIMPLEX,
          0.55, cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
    }
  }

  size_t cache_size_ = 12;
  double status_publish_rate_hz_ = 5.0;
  double remote_publish_rate_hz_ = 5.0;
  double diagnostic_timeout_sec_ = 3.0;
  double optional_input_timeout_sec_ = 2.0;
  int remote_max_width_ = 640;
  int jpeg_quality_ = 55;
  int max_frame_age_ms_ = 300;
  bool draw_detections_ = true;
  bool draw_tracks_ = true;
  bool draw_status_overlay_ = true;
  bool render_without_subscribers_ = false;
  bool enable_future_inputs_ = false;
  std::string yolo_model_{"yolov8n"};
  std::string model_path_;
  std::string inference_backend_{"tensorrt"};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<TargetArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<TargetArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<TargetArray>::SharedPtr target_3d_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalStatus>::SharedPtr gimbal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr tracking_image_pub_;
  rclcpp::Publisher<Monitor>::SharedPtr monitor_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr render_timer_;

  mutable std::mutex state_mutex_;
  std::map<int64_t, sensor_msgs::msg::Image::ConstSharedPtr> image_cache_;
  std::map<int64_t, TargetArray::ConstSharedPtr> detection_cache_;
  RateEstimator image_rate_;
  RateEstimator detection_rate_;
  RateEstimator tracking_rate_;
  ComponentHealth camera_health_;
  ComponentHealth detector_health_;
  ComponentHealth tracker_health_;
  std::string latest_frame_id_;
  double detection_latency_ms_ = -1.0;
  double tracking_latency_ms_ = -1.0;
  double visualization_latency_ms_ = -1.0;
  double inference_time_estimate_ms_ = -1.0;
  uint32_t detection_count_ = 0;
  uint32_t track_count_ = 0;
  int32_t tracking_target_id_ = -1;
  uint8_t tracking_state_ = Target::TRACKING_STATE_UNTRACKED;
  std::string tracking_state_name_{"NONE"};
  bool camera_connected_ = false;
  bool model_ready_ = false;
  uint64_t unmatched_track_frames_ = 0;
  std::optional<RenderBundle> latest_render_bundle_;
  uint64_t next_render_sequence_ = 0;
  uint64_t last_render_sequence_ = 0;
  uint64_t stale_frame_drop_count_ = 0;
  uint64_t published_remote_frame_count_ = 0;

  std::array<float, 3> target_3d_position_{};
  std::array<float, 3> cmd_vel_{};
  float gimbal_yaw_ = 0.0F;
  float gimbal_pitch_ = 0.0F;
  SteadyClock::time_point target_3d_last_update_{};
  SteadyClock::time_point cmd_vel_last_update_{};
  SteadyClock::time_point gimbal_last_update_{};
};

}  // namespace remote_monitor_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<remote_monitor_pkg::PerceptionVisualizerNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(
        rclcpp::get_logger("perception_visualizer_node"),
        "Remote monitor failed to start: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
