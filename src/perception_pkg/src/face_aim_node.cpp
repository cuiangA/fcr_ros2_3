#include "perception_pkg/face_aim_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <vision_servo_msgs/msg/aim_target2_d.hpp>

namespace perception_pkg {

FaceAimNode::FaceAimNode(const rclcpp::NodeOptions& options)
  : Node("face_aim_node", options)
{
  declare_parameter("input_timeout_seconds", 2.0);
  declare_parameter("max_tolerance_ns", 1'000'000);
  declare_parameter("publish_debug", true);

  input_timeout_seconds_ = get_parameter("input_timeout_seconds").as_double();
  max_tolerance_ns_ = get_parameter("max_tolerance_ns").as_int();
  publish_debug_ = get_parameter("publish_debug").as_bool();

  image_sub_ = image_transport::create_subscription(
      this, "/sony/image_raw",
      std::bind(&FaceAimNode::image_callback, this, std::placeholders::_1),
      "raw", qos::image().get_rmw_qos_profile());

  tracks_sub_ = create_subscription<vision_servo_msgs::msg::TargetArray>(
      "/perception/tracks", qos::perception(),
      std::bind(&FaceAimNode::tracks_callback, this, std::placeholders::_1));

  aim_pub_ = create_publisher<vision_servo_msgs::msg::AimTarget2D>(
      "/perception/aim_target_2d", qos::perception());

  debug_pub_ = image_transport::create_publisher(this, "face_aim_debug");

  timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FaceAimNode::check_timeout, this));

  last_input_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(get_logger(), "face_aim_node started");
}

void FaceAimNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  RCLCPP_DEBUG(get_logger(), "image_callback: stamp=%u.%u",
      msg->header.stamp.sec, msg->header.stamp.nanosec);
  std::lock_guard<std::mutex> lock(mutex_);
  pending_image_ = msg;
  last_input_time_ = std::chrono::steady_clock::now();
  try_match_and_process();
}

void FaceAimNode::tracks_callback(
    const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg)
{
  RCLCPP_DEBUG(get_logger(), "tracks_callback: stamp=%u.%u tracking_id=%d",
      msg->header.stamp.sec, msg->header.stamp.nanosec, msg->tracking_id);
  std::lock_guard<std::mutex> lock(mutex_);
  pending_tracks_ = msg;
  last_input_time_ = std::chrono::steady_clock::now();
  try_match_and_process();
}

bool FaceAimNode::stamps_match(
    const builtin_interfaces::msg::Time& a,
    const builtin_interfaces::msg::Time& b,
    const int64_t tolerance_ns)
{
  const int64_t diff_ns =
      (static_cast<int64_t>(a.sec) - static_cast<int64_t>(b.sec)) * 1'000'000'000LL +
      (static_cast<int64_t>(a.nanosec) - static_cast<int64_t>(b.nanosec));
  return std::abs(diff_ns) <= tolerance_ns;
}

void FaceAimNode::try_match_and_process()
{
  if (!pending_image_) {
    RCLCPP_DEBUG(get_logger(), "try_match: waiting for image");
    return;
  }
  if (!pending_tracks_) {
    RCLCPP_DEBUG(get_logger(), "try_match: waiting for tracks");
    return;
  }

  if (!stamps_match(
          pending_image_->header.stamp,
          pending_tracks_->header.stamp,
          max_tolerance_ns_)) {
    RCLCPP_DEBUG(get_logger(), "try_match: stamp mismatch image=%u.%u tracks=%u.%u",
        pending_image_->header.stamp.sec, pending_image_->header.stamp.nanosec,
        pending_tracks_->header.stamp.sec, pending_tracks_->header.stamp.nanosec);
    return;
  }
  RCLCPP_INFO(get_logger(), "try_match: matched");

  const auto& tracks = *pending_tracks_;

  vision_servo_msgs::msg::AimTarget2D aim;
  aim.header = pending_image_->header;
  aim.tracking_id = -1;
  aim.valid = false;
  aim.confidence = 0.0f;
  aim.pixel_x = 0.0f;
  aim.pixel_y = 0.0f;
  aim.source = vision_servo_msgs::msg::AimTarget2D::UPPER_BODY;

  if (tracks.tracking_id >= 0) {
    const auto it = std::find_if(
        tracks.targets.begin(), tracks.targets.end(),
        [id = tracks.tracking_id](const auto& t) {
          return t.id == id && t.class_name == "person";
        });

    if (it != tracks.targets.end() && it->visible) {
      aim.tracking_id = it->id;
      aim.pixel_x = it->center[0];
      aim.pixel_y = it->center[1];
      aim.confidence = it->confidence;
      aim.source = vision_servo_msgs::msg::AimTarget2D::UPPER_BODY;
      aim.valid = true;
    }
  }

  aim_pub_->publish(aim);
  RCLCPP_INFO(get_logger(), "published valid=%s id=%d (%.1f, %.1f)",
      aim.valid ? "true" : "false", aim.tracking_id, aim.pixel_x, aim.pixel_y);

  if (publish_debug_) {
    publish_debug_image(pending_image_, aim.header, aim);
  }

  pending_image_.reset();
  pending_tracks_.reset();
}

void FaceAimNode::check_timeout()
{
  const double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now() - last_input_time_).count();

  const bool in_timeout = elapsed > input_timeout_seconds_;

  if (in_timeout && !was_in_timeout_) {
    was_in_timeout_ = true;
    RCLCPP_INFO(get_logger(), "check_timeout: entering timeout");
    vision_servo_msgs::msg::AimTarget2D aim;
    aim.header.stamp = this->now();
    aim.header.frame_id = "";
    aim.tracking_id = -1;
    aim.valid = false;
    aim.confidence = 0.0f;
    aim.pixel_x = 0.0f;
    aim.pixel_y = 0.0f;
    aim.source = vision_servo_msgs::msg::AimTarget2D::LOST_PREDICTION;
    aim_pub_->publish(aim);
  } else if (!in_timeout && was_in_timeout_) {
    was_in_timeout_ = false;
    RCLCPP_INFO(get_logger(), "check_timeout: leaving timeout");
  }
}

void FaceAimNode::publish_debug_image(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const std_msgs::msg::Header& header,
    const vision_servo_msgs::msg::AimTarget2D& aim)
{
  if (debug_pub_.getNumSubscribers() == 0) {
    return;
  }

  cv::Mat debug;
  try {
    const auto cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
    debug = cv_ptr->image.clone();
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Debug image conversion failed (%s), using blank canvas", e.what());
    if (image_msg->height > 0 && image_msg->width > 0) {
      debug = cv::Mat::zeros(image_msg->height, image_msg->width, CV_8UC3);
    } else {
      debug = cv::Mat::zeros(480, 640, CV_8UC3);
    }
  }

  if (aim.valid) {
    const cv::Point aim_pt(
        static_cast<int>(aim.pixel_x),
        static_cast<int>(aim.pixel_y));
    cv::circle(debug, aim_pt, 5, cv::Scalar(0, 255, 0), -1);
    cv::putText(debug,
        "ID:" + std::to_string(aim.tracking_id),
        cv::Point(aim_pt.x + 10, aim_pt.y - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
  }

  const auto bridge = cv_bridge::CvImage(header, "bgr8", debug).toImageMsg();
  debug_pub_.publish(bridge);
}

}  // namespace perception_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::FaceAimNode>(
      rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
