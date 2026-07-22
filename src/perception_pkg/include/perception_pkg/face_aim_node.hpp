#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_servo_msgs/msg/aim_target2_d.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

#include "perception_pkg/qos.hpp"

namespace perception_pkg {

class FaceAimNode : public rclcpp::Node {
public:
  explicit FaceAimNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void tracks_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg);
  void try_match_and_process();
  void check_timeout();
  void publish_debug_image(
      const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
      const std_msgs::msg::Header& header,
      const vision_servo_msgs::msg::AimTarget2D& aim);
  static bool stamps_match(
      const builtin_interfaces::msg::Time& a,
      const builtin_interfaces::msg::Time& b,
      int64_t tolerance_ns);

  rclcpp::Publisher<vision_servo_msgs::msg::AimTarget2D>::SharedPtr aim_pub_;
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr tracks_sub_;
  image_transport::Publisher debug_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_image_;
  vision_servo_msgs::msg::TargetArray::ConstSharedPtr pending_tracks_;
  std::chrono::steady_clock::time_point last_input_time_;

  double input_timeout_seconds_ = 2.0;
  int64_t max_tolerance_ns_ = 1'000'000;
  bool publish_debug_ = true;
  bool was_in_timeout_ = false;
};

}  // namespace perception_pkg
