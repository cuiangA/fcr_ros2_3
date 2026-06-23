#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <vision_servo_msgs/msg/shot_reference.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

#include "servo_control_pkg/qos.hpp"

namespace
{

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double apply_deadband(double value, double deadband)
{
  return std::abs(value) < deadband ? 0.0 : value;
}

double normalize_image_coord(double value, double image_size)
{
  if (value >= 0.0 && value <= 1.0) {
    return value;
  }
  return image_size > 1.0 ? value / image_size : 0.5;
}

double low_pass(double previous, double current, double alpha)
{
  return alpha * current + (1.0 - alpha) * previous;
}

}  // namespace

class GimbalTargetControllerNode : public rclcpp::Node
{
public:
  GimbalTargetControllerNode()
  : Node("gimbal_target_controller_node")
  {
    target_topic_ = declare_parameter<std::string>("target_topic", "/target/current");
    reference_topic_ = declare_parameter<std::string>("reference_topic", "/shot/reference");
    output_topic_ = declare_parameter<std::string>("output_topic", "/control/cmd_gimbal_raw");
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 50.0);
    image_width_ = declare_parameter<double>("image_width", 640.0);
    image_height_ = declare_parameter<double>("image_height", 480.0);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.2);
    kp_pitch_ = declare_parameter<double>("kp_pitch", 0.9);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 1.5);
    max_pitch_rate_ = declare_parameter<double>("max_pitch_rate", 1.0);
    error_deadband_ = declare_parameter<double>("error_deadband", 0.015);
    filter_alpha_ = declare_parameter<double>("filter_alpha", 0.35);
    target_stale_timeout_ = declare_parameter<double>("target_stale_timeout", 0.35);
    reference_stale_timeout_ = declare_parameter<double>("reference_stale_timeout", 0.5);
    hold_on_lost_ = declare_parameter<bool>("hold_on_lost", true);

    update_rate_hz_ = std::max(update_rate_hz_, 1.0);
    filter_alpha_ = clamp(filter_alpha_, 0.0, 1.0);

    cmd_pub_ = create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      output_topic_, servo_control_pkg::qos::control_cmd());
    target_sub_ = create_subscription<vision_servo_msgs::msg::TargetArray>(
      target_topic_, rclcpp::QoS(5).reliable(),
      std::bind(&GimbalTargetControllerNode::target_callback, this, std::placeholders::_1));
    reference_sub_ = create_subscription<vision_servo_msgs::msg::ShotReference>(
      reference_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&GimbalTargetControllerNode::reference_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&GimbalTargetControllerNode::update, this));

    RCLCPP_INFO(get_logger(), "Gimbal target controller started: %s", output_topic_.c_str());
  }

private:
  void target_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr msg)
  {
    latest_targets_ = *msg;
    target_received_time_ = now();
    has_targets_ = true;
  }

  void reference_callback(const vision_servo_msgs::msg::ShotReference::ConstSharedPtr msg)
  {
    latest_reference_ = *msg;
    reference_received_time_ = now();
    has_reference_ = true;
  }

  void update()
  {
    const auto t = now();
    const auto * target = select_target();
    if (!inputs_ready(t) || target == nullptr || !latest_reference_.gimbal_tracking_enabled) {
      publish_hold(t);
      return;
    }

    const double u = normalize_image_coord(target->center[0], image_width_);
    const double v = normalize_image_coord(target->center[1], image_height_);
    const double desired_u = clamp(latest_reference_.desired_composition_u, 0.0, 1.0);
    const double desired_v = clamp(latest_reference_.desired_composition_v, 0.0, 1.0);

    const double error_u = apply_deadband(2.0 * (u - desired_u), error_deadband_);
    const double error_v = apply_deadband(2.0 * (v - desired_v), error_deadband_);

    const double yaw_limit = latest_reference_.max_gimbal_yaw_rate > 0.0f ?
      std::min(max_yaw_rate_, static_cast<double>(latest_reference_.max_gimbal_yaw_rate)) : max_yaw_rate_;
    const double pitch_limit = latest_reference_.max_gimbal_pitch_rate > 0.0f ?
      std::min(max_pitch_rate_, static_cast<double>(latest_reference_.max_gimbal_pitch_rate)) : max_pitch_rate_;

    double yaw_rate = clamp(-kp_yaw_ * error_u, -yaw_limit, yaw_limit);
    double pitch_rate = clamp(-kp_pitch_ * error_v, -pitch_limit, pitch_limit);
    yaw_rate = low_pass(previous_yaw_rate_, yaw_rate, filter_alpha_);
    pitch_rate = low_pass(previous_pitch_rate_, pitch_rate, filter_alpha_);
    previous_yaw_rate_ = yaw_rate;
    previous_pitch_rate_ = pitch_rate;

    vision_servo_msgs::msg::GimbalCmd cmd;
    cmd.header.stamp = t;
    cmd.yaw_rate = static_cast<float>(yaw_rate);
    cmd.pitch_rate = static_cast<float>(pitch_rate);
    cmd.hold_yaw = false;
    cmd.hold_pitch = false;
    cmd_pub_->publish(cmd);
  }

  bool inputs_ready(const rclcpp::Time & t) const
  {
    if (!has_targets_) {
      return false;
    }
    if ((t - target_received_time_).seconds() > target_stale_timeout_) {
      return false;
    }
    if (!has_reference_) {
      return true;
    }
    return (t - reference_received_time_).seconds() <= reference_stale_timeout_;
  }

  const vision_servo_msgs::msg::Target * select_target() const
  {
    if (!has_targets_ || latest_targets_.targets.empty()) {
      return nullptr;
    }

    int preferred_id = latest_targets_.tracking_id;
    if (has_reference_ && latest_reference_.target_id >= 0) {
      preferred_id = latest_reference_.target_id;
    }

    if (preferred_id >= 0) {
      for (const auto & target : latest_targets_.targets) {
        if (target.id == preferred_id) {
          return &target;
        }
      }
    }
    return &latest_targets_.targets.front();
  }

  void publish_hold(const rclcpp::Time & stamp)
  {
    previous_yaw_rate_ = hold_on_lost_ ? 0.0 : low_pass(previous_yaw_rate_, 0.0, filter_alpha_);
    previous_pitch_rate_ = hold_on_lost_ ? 0.0 : low_pass(previous_pitch_rate_, 0.0, filter_alpha_);

    vision_servo_msgs::msg::GimbalCmd cmd;
    cmd.header.stamp = stamp;
    cmd.yaw_rate = static_cast<float>(previous_yaw_rate_);
    cmd.pitch_rate = static_cast<float>(previous_pitch_rate_);
    cmd.hold_yaw = hold_on_lost_;
    cmd.hold_pitch = hold_on_lost_;
    cmd_pub_->publish(cmd);
  }

  std::string target_topic_;
  std::string reference_topic_;
  std::string output_topic_;
  double update_rate_hz_{50.0};
  double image_width_{640.0};
  double image_height_{480.0};
  double kp_yaw_{1.2};
  double kp_pitch_{0.9};
  double max_yaw_rate_{1.5};
  double max_pitch_rate_{1.0};
  double error_deadband_{0.015};
  double filter_alpha_{0.35};
  double target_stale_timeout_{0.35};
  double reference_stale_timeout_{0.5};
  bool hold_on_lost_{true};

  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr target_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::ShotReference>::SharedPtr reference_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_targets_{false};
  bool has_reference_{false};
  vision_servo_msgs::msg::TargetArray latest_targets_;
  vision_servo_msgs::msg::ShotReference latest_reference_;
  rclcpp::Time target_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reference_received_time_{0, 0, RCL_ROS_TIME};
  double previous_yaw_rate_{0.0};
  double previous_pitch_rate_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalTargetControllerNode>());
  rclcpp::shutdown();
  return 0;
}
