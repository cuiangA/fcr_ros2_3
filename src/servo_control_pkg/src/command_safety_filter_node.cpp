#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <vision_servo_msgs/msg/platform_state.hpp>
#include <vision_servo_msgs/msg/shot_reference.hpp>

#include "servo_control_pkg/qos.hpp"

namespace
{

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

}  // namespace

class CommandSafetyFilterNode : public rclcpp::Node
{
public:
  CommandSafetyFilterNode()
  : Node("command_safety_filter_node")
  {
    raw_cmd_vel_topic_ = declare_parameter<std::string>("raw_cmd_vel_topic", "/control/cmd_vel_raw");
    raw_gimbal_topic_ = declare_parameter<std::string>("raw_gimbal_topic", "/control/cmd_gimbal_raw");
    platform_state_topic_ = declare_parameter<std::string>("platform_state_topic", "/platform/state");
    reference_topic_ = declare_parameter<std::string>("reference_topic", "/shot/reference");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cmd_gimbal_topic_ = declare_parameter<std::string>("cmd_gimbal_topic", "/cmd_gimbal");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 50.0);
    raw_timeout_sec_ = declare_parameter<double>("raw_timeout_sec", 0.25);
    reference_timeout_sec_ = declare_parameter<double>("reference_timeout_sec", 0.5);
    platform_timeout_sec_ = declare_parameter<double>("platform_timeout_sec", 0.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.8);
    max_vy_ = declare_parameter<double>("max_vy", 0.5);
    max_wz_ = declare_parameter<double>("max_wz", 1.2);
    max_gimbal_yaw_rate_ = declare_parameter<double>("max_gimbal_yaw_rate", 1.5);
    max_gimbal_pitch_rate_ = declare_parameter<double>("max_gimbal_pitch_rate", 1.0);

    update_rate_hz_ = std::max(update_rate_hz_, 1.0);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic_, servo_control_pkg::qos::control_cmd());
    cmd_gimbal_pub_ = create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      cmd_gimbal_topic_, servo_control_pkg::qos::control_cmd());

    raw_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      raw_cmd_vel_topic_, servo_control_pkg::qos::control_cmd(),
      std::bind(&CommandSafetyFilterNode::raw_cmd_vel_callback, this, std::placeholders::_1));
    raw_gimbal_sub_ = create_subscription<vision_servo_msgs::msg::GimbalCmd>(
      raw_gimbal_topic_, servo_control_pkg::qos::control_cmd(),
      std::bind(&CommandSafetyFilterNode::raw_gimbal_callback, this, std::placeholders::_1));
    platform_sub_ = create_subscription<vision_servo_msgs::msg::PlatformState>(
      platform_state_topic_, servo_control_pkg::qos::platform_state(),
      std::bind(&CommandSafetyFilterNode::platform_callback, this, std::placeholders::_1));
    reference_sub_ = create_subscription<vision_servo_msgs::msg::ShotReference>(
      reference_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&CommandSafetyFilterNode::reference_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&CommandSafetyFilterNode::update, this));

    RCLCPP_INFO(
      get_logger(), "Command safety filter started: %s, %s",
      cmd_vel_topic_.c_str(), cmd_gimbal_topic_.c_str());
  }

private:
  void raw_cmd_vel_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    latest_raw_cmd_vel_ = *msg;
    raw_cmd_vel_time_ = now();
    has_raw_cmd_vel_ = true;
  }

  void raw_gimbal_callback(const vision_servo_msgs::msg::GimbalCmd::ConstSharedPtr msg)
  {
    latest_raw_gimbal_ = *msg;
    raw_gimbal_time_ = now();
    has_raw_gimbal_ = true;
  }

  void platform_callback(const vision_servo_msgs::msg::PlatformState::ConstSharedPtr msg)
  {
    latest_platform_ = *msg;
    platform_time_ = now();
    has_platform_ = true;
  }

  void reference_callback(const vision_servo_msgs::msg::ShotReference::ConstSharedPtr msg)
  {
    latest_reference_ = *msg;
    reference_time_ = now();
    has_reference_ = true;
  }

  void update()
  {
    const auto t = now();
    // TODO: Add proximity/obstacle safety gates before allowing final base motion.
    const bool platform_ok = has_platform_ &&
      (t - platform_time_).seconds() <= platform_timeout_sec_ &&
      !latest_platform_.emergency_stop;
    const bool reference_ok = has_reference_ &&
      (t - reference_time_).seconds() <= reference_timeout_sec_ &&
      latest_reference_.valid;
    const bool raw_base_ok = has_raw_cmd_vel_ &&
      (t - raw_cmd_vel_time_).seconds() <= raw_timeout_sec_;
    const bool raw_gimbal_ok = has_raw_gimbal_ &&
      (t - raw_gimbal_time_).seconds() <= raw_timeout_sec_;

    if (!platform_ok || !reference_ok) {
      publish_safe_stop(t);
      return;
    }

    publish_base(t, raw_base_ok);
    publish_gimbal(t, raw_gimbal_ok);
  }

  void publish_base(const rclcpp::Time & stamp, bool raw_ok)
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = base_frame_;

    if (raw_ok && latest_reference_.base_motion_enabled) {
      const double vx_limit = latest_reference_.max_base_speed > 0.0f ?
        std::min(max_vx_, static_cast<double>(latest_reference_.max_base_speed)) : max_vx_;
      const double wz_limit = latest_reference_.max_base_yaw_rate > 0.0f ?
        std::min(max_wz_, static_cast<double>(latest_reference_.max_base_yaw_rate)) : max_wz_;
      cmd.twist.linear.x = clamp(latest_raw_cmd_vel_.twist.linear.x, -vx_limit, vx_limit);
      cmd.twist.linear.y = clamp(latest_raw_cmd_vel_.twist.linear.y, -max_vy_, max_vy_);
      cmd.twist.angular.z = clamp(latest_raw_cmd_vel_.twist.angular.z, -wz_limit, wz_limit);
    }

    cmd_vel_pub_->publish(cmd);
  }

  void publish_gimbal(const rclcpp::Time & stamp, bool raw_ok)
  {
    vision_servo_msgs::msg::GimbalCmd cmd;
    cmd.header.stamp = stamp;
    cmd.hold_yaw = true;
    cmd.hold_pitch = true;

    if (raw_ok && latest_reference_.gimbal_tracking_enabled) {
      const double yaw_limit = latest_reference_.max_gimbal_yaw_rate > 0.0f ?
        std::min(max_gimbal_yaw_rate_, static_cast<double>(latest_reference_.max_gimbal_yaw_rate)) :
        max_gimbal_yaw_rate_;
      const double pitch_limit = latest_reference_.max_gimbal_pitch_rate > 0.0f ?
        std::min(max_gimbal_pitch_rate_, static_cast<double>(latest_reference_.max_gimbal_pitch_rate)) :
        max_gimbal_pitch_rate_;
      cmd.yaw_rate = static_cast<float>(
        clamp(latest_raw_gimbal_.yaw_rate, -yaw_limit, yaw_limit));
      cmd.pitch_rate = static_cast<float>(
        clamp(latest_raw_gimbal_.pitch_rate, -pitch_limit, pitch_limit));
      cmd.hold_yaw = latest_raw_gimbal_.hold_yaw;
      cmd.hold_pitch = latest_raw_gimbal_.hold_pitch;
    }

    cmd_gimbal_pub_->publish(cmd);
  }

  void publish_safe_stop(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TwistStamped base_cmd;
    base_cmd.header.stamp = stamp;
    base_cmd.header.frame_id = base_frame_;
    cmd_vel_pub_->publish(base_cmd);

    vision_servo_msgs::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header.stamp = stamp;
    gimbal_cmd.hold_yaw = true;
    gimbal_cmd.hold_pitch = true;
    cmd_gimbal_pub_->publish(gimbal_cmd);
  }

  std::string raw_cmd_vel_topic_;
  std::string raw_gimbal_topic_;
  std::string platform_state_topic_;
  std::string reference_topic_;
  std::string cmd_vel_topic_;
  std::string cmd_gimbal_topic_;
  std::string base_frame_;
  double update_rate_hz_{50.0};
  double raw_timeout_sec_{0.25};
  double reference_timeout_sec_{0.5};
  double platform_timeout_sec_{0.5};
  double max_vx_{0.8};
  double max_vy_{0.5};
  double max_wz_{1.2};
  double max_gimbal_yaw_rate_{1.5};
  double max_gimbal_pitch_rate_{1.0};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr cmd_gimbal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr raw_cmd_vel_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalCmd>::SharedPtr raw_gimbal_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::PlatformState>::SharedPtr platform_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::ShotReference>::SharedPtr reference_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_raw_cmd_vel_{false};
  bool has_raw_gimbal_{false};
  bool has_platform_{false};
  bool has_reference_{false};
  geometry_msgs::msg::TwistStamped latest_raw_cmd_vel_;
  vision_servo_msgs::msg::GimbalCmd latest_raw_gimbal_;
  vision_servo_msgs::msg::PlatformState latest_platform_;
  vision_servo_msgs::msg::ShotReference latest_reference_;
  rclcpp::Time raw_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time raw_gimbal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time platform_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reference_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandSafetyFilterNode>());
  rclcpp::shutdown();
  return 0;
}
