#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <vision_servo_msgs/msg/platform_state.hpp>
#include <vision_servo_msgs/msg/shot_reference.hpp>

#include "servo_control_pkg/qos.hpp"

namespace
{

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double wrap_pi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double limit_rate(double desired, double previous, double max_delta)
{
  return previous + clamp(desired - previous, -max_delta, max_delta);
}

}  // namespace

class BasePoseControllerNode : public rclcpp::Node
{
public:
  BasePoseControllerNode()
  : Node("base_pose_controller_node")
  {
    reference_topic_ = declare_parameter<std::string>("reference_topic", "/shot/reference");
    platform_state_topic_ = declare_parameter<std::string>("platform_state_topic", "/platform/state");
    output_topic_ = declare_parameter<std::string>("output_topic", "/control/cmd_vel_raw");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 50.0);
    kp_x_ = declare_parameter<double>("kp_x", 0.9);
    kp_y_ = declare_parameter<double>("kp_y", 0.9);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.4);
    max_vx_ = declare_parameter<double>("max_vx", 0.8);
    max_vy_ = declare_parameter<double>("max_vy", 0.5);
    max_wz_ = declare_parameter<double>("max_wz", 1.2);
    max_accel_ = declare_parameter<double>("max_accel", 1.2);
    max_yaw_accel_ = declare_parameter<double>("max_yaw_accel", 2.0);
    position_deadband_ = declare_parameter<double>("position_deadband", 0.03);
    yaw_deadband_ = declare_parameter<double>("yaw_deadband", 0.02);
    holonomic_ = declare_parameter<bool>("holonomic", true);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 0.4);

    update_rate_hz_ = std::max(update_rate_hz_, 1.0);
    max_vx_ = std::max(max_vx_, 0.0);
    max_vy_ = std::max(max_vy_, 0.0);
    max_wz_ = std::max(max_wz_, 0.0);

    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      output_topic_, servo_control_pkg::qos::control_cmd());
    reference_sub_ = create_subscription<vision_servo_msgs::msg::ShotReference>(
      reference_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&BasePoseControllerNode::reference_callback, this, std::placeholders::_1));
    platform_sub_ = create_subscription<vision_servo_msgs::msg::PlatformState>(
      platform_state_topic_, servo_control_pkg::qos::platform_state(),
      std::bind(&BasePoseControllerNode::platform_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&BasePoseControllerNode::update, this));

    RCLCPP_INFO(get_logger(), "Base pose controller started: %s", output_topic_.c_str());
  }

private:
  void reference_callback(const vision_servo_msgs::msg::ShotReference::ConstSharedPtr msg)
  {
    latest_reference_ = *msg;
    reference_received_time_ = now();
    has_reference_ = true;
  }

  void platform_callback(const vision_servo_msgs::msg::PlatformState::ConstSharedPtr msg)
  {
    latest_platform_ = *msg;
    platform_received_time_ = now();
    has_platform_ = true;
  }

  void update()
  {
    const auto t = now();
    const double dt = last_update_time_.nanoseconds() == 0 ?
      1.0 / update_rate_hz_ : (t - last_update_time_).seconds();
    last_update_time_ = t;

    if (!inputs_ready(t) || !latest_reference_.valid || !latest_reference_.base_motion_enabled) {
      publish_zero(t, dt);
      return;
    }

    const double robot_x = latest_platform_.chassis_pose[0];
    const double robot_y = latest_platform_.chassis_pose[1];
    const double robot_yaw = latest_platform_.chassis_pose[2];
    const double desired_x = latest_reference_.virtual_base_pose.position.x;
    const double desired_y = latest_reference_.virtual_base_pose.position.y;
    const double desired_yaw = tf2::getYaw(latest_reference_.virtual_base_pose.orientation);

    const double dx = desired_x - robot_x;
    const double dy = desired_y - robot_y;
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    const double ex_body = cos_yaw * dx + sin_yaw * dy;
    const double ey_body = -sin_yaw * dx + cos_yaw * dy;
    const double yaw_error = wrap_pi(desired_yaw - robot_yaw);

    double vx = std::abs(ex_body) < position_deadband_ ? 0.0 : kp_x_ * ex_body;
    double vy = std::abs(ey_body) < position_deadband_ ? 0.0 : kp_y_ * ey_body;
    double wz = std::abs(yaw_error) < yaw_deadband_ ? 0.0 : kp_yaw_ * yaw_error;

    if (!holonomic_) {
      vx += 0.25 * std::abs(ey_body);
      vy = 0.0;
    }

    const double speed_scale = clamp(latest_reference_.speed_scale, 0.0, 1.0);
    vx = clamp(vx * speed_scale, -std::min(max_vx_, static_cast<double>(latest_reference_.max_base_speed)),
      std::min(max_vx_, static_cast<double>(latest_reference_.max_base_speed)));
    vy = clamp(vy * speed_scale, -max_vy_, max_vy_);
    wz = clamp(wz, -std::min(max_wz_, static_cast<double>(latest_reference_.max_base_yaw_rate)),
      std::min(max_wz_, static_cast<double>(latest_reference_.max_base_yaw_rate)));

    const double safe_dt = dt > 0.0 && dt < 0.5 ? dt : 1.0 / update_rate_hz_;
    vx = limit_rate(vx, previous_vx_, max_accel_ * safe_dt);
    vy = limit_rate(vy, previous_vy_, max_accel_ * safe_dt);
    wz = limit_rate(wz, previous_wz_, max_yaw_accel_ * safe_dt);

    previous_vx_ = vx;
    previous_vy_ = vy;
    previous_wz_ = wz;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = t;
    cmd.header.frame_id = base_frame_;
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.angular.z = wz;
    command_pub_->publish(cmd);
  }

  bool inputs_ready(const rclcpp::Time & t) const
  {
    if (!has_reference_ || !has_platform_) {
      return false;
    }
    if ((t - reference_received_time_).seconds() > stale_timeout_sec_) {
      return false;
    }
    if ((t - platform_received_time_).seconds() > stale_timeout_sec_) {
      return false;
    }
    if (latest_platform_.emergency_stop) {
      return false;
    }
    return true;
  }

  void publish_zero(const rclcpp::Time & stamp, double dt)
  {
    const double safe_dt = dt > 0.0 && dt < 0.5 ? dt : 1.0 / update_rate_hz_;
    previous_vx_ = limit_rate(0.0, previous_vx_, max_accel_ * safe_dt);
    previous_vy_ = limit_rate(0.0, previous_vy_, max_accel_ * safe_dt);
    previous_wz_ = limit_rate(0.0, previous_wz_, max_yaw_accel_ * safe_dt);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = base_frame_;
    cmd.twist.linear.x = previous_vx_;
    cmd.twist.linear.y = previous_vy_;
    cmd.twist.angular.z = previous_wz_;
    command_pub_->publish(cmd);
  }

  std::string reference_topic_;
  std::string platform_state_topic_;
  std::string output_topic_;
  std::string base_frame_;
  double update_rate_hz_{50.0};
  double kp_x_{0.9};
  double kp_y_{0.9};
  double kp_yaw_{1.4};
  double max_vx_{0.8};
  double max_vy_{0.5};
  double max_wz_{1.2};
  double max_accel_{1.2};
  double max_yaw_accel_{2.0};
  double position_deadband_{0.03};
  double yaw_deadband_{0.02};
  bool holonomic_{true};
  double stale_timeout_sec_{0.4};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Subscription<vision_servo_msgs::msg::ShotReference>::SharedPtr reference_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::PlatformState>::SharedPtr platform_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_reference_{false};
  bool has_platform_{false};
  vision_servo_msgs::msg::ShotReference latest_reference_;
  vision_servo_msgs::msg::PlatformState latest_platform_;
  rclcpp::Time reference_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time platform_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double previous_vx_{0.0};
  double previous_vy_{0.0};
  double previous_wz_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasePoseControllerNode>());
  rclcpp::shutdown();
  return 0;
}
