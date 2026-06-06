/**
 * @file mvp_follow_controller_node.cpp
 * @brief Stage-1 MVP follower: gimbal tracks image error, base tracks depth and gimbal yaw.
 */

#include "servo_control_pkg/qos.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <vision_servo_msgs/msg/platform_state.hpp>
#include <vision_servo_msgs/msg/target.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace servo_control_pkg {

namespace {

using vision_servo_msgs::msg::Target;

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

double clamp_unit_alpha(double alpha)
{
  return std::clamp(alpha, 0.0, 1.0);
}

double low_pass(double current, double last, double alpha)
{
  alpha = clamp_unit_alpha(alpha);
  return alpha * current + (1.0 - alpha) * last;
}

double apply_deadband(double value, double deadband)
{
  return std::abs(value) < deadband ? 0.0 : value;
}

bool bbox_looks_normalized(const Target& target)
{
  return std::all_of(target.bbox.begin(), target.bbox.end(), [](float value) {
    return std::isfinite(value) && value >= 0.0f && value <= 1.0f;
  });
}

}  // namespace

class MvpFollowControllerNode final : public rclcpp::Node
{
public:
  explicit MvpFollowControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("mvp_follow_controller_node", options)
  {
    load_parameters();

    target_sub_ = create_subscription<vision_servo_msgs::msg::TargetArray>(
      target_topic_, qos::control_cmd(),
      std::bind(&MvpFollowControllerNode::target_callback, this, std::placeholders::_1));

    platform_sub_ = create_subscription<vision_servo_msgs::msg::PlatformState>(
      platform_state_topic_, qos::platform_state(),
      std::bind(&MvpFollowControllerNode::platform_state_callback, this, std::placeholders::_1));

    if (use_twist_stamped_) {
      cmd_vel_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_, qos::control_cmd());
    } else {
      cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, qos::control_cmd());
    }

    cmd_gimbal_pub_ = create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      cmd_gimbal_topic_, qos::control_cmd());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0)));
    control_timer_ = create_wall_timer(
      period, std::bind(&MvpFollowControllerNode::control_loop, this));

    last_valid_target_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "MVP follow controller started: target=%s, platform=%s, cmd_vel=%s (%s), cmd_gimbal=%s",
      target_topic_.c_str(), platform_state_topic_.c_str(), cmd_vel_topic_.c_str(),
      use_twist_stamped_ ? "TwistStamped" : "Twist", cmd_gimbal_topic_.c_str());
  }

  ~MvpFollowControllerNode() override
  {
    publish_zero_command();
  }

private:
  struct ControlCommand
  {
    double base_vx = 0.0;
    double base_wz = 0.0;
    double gimbal_yaw_vel = 0.0;
    double gimbal_pitch_vel = 0.0;
  };

  template <typename T>
  T declare_described_parameter(
    const std::string& name, const T& default_value, const std::string& description)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = description;
    return declare_parameter<T>(name, default_value, descriptor);
  }

  void load_parameters()
  {
    target_topic_ = declare_described_parameter<std::string>(
      "target_topic", "/target/current", "TargetArray input topic for the MVP controller.");
    platform_state_topic_ = declare_described_parameter<std::string>(
      "platform_state_topic", "/platform/state", "PlatformState topic carrying gimbal angles.");
    cmd_vel_topic_ = declare_described_parameter<std::string>(
      "cmd_vel_topic", "/cmd_vel", "Base velocity command topic.");
    cmd_gimbal_topic_ = declare_described_parameter<std::string>(
      "cmd_gimbal_topic", "/cmd_gimbal", "Gimbal velocity command topic.");

    use_twist_stamped_ = declare_described_parameter<bool>(
      "use_twist_stamped", true, "Publish geometry_msgs/TwistStamped on cmd_vel when true.");
    use_mock_gimbal_state_ = declare_described_parameter<bool>(
      "use_mock_gimbal_state", true, "Use mock_q_yaw/mock_q_pitch instead of PlatformState.");

    image_width_ = declare_described_parameter<int>(
      "image_width", 640, "Input image width in pixels.");
    image_height_ = declare_described_parameter<int>(
      "image_height", 480, "Input image height in pixels.");
    control_rate_hz_ = declare_described_parameter<double>(
      "control_rate_hz", 50.0, "Control loop frequency in Hz.");

    desired_distance_ = declare_described_parameter<double>(
      "desired_distance", 2.0, "Desired target distance in meters.");

    k_gimbal_x_ = declare_described_parameter<double>(
      "K_gimbal_x", 0.8, "Gimbal yaw proportional gain on normalized image x error.");
    k_gimbal_y_ = declare_described_parameter<double>(
      "K_gimbal_y", 0.6, "Gimbal pitch proportional gain on normalized image y error.");
    k_base_z_ = declare_described_parameter<double>(
      "K_base_z", 0.4, "Base forward proportional gain on depth error.");
    k_base_yaw_ = declare_described_parameter<double>(
      "K_base_yaw", 0.5, "Base yaw proportional gain on gimbal yaw angle.");
    use_ibvs_gimbal_ = declare_described_parameter<bool>(
      "use_ibvs_gimbal", true, "Use point-feature IBVS for gimbal yaw/pitch commands.");
    ibvs_gimbal_gain_ = declare_described_parameter<double>(
      "ibvs_gimbal_gain", 0.8, "IBVS gain used by the MVP gimbal-only controller.");
    ibvs_damping_ = declare_described_parameter<double>(
      "ibvs_damping", 0.01, "Damping coefficient for the gimbal IBVS pseudo-inverse.");
    camera_fx_ = declare_described_parameter<double>(
      "camera_fx", 0.0, "Camera fx in pixels; <=0 falls back to image_width / 2.");
    camera_fy_ = declare_described_parameter<double>(
      "camera_fy", 0.0, "Camera fy in pixels; <=0 falls back to image_height / 2.");
    camera_cx_ = declare_described_parameter<double>(
      "camera_cx", 0.0, "Camera principal point x; <=0 falls back to image_width / 2.");
    camera_cy_ = declare_described_parameter<double>(
      "camera_cy", 0.0, "Camera principal point y; <=0 falls back to image_height / 2.");

    ex_deadband_ = declare_described_parameter<double>(
      "ex_deadband", 0.05, "Deadband for normalized image x error.");
    ey_deadband_ = declare_described_parameter<double>(
      "ey_deadband", 0.05, "Deadband for normalized image y error.");
    depth_deadband_ = declare_described_parameter<double>(
      "depth_deadband", 0.2, "Deadband for depth error in meters.");
    q_yaw_deadband_ = declare_described_parameter<double>(
      "q_yaw_deadband", 0.0873, "Deadband for gimbal yaw angle in radians.");

    max_gimbal_yaw_vel_ = declare_described_parameter<double>(
      "max_gimbal_yaw_vel", 0.8, "Maximum absolute gimbal yaw velocity in rad/s.");
    max_gimbal_pitch_vel_ = declare_described_parameter<double>(
      "max_gimbal_pitch_vel", 0.6, "Maximum absolute gimbal pitch velocity in rad/s.");
    max_base_vx_ = declare_described_parameter<double>(
      "max_base_vx", 0.4, "Maximum absolute base forward velocity in m/s.");
    max_base_wz_ = declare_described_parameter<double>(
      "max_base_wz", 0.6, "Maximum absolute base yaw velocity in rad/s.");

    filter_alpha_error_ = declare_described_parameter<double>(
      "filter_alpha_error", 0.5, "Low-pass alpha for image errors.");
    filter_alpha_depth_ = declare_described_parameter<double>(
      "filter_alpha_depth", 0.3, "Low-pass alpha for target depth.");
    filter_alpha_cmd_ = declare_described_parameter<double>(
      "filter_alpha_cmd", 0.3, "Low-pass alpha for output commands.");

    target_timeout_ = declare_described_parameter<double>(
      "target_timeout", 0.5, "Seconds before the target is treated as lost.");
    mock_q_yaw_ = declare_described_parameter<double>(
      "mock_q_yaw", 0.0, "Mock gimbal yaw angle in radians.");
    mock_q_pitch_ = declare_described_parameter<double>(
      "mock_q_pitch", 0.0, "Mock gimbal pitch angle in radians.");
  }

  void target_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr msg)
  {
    const auto selected = select_target(*msg);
    if (!selected || !is_valid_target(*selected)) {
      has_valid_target_ = false;
      return;
    }

    active_target_ = *selected;
    has_valid_target_ = true;
    last_valid_target_time_ = now();
  }

  void platform_state_callback(const vision_servo_msgs::msg::PlatformState::ConstSharedPtr msg)
  {
    platform_state_received_ = true;
    platform_q_yaw_ = msg->gimbal_yaw;
    platform_q_pitch_ = msg->gimbal_pitch;
  }

  std::optional<Target> select_target(const vision_servo_msgs::msg::TargetArray& msg) const
  {
    if (msg.targets.empty()) {
      return std::nullopt;
    }

    if (msg.tracking_id >= 0) {
      const auto tracked = std::find_if(
        msg.targets.begin(), msg.targets.end(),
        [&msg](const Target& target) { return target.id == msg.tracking_id; });
      if (tracked != msg.targets.end()) {
        return *tracked;
      }
    }

    return *std::max_element(
      msg.targets.begin(), msg.targets.end(),
      [](const Target& lhs, const Target& rhs) {
        return lhs.confidence < rhs.confidence;
      });
  }

  bool is_valid_target(const Target& target) const
  {
    const double depth = target.position[2];
    const auto center = target_center_pixels(target);
    return finite_positive(depth) &&
      std::isfinite(center.first) &&
      std::isfinite(center.second);
  }

  std::pair<double, double> target_center_pixels(const Target& target) const
  {
    double cx = target.center[0];
    double cy = target.center[1];

    if ((!std::isfinite(cx) || !std::isfinite(cy)) && std::isfinite(target.bbox[0])) {
      cx = 0.5 * (target.bbox[0] + target.bbox[2]);
      cy = 0.5 * (target.bbox[1] + target.bbox[3]);
    }

    if (bbox_looks_normalized(target) &&
        cx >= 0.0 && cx <= 1.0 && cy >= 0.0 && cy <= 1.0) {
      cx *= static_cast<double>(image_width_);
      cy *= static_cast<double>(image_height_);
    }

    return {cx, cy};
  }

  void control_loop()
  {
    const auto current_time = now();
    if (!has_recent_target(current_time)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MVP target lost or invalid; publishing zero base and gimbal commands");
      reset_command_filters();
      publish_zero_command();
      return;
    }

    const auto command = compute_control();
    publish_command(command);
  }

  bool has_recent_target(const rclcpp::Time& current_time) const
  {
    if (!has_valid_target_) {
      return false;
    }
    return (current_time - last_valid_target_time_).seconds() <= target_timeout_;
  }

  ControlCommand compute_control()
  {
    const auto center = target_center_pixels(active_target_);
    const double half_width = 0.5 * static_cast<double>(image_width_);
    const double half_height = 0.5 * static_cast<double>(image_height_);

    const double ex_raw = (center.first - half_width) / half_width;
    const double ey_raw = (center.second - half_height) / half_height;
    const double depth_raw = active_target_.position[2];

    if (!error_filter_initialized_) {
      filtered_ex_ = ex_raw;
      filtered_ey_ = ey_raw;
      error_filter_initialized_ = true;
    } else {
      filtered_ex_ = low_pass(ex_raw, filtered_ex_, filter_alpha_error_);
      filtered_ey_ = low_pass(ey_raw, filtered_ey_, filter_alpha_error_);
    }

    if (!depth_filter_initialized_) {
      filtered_depth_ = depth_raw;
      depth_filter_initialized_ = true;
    } else {
      filtered_depth_ = low_pass(depth_raw, filtered_depth_, filter_alpha_depth_);
    }

    const double ex = apply_deadband(filtered_ex_, ex_deadband_);
    const double ey = apply_deadband(filtered_ey_, ey_deadband_);
    const double ez = apply_deadband(filtered_depth_ - desired_distance_, depth_deadband_);
    const double q_yaw = current_gimbal_yaw();

    double base_vx = std::clamp(k_base_z_ * ez, -max_base_vx_, max_base_vx_);
    double base_wz = 0.0;
    if (std::abs(q_yaw) >= q_yaw_deadband_) {
      base_wz = std::clamp(k_base_yaw_ * q_yaw, -max_base_wz_, max_base_wz_);
    }

    const auto gimbal_command = use_ibvs_gimbal_
      ? compute_ibvs_gimbal_velocity(center.first, center.second)
      : compute_proportional_gimbal_velocity(ex, ey);
    double gimbal_yaw_vel = gimbal_command.first;
    double gimbal_pitch_vel = gimbal_command.second;

    filtered_base_vx_ = low_pass(base_vx, filtered_base_vx_, filter_alpha_cmd_);
    filtered_base_wz_ = low_pass(base_wz, filtered_base_wz_, filter_alpha_cmd_);
    filtered_gimbal_yaw_vel_ = low_pass(
      gimbal_yaw_vel, filtered_gimbal_yaw_vel_, filter_alpha_cmd_);
    filtered_gimbal_pitch_vel_ = low_pass(
      gimbal_pitch_vel, filtered_gimbal_pitch_vel_, filter_alpha_cmd_);

    ControlCommand command;
    command.base_vx = std::clamp(filtered_base_vx_, -max_base_vx_, max_base_vx_);
    command.base_wz = std::clamp(filtered_base_wz_, -max_base_wz_, max_base_wz_);
    command.gimbal_yaw_vel = std::clamp(
      filtered_gimbal_yaw_vel_, -max_gimbal_yaw_vel_, max_gimbal_yaw_vel_);
    command.gimbal_pitch_vel = std::clamp(
      filtered_gimbal_pitch_vel_, -max_gimbal_pitch_vel_, max_gimbal_pitch_vel_);

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 500,
      "MVP ex=%.3f ey=%.3f depth=%.3f ez=%.3f q_yaw=%.3f | vx=%.3f wz=%.3f gyaw=%.3f gpitch=%.3f",
      ex, ey, filtered_depth_, ez, q_yaw, command.base_vx, command.base_wz,
      command.gimbal_yaw_vel, command.gimbal_pitch_vel);

    return command;
  }

  std::pair<double, double> compute_proportional_gimbal_velocity(double ex, double ey) const
  {
    return {
      std::clamp(-k_gimbal_x_ * ex, -max_gimbal_yaw_vel_, max_gimbal_yaw_vel_),
      std::clamp(-k_gimbal_y_ * ey, -max_gimbal_pitch_vel_, max_gimbal_pitch_vel_)
    };
  }

  std::pair<double, double> compute_ibvs_gimbal_velocity(double pixel_x, double pixel_y) const
  {
    const double fx = camera_fx_ > 0.0 ? camera_fx_ : 0.5 * static_cast<double>(image_width_);
    const double fy = camera_fy_ > 0.0 ? camera_fy_ : 0.5 * static_cast<double>(image_height_);
    const double cx = camera_cx_ > 0.0 ? camera_cx_ : 0.5 * static_cast<double>(image_width_);
    const double cy = camera_cy_ > 0.0 ? camera_cy_ : 0.5 * static_cast<double>(image_height_);

    if (fx <= 1e-6 || fy <= 1e-6) {
      return {0.0, 0.0};
    }

    // MVP 云台只执行角速度，不让云台参与平移。因此这里取点特征交互矩阵
    // 的角速度 3 列 Lw，求解 camera angular velocity [wx, wy, wz]。
    //
    // 对单点特征 (x, y)，IBVS 关系为：
    //   s_dot = Lw * omega
    //   omega = -lambda * Lw^+ * (s - s*)
    // 期望点 s* 位于图像主点，因此误差 e = [x, y]^T。
    const double x = (pixel_x - cx) / fx;
    const double y = (pixel_y - cy) / fy;
    Eigen::Matrix<double, 2, 3> angular_interaction;
    angular_interaction <<
      x * y, -(1.0 + x * x), y,
      1.0 + y * y, -x * y, -x;

    const Eigen::Vector2d error(x, y);
    const double damping = std::max(0.0, ibvs_damping_);
    const Eigen::Matrix3d normal =
      angular_interaction.transpose() * angular_interaction +
      damping * damping * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d camera_omega =
      -ibvs_gimbal_gain_ *
      normal.ldlt().solve(angular_interaction.transpose() * error);

    // 沿用 ControlAllocator 的 optical-frame 映射：
    //   gimbal_yaw_rate   = -camera_wy
    //   gimbal_pitch_rate = -camera_wx
    // camera_wz 是光轴 roll，二轴云台在第一阶段不执行。
    const double yaw_rate = std::clamp(
      -camera_omega.y(), -max_gimbal_yaw_vel_, max_gimbal_yaw_vel_);
    const double pitch_rate = std::clamp(
      -camera_omega.x(), -max_gimbal_pitch_vel_, max_gimbal_pitch_vel_);

    return {yaw_rate, pitch_rate};
  }

  double current_gimbal_yaw()
  {
    if (use_mock_gimbal_state_) {
      return mock_q_yaw_;
    }

    if (!platform_state_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No PlatformState received yet; using mock_q_yaw fallback");
      return mock_q_yaw_;
    }

    return platform_q_yaw_;
  }

  void reset_command_filters()
  {
    filtered_base_vx_ = 0.0;
    filtered_base_wz_ = 0.0;
    filtered_gimbal_yaw_vel_ = 0.0;
    filtered_gimbal_pitch_vel_ = 0.0;
  }

  void publish_zero_command()
  {
    publish_command(ControlCommand{});
  }

  void publish_command(const ControlCommand& command)
  {
    const auto stamp = now();

    if (use_twist_stamped_ && cmd_vel_stamped_pub_) {
      auto twist_stamped = geometry_msgs::msg::TwistStamped();
      twist_stamped.header.stamp = stamp;
      twist_stamped.header.frame_id = "base_link";
      twist_stamped.twist.linear.x = command.base_vx;
      twist_stamped.twist.angular.z = command.base_wz;
      cmd_vel_stamped_pub_->publish(twist_stamped);
    } else if (cmd_vel_pub_) {
      auto twist = geometry_msgs::msg::Twist();
      twist.linear.x = command.base_vx;
      twist.angular.z = command.base_wz;
      cmd_vel_pub_->publish(twist);
    }

    auto gimbal_cmd = vision_servo_msgs::msg::GimbalCmd();
    gimbal_cmd.header.stamp = stamp;
    gimbal_cmd.yaw_rate = static_cast<float>(command.gimbal_yaw_vel);
    gimbal_cmd.pitch_rate = static_cast<float>(command.gimbal_pitch_vel);
    gimbal_cmd.hold_yaw = false;
    gimbal_cmd.hold_pitch = false;
    cmd_gimbal_pub_->publish(gimbal_cmd);
  }

  std::string target_topic_;
  std::string platform_state_topic_;
  std::string cmd_vel_topic_;
  std::string cmd_gimbal_topic_;

  bool use_twist_stamped_ = true;
  bool use_mock_gimbal_state_ = true;
  int image_width_ = 640;
  int image_height_ = 480;
  double control_rate_hz_ = 50.0;
  double desired_distance_ = 2.0;
  double k_gimbal_x_ = 0.8;
  double k_gimbal_y_ = 0.6;
  double k_base_z_ = 0.4;
  double k_base_yaw_ = 0.5;
  bool use_ibvs_gimbal_ = true;
  double ibvs_gimbal_gain_ = 0.8;
  double ibvs_damping_ = 0.01;
  double camera_fx_ = 0.0;
  double camera_fy_ = 0.0;
  double camera_cx_ = 0.0;
  double camera_cy_ = 0.0;
  double ex_deadband_ = 0.05;
  double ey_deadband_ = 0.05;
  double depth_deadband_ = 0.2;
  double q_yaw_deadband_ = 0.0873;
  double max_gimbal_yaw_vel_ = 0.8;
  double max_gimbal_pitch_vel_ = 0.6;
  double max_base_vx_ = 0.4;
  double max_base_wz_ = 0.6;
  double filter_alpha_error_ = 0.5;
  double filter_alpha_depth_ = 0.3;
  double filter_alpha_cmd_ = 0.3;
  double target_timeout_ = 0.5;
  double mock_q_yaw_ = 0.0;
  double mock_q_pitch_ = 0.0;

  Target active_target_;
  bool has_valid_target_ = false;
  rclcpp::Time last_valid_target_time_;

  bool platform_state_received_ = false;
  double platform_q_yaw_ = 0.0;
  double platform_q_pitch_ = 0.0;

  bool error_filter_initialized_ = false;
  bool depth_filter_initialized_ = false;
  double filtered_ex_ = 0.0;
  double filtered_ey_ = 0.0;
  double filtered_depth_ = 0.0;
  double filtered_base_vx_ = 0.0;
  double filtered_base_wz_ = 0.0;
  double filtered_gimbal_yaw_vel_ = 0.0;
  double filtered_gimbal_pitch_vel_ = 0.0;

  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr target_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::PlatformState>::SharedPtr platform_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr cmd_gimbal_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace servo_control_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<servo_control_pkg::MvpFollowControllerNode>());
  rclcpp::shutdown();
  return 0;
}
