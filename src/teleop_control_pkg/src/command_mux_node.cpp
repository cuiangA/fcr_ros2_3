#include "teleop_control_pkg/command_mux_core.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace teleop_control_pkg
{

class CommandMuxNode : public rclcpp::Node
{
public:
  CommandMuxNode() : Node("command_mux")
  {
    CommandMuxConfig config;
    config.heartbeat_timeout_ms = declare_parameter<int>("heartbeat_timeout_ms", 250);
    config.command_timeout_ms = declare_parameter<int>("command_timeout_ms", 200);
    config.zero_dwell_ms = declare_parameter<int>("zero_dwell_ms", 200);
    config.max_linear_x = declare_parameter<double>("max_linear_x", 0.05);
    config.max_linear_y = declare_parameter<double>("max_linear_y", 0.05);
    config.max_angular_z = declare_parameter<double>("max_angular_z", 0.25);
    config.max_accel_x = declare_parameter<double>("max_accel_x", 0.15);
    config.max_accel_y = declare_parameter<double>("max_accel_y", 0.15);
    config.max_accel_yaw = declare_parameter<double>("max_accel_yaw", 0.50);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    max_gimbal_yaw_rate_ = declare_parameter<double>("max_gimbal_yaw_rate", 0.25);
    max_gimbal_pitch_rate_ = declare_parameter<double>("max_gimbal_pitch_rate", 0.20);
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    const auto default_mode = declare_parameter<std::string>("default_mode", "manual");
    if (publish_rate_hz_ <= 0.0 || max_gimbal_yaw_rate_ <= 0.0 ||
      max_gimbal_pitch_rate_ <= 0.0)
    {
      throw std::invalid_argument("publish rate and gimbal limits must be positive");
    }

    core_ = std::make_unique<CommandMuxCore>(config);
    core_->set_mode(parse_mode(default_mode), steady_now_ms());
    command_timeout_ms_ = config.command_timeout_ms;

    const auto command_qos = rclcpp::QoS(1).reliable().durability_volatile();
    final_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", command_qos);
    final_gimbal_pub_ = create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      "cmd_gimbal", command_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "remote_control/status", rclcpp::QoS(10).reliable());

    manual_velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "teleop/cmd_vel", command_qos,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        core_->receive_manual_command(from_twist(*msg), steady_now_ms());
      });
    auto_velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "auto/cmd_vel", command_qos,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        core_->receive_auto_command(from_twist(*msg), steady_now_ms());
      });
    manual_gimbal_sub_ = create_subscription<vision_servo_msgs::msg::GimbalCmd>(
      "teleop/cmd_gimbal", command_qos,
      [this](vision_servo_msgs::msg::GimbalCmd::ConstSharedPtr msg) {
        manual_gimbal_ = clamp_gimbal(*msg);
        manual_gimbal_time_ms_ = steady_now_ms();
      });
    auto_gimbal_sub_ = create_subscription<vision_servo_msgs::msg::GimbalCmd>(
      "auto/cmd_gimbal", command_qos,
      [this](vision_servo_msgs::msg::GimbalCmd::ConstSharedPtr msg) {
        auto_gimbal_ = clamp_gimbal(*msg);
        auto_gimbal_time_ms_ = steady_now_ms();
      });
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
      "teleop/heartbeat", command_qos,
      [this](std_msgs::msg::Empty::ConstSharedPtr) {
        core_->receive_heartbeat(steady_now_ms());
      });
    deadman_sub_ = create_subscription<std_msgs::msg::Bool>(
      "teleop/deadman", command_qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        core_->receive_deadman(msg->data, steady_now_ms());
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "teleop/estop", command_qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data) {
          core_->latch_estop();
          RCLCPP_ERROR(get_logger(), "软件急停已锁定");
        }
      });
    clear_estop_sub_ = create_subscription<std_msgs::msg::Empty>(
      "teleop/clear_estop", command_qos,
      [this](std_msgs::msg::Empty::ConstSharedPtr) {
        if (core_->clear_estop(steady_now_ms())) {
          RCLCPP_WARN(get_logger(), "软件急停已解除，仍需重新发送运动指令");
        } else {
          RCLCPP_WARN(get_logger(), "拒绝解除急停：需心跳正常、使能释放且底盘静止");
        }
      });
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "teleop/mode", command_qos,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        try {
          core_->set_mode(parse_mode(msg->data), steady_now_ms());
          RCLCPP_WARN(get_logger(), "控制模式切换为 %s", to_string(core_->mode()));
        } catch (const std::invalid_argument & error) {
          RCLCPP_ERROR(get_logger(), "%s", error.what());
        }
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    last_tick_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CommandMuxNode::tick, this));
    RCLCPP_INFO(
      get_logger(), "安全指令仲裁已启动 (mode=%s, rate=%.1f Hz)",
      to_string(core_->mode()), publish_rate_hz_);
  }

  ~CommandMuxNode() override
  {
    publish_stop();
  }

private:
  static VelocityCommand from_twist(const geometry_msgs::msg::TwistStamped & msg)
  {
    return {msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z};
  }

  static ControlMode parse_mode(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    if (value == "manual") {return ControlMode::kManual;}
    if (value == "auto") {return ControlMode::kAuto;}
    if (value == "stop" || value == "safe_stop") {return ControlMode::kStop;}
    throw std::invalid_argument("unknown control mode: " + value);
  }

  int64_t steady_now_ms() const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  vision_servo_msgs::msg::GimbalCmd clamp_gimbal(
    const vision_servo_msgs::msg::GimbalCmd & input) const
  {
    auto output = input;
    output.yaw_rate = static_cast<float>(std::clamp(
      static_cast<double>(input.yaw_rate), -max_gimbal_yaw_rate_, max_gimbal_yaw_rate_));
    output.pitch_rate = static_cast<float>(std::clamp(
      static_cast<double>(input.pitch_rate), -max_gimbal_pitch_rate_, max_gimbal_pitch_rate_));
    return output;
  }

  vision_servo_msgs::msg::GimbalCmd selected_gimbal(
    CommandSource source, int64_t now_ms) const
  {
    vision_servo_msgs::msg::GimbalCmd command;
    command.hold_yaw = true;
    command.hold_pitch = true;
    if (source == CommandSource::kManual &&
      now_ms - manual_gimbal_time_ms_ <= command_timeout_ms_)
    {
      command = manual_gimbal_;
    } else if (source == CommandSource::kAuto &&
      now_ms - auto_gimbal_time_ms_ <= command_timeout_ms_)
    {
      command = auto_gimbal_;
    }
    command.header.stamp = now();
    return command;
  }

  void tick()
  {
    const auto tick_time = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(tick_time - last_tick_).count();
    last_tick_ = tick_time;
    const auto now_ms = steady_now_ms();
    const auto decision = core_->step(now_ms, dt);

    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = now();
    velocity.header.frame_id = frame_id_;
    velocity.twist.linear.x = decision.velocity.x;
    velocity.twist.linear.y = decision.velocity.y;
    velocity.twist.angular.z = decision.velocity.yaw;
    final_velocity_pub_->publish(velocity);
    final_gimbal_pub_->publish(selected_gimbal(decision.source, now_ms));

    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << "{\"mode\":\"" << to_string(core_->mode())
           << "\",\"active_source\":\"" << to_string(decision.source)
           << "\",\"estop\":" << (decision.estop_latched ? "true" : "false")
           << ",\"deadman\":" << (core_->deadman_active() ? "true" : "false")
           << ",\"heartbeat_age_ms\":" << core_->heartbeat_age_ms(now_ms)
           << ",\"manual_command_age_ms\":" << core_->manual_command_age_ms(now_ms)
           << ",\"auto_command_age_ms\":" << core_->auto_command_age_ms(now_ms)
           << ",\"reason\":\"" << decision.reason << "\"}";
    status.data = stream.str();
    status_pub_->publish(status);
  }

  void publish_stop()
  {
    if (!rclcpp::ok() || !final_velocity_pub_ || !final_gimbal_pub_) {return;}
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = now();
    velocity.header.frame_id = frame_id_;
    vision_servo_msgs::msg::GimbalCmd gimbal;
    gimbal.header.stamp = velocity.header.stamp;
    gimbal.hold_yaw = true;
    gimbal.hold_pitch = true;
    for (int i = 0; i < 3; ++i) {
      final_velocity_pub_->publish(velocity);
      final_gimbal_pub_->publish(gimbal);
    }
  }

  std::unique_ptr<CommandMuxCore> core_;
  double publish_rate_hz_{20.0};
  double max_gimbal_yaw_rate_{0.25};
  double max_gimbal_pitch_rate_{0.20};
  int64_t command_timeout_ms_{200};
  std::string frame_id_{"base_link"};
  int64_t manual_gimbal_time_ms_{-1000000};
  int64_t auto_gimbal_time_ms_{-1000000};
  vision_servo_msgs::msg::GimbalCmd manual_gimbal_;
  vision_servo_msgs::msg::GimbalCmd auto_gimbal_;
  std::chrono::steady_clock::time_point last_tick_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr final_velocity_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr final_gimbal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr manual_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr auto_velocity_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalCmd>::SharedPtr manual_gimbal_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalCmd>::SharedPtr auto_gimbal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deadman_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_estop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace teleop_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<teleop_control_pkg::CommandMuxNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("command_mux"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
