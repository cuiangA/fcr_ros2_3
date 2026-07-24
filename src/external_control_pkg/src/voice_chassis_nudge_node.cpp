#include "external_control_pkg/msg/voice_command.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace external_control_pkg {

class VoiceChassisNudgeNode : public rclcpp::Node {
public:
  VoiceChassisNudgeNode()
  : Node("voice_chassis_nudge_node")
  {
    declare_parameter("voice_command_topic", "/voice/chassis_command");
    declare_parameter("cmd_vel_topic", "/voice/cmd_vel");
    declare_parameter("estop_topic", "/e_stop");
    declare_parameter("publish_rate_hz", 20.0);
    declare_parameter("step_duration_sec", 0.4);
    declare_parameter("linear_speed", 0.05);
    declare_parameter("angular_speed", 0.20);
    declare_parameter("min_confidence", 0.5);
    declare_parameter("forward_x_sign", 1.0);
    declare_parameter("left_y_sign", 1.0);
    declare_parameter("left_turn_sign", 1.0);
    declare_parameter("frame_id", "base_link");

    const auto voice_topic =
      get_parameter("voice_command_topic").as_string();
    const auto output_topic = get_parameter("cmd_vel_topic").as_string();
    publish_rate_hz_ =
      std::max(1.0, get_parameter("publish_rate_hz").as_double());
    step_duration_sec_ =
      std::max(0.05, get_parameter("step_duration_sec").as_double());
    linear_speed_ =
      std::abs(get_parameter("linear_speed").as_double());
    angular_speed_ =
      std::abs(get_parameter("angular_speed").as_double());
    min_confidence_ = get_parameter("min_confidence").as_double();
    forward_x_sign_ = sign(get_parameter("forward_x_sign").as_double());
    left_y_sign_ = sign(get_parameter("left_y_sign").as_double());
    left_turn_sign_ = sign(get_parameter("left_turn_sign").as_double());
    frame_id_ = get_parameter("frame_id").as_string();

    const auto qos = rclcpp::QoS(10).reliable();
    voice_sub_ = create_subscription<msg::VoiceCommand>(
      voice_topic, qos,
      std::bind(
        &VoiceChassisNudgeNode::voiceCallback, this,
        std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("estop_topic").as_string(), qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr state) {
        estop_active_ = state->data;
        if (estop_active_) {
          motion_active_ = false;
          publishStop();
        }
      });
    command_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, qos);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&VoiceChassisNudgeNode::publishLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "voice chassis nudge started | voice=%s -> cmd=%s | "
      "linear=%.3f m/s angular=%.3f rad/s duration=%.2fs",
      voice_topic.c_str(), output_topic.c_str(), linear_speed_,
      angular_speed_, step_duration_sec_);
  }

  ~VoiceChassisNudgeNode() override
  {
    publishStop();
  }

private:
  enum class Motion {
    None,
    Forward,
    Backward,
    Left,
    Right,
    TurnLeft,
    TurnRight,
    Stop,
  };

  bool hasIntent(
    const msg::VoiceCommand& command,
    const std::string& expected) const
  {
    for (size_t index = 0; index < command.intents.size(); ++index) {
      if (command.intents[index] != expected) {
        continue;
      }
      const float confidence =
        index < command.confidences.size() ?
        command.confidences[index] : 0.0f;
      return confidence >= min_confidence_;
    }
    return false;
  }

  Motion classify(const msg::VoiceCommand& command) const
  {
    if (hasIntent(command, "chassis_stop")) {
      return Motion::Stop;
    }
    if (hasIntent(command, "chassis_move_forward")) {
      return Motion::Forward;
    }
    if (hasIntent(command, "chassis_move_backward")) {
      return Motion::Backward;
    }
    if (hasIntent(command, "chassis_move_left")) {
      return Motion::Left;
    }
    if (hasIntent(command, "chassis_move_right")) {
      return Motion::Right;
    }
    if (hasIntent(command, "chassis_turn_left")) {
      return Motion::TurnLeft;
    }
    if (hasIntent(command, "chassis_turn_right")) {
      return Motion::TurnRight;
    }
    return Motion::None;
  }

  void voiceCallback(const msg::VoiceCommand::ConstSharedPtr& command)
  {
    if (estop_active_) {
      return;
    }
    const auto motion = classify(*command);
    if (motion == Motion::None) {
      return;
    }
    if (motion == Motion::Stop) {
      motion_active_ = false;
      publishStop();
      return;
    }

    active_command_ = geometry_msgs::msg::Twist();
    if (motion == Motion::Forward) {
      active_command_.linear.x = forward_x_sign_ * linear_speed_;
    } else if (motion == Motion::Backward) {
      active_command_.linear.x = -forward_x_sign_ * linear_speed_;
    } else if (motion == Motion::Left) {
      active_command_.linear.y = left_y_sign_ * linear_speed_;
    } else if (motion == Motion::Right) {
      active_command_.linear.y = -left_y_sign_ * linear_speed_;
    } else if (motion == Motion::TurnLeft) {
      active_command_.angular.z = left_turn_sign_ * angular_speed_;
    } else if (motion == Motion::TurnRight) {
      active_command_.angular.z = -left_turn_sign_ * angular_speed_;
    }

    active_until_ =
      now() + rclcpp::Duration::from_seconds(step_duration_sec_);
    motion_active_ = true;
    publishCommand(active_command_);
  }

  void publishLoop()
  {
    if (!motion_active_) {
      return;
    }
    if (now() <= active_until_) {
      publishCommand(active_command_);
      return;
    }
    motion_active_ = false;
    publishStop();
  }

  void publishCommand(const geometry_msgs::msg::Twist& twist)
  {
    auto command = geometry_msgs::msg::TwistStamped();
    command.header.stamp = now();
    command.header.frame_id = frame_id_;
    command.twist = twist;
    command_pub_->publish(command);
  }

  void publishStop()
  {
    if (command_pub_) {
      publishCommand(geometry_msgs::msg::Twist());
    }
  }

  static double sign(double value)
  {
    return value >= 0.0 ? 1.0 : -1.0;
  }

  rclcpp::Subscription<msg::VoiceCommand>::SharedPtr voice_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist active_command_;
  rclcpp::Time active_until_{0, 0, RCL_ROS_TIME};

  double publish_rate_hz_ = 20.0;
  double step_duration_sec_ = 0.4;
  double linear_speed_ = 0.05;
  double angular_speed_ = 0.2;
  double min_confidence_ = 0.5;
  double forward_x_sign_ = 1.0;
  double left_y_sign_ = 1.0;
  double left_turn_sign_ = 1.0;
  std::string frame_id_;
  bool motion_active_ = false;
  bool estop_active_ = false;
};

}  // namespace external_control_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<external_control_pkg::VoiceChassisNudgeNode>());
  rclcpp::shutdown();
  return 0;
}
