/**
 * @file voice_gimbal_nudge_node.cpp
 * @brief Minimal bridge from VoiceCommand to short gimbal yaw nudge commands.
 *
 * This node intentionally handles only small manual gimbal nudges. It is the
 * first narrow link from natural language to actuator command:
 *   "向右一点" -> /external/voice_command -> /voice/cmd_gimbal -> command_router_node
 */

#include "external_control_pkg/msg/voice_command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace external_control_pkg {

class VoiceGimbalNudgeNode : public rclcpp::Node {
public:
  explicit VoiceGimbalNudgeNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("voice_gimbal_nudge_node", options)
  {
    declare_parameter("voice_command_topic", "/external/voice_command");
    declare_parameter("cmd_gimbal_topic", "/voice/cmd_gimbal");
    declare_parameter("publish_rate_hz", 20.0);
    declare_parameter("step_duration_sec", 0.4);
    declare_parameter("yaw_step_rate", 0.25);
    declare_parameter("pitch_step_rate", 0.20);
    declare_parameter("min_confidence", 0.5);
    declare_parameter("right_yaw_sign", 1.0);
    declare_parameter("up_pitch_sign", 1.0);
    declare_parameter("speed_scale_step", 0.2);
    declare_parameter("frame_id", "gimbal_link");

    voice_command_topic_ = get_parameter("voice_command_topic").as_string();
    cmd_gimbal_topic_ = get_parameter("cmd_gimbal_topic").as_string();
    publish_rate_hz_ = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    step_duration_sec_ = std::max(0.05, get_parameter("step_duration_sec").as_double());
    yaw_step_rate_ = std::abs(get_parameter("yaw_step_rate").as_double());
    pitch_step_rate_ = std::abs(get_parameter("pitch_step_rate").as_double());
    min_confidence_ = get_parameter("min_confidence").as_double();
    right_yaw_sign_ = sign(get_parameter("right_yaw_sign").as_double());
    up_pitch_sign_ = sign(get_parameter("up_pitch_sign").as_double());
    speed_scale_step_ = std::clamp(get_parameter("speed_scale_step").as_double(), 0.05, 0.5);
    frame_id_ = get_parameter("frame_id").as_string();

    auto reliable_qos = rclcpp::QoS(10).reliable();
    voice_sub_ = create_subscription<external_control_pkg::msg::VoiceCommand>(
      voice_command_topic_, reliable_qos,
      std::bind(&VoiceGimbalNudgeNode::voiceCallback, this, std::placeholders::_1));

    gimbal_pub_ = create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      cmd_gimbal_topic_, reliable_qos);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&VoiceGimbalNudgeNode::publishLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "voice_gimbal_nudge_node started | voice=%s -> gimbal=%s | yaw_rate=%.3f rad/s, duration=%.2fs",
      voice_command_topic_.c_str(), cmd_gimbal_topic_.c_str(),
      yaw_step_rate_, step_duration_sec_);
  }

  ~VoiceGimbalNudgeNode() override {
    publishStop();
  }

private:
  enum class NudgeDirection {
    NONE,
    LEFT,
    RIGHT,
    UP,
    DOWN,
    STOP,
    SPEED_UP,
    SPEED_DOWN,
  };

  void voiceCallback(
    const external_control_pkg::msg::VoiceCommand::ConstSharedPtr& msg)
  {
    const auto direction = classifyCommand(*msg);
    if (direction == NudgeDirection::NONE) {
      return;
    }

    if (direction == NudgeDirection::STOP) {
      motion_active_ = false;
      publishStop();
      RCLCPP_INFO(get_logger(), "voice gimbal stop | raw=\"%s\"", msg->raw_text.c_str());
      return;
    }

    if (direction == NudgeDirection::SPEED_UP || direction == NudgeDirection::SPEED_DOWN) {
      const double delta = direction == NudgeDirection::SPEED_UP
        ? speed_scale_step_
        : -speed_scale_step_;
      speed_scale_ = std::clamp(speed_scale_ + delta, 0.5, 2.0);
      RCLCPP_INFO(
        get_logger(), "voice gimbal speed scale %.2f | raw=\"%s\"",
        speed_scale_, msg->raw_text.c_str());
      return;
    }

    active_yaw_rate_ = 0.0;
    active_pitch_rate_ = 0.0;
    if (direction == NudgeDirection::RIGHT) {
      active_yaw_rate_ = right_yaw_sign_ * yaw_step_rate_ * speed_scale_;
    } else if (direction == NudgeDirection::LEFT) {
      active_yaw_rate_ = -right_yaw_sign_ * yaw_step_rate_ * speed_scale_;
    } else if (direction == NudgeDirection::UP) {
      active_pitch_rate_ = up_pitch_sign_ * pitch_step_rate_ * speed_scale_;
    } else if (direction == NudgeDirection::DOWN) {
      active_pitch_rate_ = -up_pitch_sign_ * pitch_step_rate_ * speed_scale_;
    }

    active_until_ = now() + rclcpp::Duration::from_seconds(step_duration_sec_);
    motion_active_ = true;

    publishRate(active_yaw_rate_, active_pitch_rate_);
    RCLCPP_INFO(
      get_logger(),
      "voice gimbal nudge %s | yaw_rate=%.3f, pitch_rate=%.3f rad/s, duration=%.2fs, raw=\"%s\"",
      directionName(direction).c_str(), active_yaw_rate_, active_pitch_rate_,
      step_duration_sec_, msg->raw_text.c_str());
  }

  void publishLoop() {
    if (!motion_active_) {
      return;
    }

    if (now() <= active_until_) {
      publishRate(active_yaw_rate_, active_pitch_rate_);
      return;
    }

    motion_active_ = false;
    publishStop();
  }

  NudgeDirection classifyCommand(
    const external_control_pkg::msg::VoiceCommand& msg) const
  {
    if (hasIntent(msg, {"gimbal_stop", "stop_gimbal", "stop"})) {
      return NudgeDirection::STOP;
    }
    if (hasIntent(msg, {"gimbal_nudge_right", "gimbal_right", "turn_gimbal_right"})) {
      return NudgeDirection::RIGHT;
    }
    if (hasIntent(msg, {"gimbal_nudge_left", "gimbal_left", "turn_gimbal_left"})) {
      return NudgeDirection::LEFT;
    }
    if (hasIntent(msg, {"gimbal_nudge_up", "gimbal_up", "tilt_gimbal_up"})) {
      return NudgeDirection::UP;
    }
    if (hasIntent(msg, {"gimbal_nudge_down", "gimbal_down", "tilt_gimbal_down"})) {
      return NudgeDirection::DOWN;
    }
    if (hasIntent(msg, {"gimbal_speed_up", "speed_up"})) {
      return NudgeDirection::SPEED_UP;
    }
    if (hasIntent(msg, {"gimbal_speed_down", "speed_down"})) {
      return NudgeDirection::SPEED_DOWN;
    }

    const auto& text = msg.raw_text;
    if (containsAny(text, {"停止", "停一下", "别动"})) {
      return NudgeDirection::STOP;
    }
    if (containsAny(text, {"向右一点", "向右一下", "右转一点", "右移一点", "往右一点"})) {
      return NudgeDirection::RIGHT;
    }
    if (containsAny(text, {"向左一点", "向左一下", "左转一点", "左移一点", "往左一点"})) {
      return NudgeDirection::LEFT;
    }
    if (containsAny(text, {"向上一点", "向上一下", "抬高一点", "往上一点", "上移一点"})) {
      return NudgeDirection::UP;
    }
    if (containsAny(text, {"向下一点", "向下一下", "降低一点", "往下一点", "下移一点"})) {
      return NudgeDirection::DOWN;
    }
    if (containsAny(text, {"快一点", "速度快一点", "快一些"})) {
      return NudgeDirection::SPEED_UP;
    }
    if (containsAny(text, {"慢一点", "速度慢一点", "慢一些"})) {
      return NudgeDirection::SPEED_DOWN;
    }

    return NudgeDirection::NONE;
  }

  bool hasIntent(
    const external_control_pkg::msg::VoiceCommand& msg,
    const std::vector<std::string>& candidates) const
  {
    for (size_t i = 0; i < msg.intents.size(); ++i) {
      const auto& intent = msg.intents[i];
      if (std::find(candidates.begin(), candidates.end(), intent) == candidates.end()) {
        continue;
      }

      const float confidence = i < msg.confidences.size() ? msg.confidences[i] : 1.0f;
      if (confidence >= min_confidence_) {
        return true;
      }
    }
    return false;
  }

  static bool containsAny(
    const std::string& text,
    const std::vector<std::string>& needles)
  {
    return std::any_of(needles.begin(), needles.end(),
      [&](const std::string& needle) {
        return text.find(needle) != std::string::npos;
      });
  }

  static std::string directionName(NudgeDirection direction) {
    switch (direction) {
    case NudgeDirection::LEFT:
      return "left";
    case NudgeDirection::RIGHT:
      return "right";
    case NudgeDirection::UP:
      return "up";
    case NudgeDirection::DOWN:
      return "down";
    default:
      return "none";
    }
  }

  void publishRate(double yaw_rate, double pitch_rate) {
    auto cmd = vision_servo_msgs::msg::GimbalCmd();
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_id_;
    cmd.yaw_rate = static_cast<float>(yaw_rate);
    cmd.pitch_rate = static_cast<float>(pitch_rate);
    cmd.hold_yaw = std::abs(yaw_rate) < 1e-6;
    cmd.hold_pitch = std::abs(pitch_rate) < 1e-6;
    gimbal_pub_->publish(cmd);
  }

  void publishStop() {
    if (!gimbal_pub_) {
      return;
    }
    auto cmd = vision_servo_msgs::msg::GimbalCmd();
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_id_;
    cmd.yaw_rate = 0.0f;
    cmd.pitch_rate = 0.0f;
    cmd.hold_yaw = true;
    cmd.hold_pitch = true;
    gimbal_pub_->publish(cmd);
  }

  static double sign(double value) {
    return value >= 0.0 ? 1.0 : -1.0;
  }

  rclcpp::Subscription<external_control_pkg::msg::VoiceCommand>::SharedPtr voice_sub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string voice_command_topic_;
  std::string cmd_gimbal_topic_;
  std::string frame_id_;
  double publish_rate_hz_ = 20.0;
  double step_duration_sec_ = 0.4;
  double yaw_step_rate_ = 0.25;
  double pitch_step_rate_ = 0.20;
  double min_confidence_ = 0.5;
  double right_yaw_sign_ = 1.0;
  double up_pitch_sign_ = 1.0;
  double speed_scale_step_ = 0.2;
  double speed_scale_ = 1.0;

  bool motion_active_ = false;
  double active_yaw_rate_ = 0.0;
  double active_pitch_rate_ = 0.0;
  rclcpp::Time active_until_{0, 0, RCL_ROS_TIME};
};

}  // namespace external_control_pkg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<external_control_pkg::VoiceGimbalNudgeNode>());
  rclcpp::shutdown();
  return 0;
}
