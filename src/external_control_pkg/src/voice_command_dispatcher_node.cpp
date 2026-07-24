#include "external_control_pkg/msg/voice_command.hpp"
#include "external_control_pkg/msg/voice_dispatch_status.hpp"
#include "external_control_pkg/voice_intent_contract.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace external_control_pkg {

class VoiceCommandDispatcherNode : public rclcpp::Node {
public:
  VoiceCommandDispatcherNode()
  : Node("voice_command_dispatcher_node")
  {
    declare_parameter("input_topic", "/external/voice_command");
    declare_parameter("gimbal_topic", "/voice/gimbal_command");
    declare_parameter("chassis_topic", "/voice/chassis_command");
    declare_parameter("camera_topic", "/voice/camera_command");
    declare_parameter("autonomy_topic", "/voice/autonomy_command");
    declare_parameter("system_topic", "/voice/system_command");
    declare_parameter("status_topic", "/voice/dispatch_status");
    declare_parameter("min_confidence", 0.5);
    declare_parameter("duplicate_window_sec", 0.5);

    input_topic_ = get_parameter("input_topic").as_string();
    min_confidence_ = get_parameter("min_confidence").as_double();
    duplicate_window_sec_ = std::max(
      0.0, get_parameter("duplicate_window_sec").as_double());

    const auto qos = rclcpp::QoS(10).reliable();
    publishers_[VoiceTarget::Gimbal] =
      create_publisher<msg::VoiceCommand>(
      get_parameter("gimbal_topic").as_string(), qos);
    publishers_[VoiceTarget::Chassis] =
      create_publisher<msg::VoiceCommand>(
      get_parameter("chassis_topic").as_string(), qos);
    publishers_[VoiceTarget::Camera] =
      create_publisher<msg::VoiceCommand>(
      get_parameter("camera_topic").as_string(), qos);
    publishers_[VoiceTarget::Autonomy] =
      create_publisher<msg::VoiceCommand>(
      get_parameter("autonomy_topic").as_string(), qos);
    publishers_[VoiceTarget::System] =
      create_publisher<msg::VoiceCommand>(
      get_parameter("system_topic").as_string(), qos);
    status_pub_ = create_publisher<msg::VoiceDispatchStatus>(
      get_parameter("status_topic").as_string(), qos);

    input_sub_ = create_subscription<msg::VoiceCommand>(
      input_topic_, qos,
      std::bind(
        &VoiceCommandDispatcherNode::commandCallback, this,
        std::placeholders::_1));
    graph_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&VoiceCommandDispatcherNode::checkInputPublishers, this));

    RCLCPP_INFO(
      get_logger(),
      "voice dispatcher started | input=%s | structured intents only",
      input_topic_.c_str());
  }

private:
  static std::string commandKey(const msg::VoiceCommand& command)
  {
    std::ostringstream stream;
    stream << command.header.frame_id << '\n' << command.raw_text;
    for (const auto& intent : command.intents) {
      stream << '\n' << intent;
    }
    return stream.str();
  }

  bool isDuplicate(const msg::VoiceCommand& command)
  {
    if (duplicate_window_sec_ <= 0.0) {
      return false;
    }
    const auto key = commandKey(command);
    const auto now_time = now();
    const bool duplicate =
      key == last_command_key_ &&
      (now_time - last_command_time_).seconds() <= duplicate_window_sec_;
    last_command_key_ = key;
    last_command_time_ = now_time;
    return duplicate;
  }

  void commandCallback(const msg::VoiceCommand::ConstSharedPtr& command)
  {
    if (command->intents.empty()) {
      publishStatus(
        *command, 0, VoiceTarget::Unknown, false, "empty_intents");
      return;
    }

    if (isDuplicate(*command)) {
      for (size_t index = 0; index < command->intents.size(); ++index) {
        publishStatus(
          *command, index, VoiceTarget::Unknown, false,
          "duplicate_suppressed");
      }
      return;
    }

    std::map<VoiceTarget, msg::VoiceCommand> routed;
    for (size_t index = 0; index < command->intents.size(); ++index) {
      const auto& intent = command->intents[index];
      const float confidence =
        index < command->confidences.size() ?
        command->confidences[index] : 0.0f;

      if (isAmbiguousStopIntent(intent)) {
        publishStatus(
          *command, index, VoiceTarget::Unknown, false,
          "ambiguous_stop_requires_target");
        continue;
      }

      const auto target = intentTarget(intent);
      if (target == VoiceTarget::Unknown) {
        publishStatus(
          *command, index, target, false, "unknown_intent");
        continue;
      }
      if (confidence < min_confidence_) {
        publishStatus(
          *command, index, target, false, "confidence_below_threshold");
        continue;
      }

      auto found = routed.find(target);
      if (found == routed.end()) {
        auto filtered = *command;
        filtered.intents.clear();
        filtered.confidences.clear();
        found = routed.emplace(target, std::move(filtered)).first;
      }
      found->second.intents.push_back(intent);
      found->second.confidences.push_back(confidence);
      publishStatus(*command, index, target, true, "routed");
    }

    for (auto& entry : routed) {
      publishers_.at(entry.first)->publish(entry.second);
    }
  }

  void publishStatus(
    const msg::VoiceCommand& command,
    size_t intent_index,
    VoiceTarget target,
    bool accepted,
    const std::string& reason)
  {
    auto status = msg::VoiceDispatchStatus();
    status.header.stamp = now();
    status.header.frame_id = "voice_command_dispatcher";
    status.source = command.header.frame_id;
    status.target = targetName(target);
    status.intent =
      intent_index < command.intents.size() ?
      command.intents[intent_index] : "";
    status.confidence =
      intent_index < command.confidences.size() ?
      command.confidences[intent_index] : 0.0f;
    status.accepted = accepted;
    status.reason = reason;
    status_pub_->publish(status);
  }

  void checkInputPublishers()
  {
    const auto publisher_count = count_publishers(input_topic_);
    if (publisher_count > 1 && publisher_count != last_publisher_count_) {
      RCLCPP_WARN(
        get_logger(),
        "%zu publishers detected on %s; run only one production intent source",
        publisher_count, input_topic_.c_str());
    }
    last_publisher_count_ = publisher_count;
  }

  std::string input_topic_;
  double min_confidence_ = 0.5;
  double duplicate_window_sec_ = 0.5;
  std::string last_command_key_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  size_t last_publisher_count_ = 0;

  std::map<VoiceTarget, rclcpp::Publisher<msg::VoiceCommand>::SharedPtr>
    publishers_;
  rclcpp::Publisher<msg::VoiceDispatchStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<msg::VoiceCommand>::SharedPtr input_sub_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
};

}  // namespace external_control_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<external_control_pkg::VoiceCommandDispatcherNode>());
  rclcpp::shutdown();
  return 0;
}
