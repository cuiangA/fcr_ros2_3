/**
 * @file command_router_node.cpp
 * @brief Route gimbal commands by priority before they reach /cmd_gimbal.
 *
 * Priority:
 *   e_stop > manual keyboard > voice nudge > autonomy
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>

namespace external_control_pkg {

class CommandRouterNode : public rclcpp::Node {
public:
  explicit CommandRouterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("command_router_node", options)
  {
    declare_parameter("manual_cmd_topic", "/manual/cmd_gimbal");
    declare_parameter("voice_cmd_topic", "/voice/cmd_gimbal");
    declare_parameter("autonomy_cmd_topic", "/autonomy/cmd_gimbal");
    declare_parameter("output_cmd_topic", "/cmd_gimbal");
    declare_parameter("estop_topic", "/e_stop");
    declare_parameter("publish_rate_hz", 50.0);
    declare_parameter("manual_timeout_sec", 0.35);
    declare_parameter("voice_timeout_sec", 0.8);
    declare_parameter("autonomy_timeout_sec", 0.25);
    declare_parameter("frame_id", "gimbal_link");

    manual_.name = "manual";
    voice_.name = "voice";
    autonomy_.name = "autonomy";
    manual_.timeout_sec = get_parameter("manual_timeout_sec").as_double();
    voice_.timeout_sec = get_parameter("voice_timeout_sec").as_double();
    autonomy_.timeout_sec = get_parameter("autonomy_timeout_sec").as_double();
    frame_id_ = get_parameter("frame_id").as_string();

    const auto reliable_qos = rclcpp::QoS(1).reliable();

    manual_sub_ = create_subscription<vision_servo_msgs::msg::GimbalCmd>(
      get_parameter("manual_cmd_topic").as_string(), reliable_qos,
      [this](vision_servo_msgs::msg::GimbalCmd::ConstSharedPtr msg) {
        storeCommand(manual_, *msg);
      });

    voice_sub_ = create_subscription<vision_servo_msgs::msg::GimbalCmd>(
      get_parameter("voice_cmd_topic").as_string(), reliable_qos,
      [this](vision_servo_msgs::msg::GimbalCmd::ConstSharedPtr msg) {
        storeCommand(voice_, *msg);
      });

    autonomy_sub_ = create_subscription<vision_servo_msgs::msg::GimbalCmd>(
      get_parameter("autonomy_cmd_topic").as_string(), reliable_qos,
      [this](vision_servo_msgs::msg::GimbalCmd::ConstSharedPtr msg) {
        storeCommand(autonomy_, *msg);
      });

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("estop_topic").as_string(), reliable_qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        estop_active_ = msg->data;
      });

    output_pub_ = create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      get_parameter("output_cmd_topic").as_string(), reliable_qos);

    const double rate = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&CommandRouterNode::routeOnce, this));

    RCLCPP_INFO(
      get_logger(),
      "command_router_node started | manual > voice > autonomy -> %s",
      get_parameter("output_cmd_topic").as_string().c_str());
  }

private:
  struct SourceState {
    std::string name;
    vision_servo_msgs::msg::GimbalCmd command;
    rclcpp::Time last_rx{0, 0, RCL_ROS_TIME};
    double timeout_sec = 0.5;
    bool received = false;
  };

  void storeCommand(SourceState& source, const vision_servo_msgs::msg::GimbalCmd& command) {
    source.command = command;
    source.last_rx = now();
    source.received = true;
  }

  bool isFresh(const SourceState& source, const rclcpp::Time& now_time) const {
    return source.received && (now_time - source.last_rx).seconds() <= source.timeout_sec;
  }

  void routeOnce() {
    const auto now_time = now();
    if (estop_active_) {
      publishStop();
      last_output_was_stop_ = true;
      return;
    }

    const SourceState* selected = nullptr;
    if (isFresh(manual_, now_time)) {
      selected = &manual_;
    } else if (isFresh(voice_, now_time)) {
      selected = &voice_;
    } else if (isFresh(autonomy_, now_time)) {
      selected = &autonomy_;
    }

    if (selected) {
      auto command = selected->command;
      command.header.stamp = now_time;
      if (command.header.frame_id.empty()) {
        command.header.frame_id = frame_id_;
      }
      output_pub_->publish(command);
      last_output_was_stop_ = command.hold_yaw && command.hold_pitch;
      return;
    }

    if (!last_output_was_stop_) {
      publishStop();
      last_output_was_stop_ = true;
    }
  }

  void publishStop() {
    auto command = vision_servo_msgs::msg::GimbalCmd();
    command.header.stamp = now();
    command.header.frame_id = frame_id_;
    command.yaw_rate = 0.0f;
    command.pitch_rate = 0.0f;
    command.hold_yaw = true;
    command.hold_pitch = true;
    output_pub_->publish(command);
  }

  SourceState manual_;
  SourceState voice_;
  SourceState autonomy_;
  std::string frame_id_;
  bool estop_active_ = false;
  bool last_output_was_stop_ = true;

  rclcpp::Subscription<vision_servo_msgs::msg::GimbalCmd>::SharedPtr manual_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalCmd>::SharedPtr voice_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalCmd>::SharedPtr autonomy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace external_control_pkg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<external_control_pkg::CommandRouterNode>());
  rclcpp::shutdown();
  return 0;
}
