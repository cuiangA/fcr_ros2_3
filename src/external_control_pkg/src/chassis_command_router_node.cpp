#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

namespace external_control_pkg {

class ChassisCommandRouterNode : public rclcpp::Node {
public:
  ChassisCommandRouterNode()
  : Node("chassis_command_router_node")
  {
    declare_parameter("manual_cmd_topic", "/manual/cmd_vel");
    declare_parameter("voice_cmd_topic", "/voice/cmd_vel");
    declare_parameter("autonomy_cmd_topic", "/autonomy/cmd_vel");
    declare_parameter("output_cmd_topic", "/cmd_vel");
    declare_parameter("estop_topic", "/e_stop");
    declare_parameter("manual_timeout_sec", 0.5);
    declare_parameter("voice_timeout_sec", 0.5);
    declare_parameter("autonomy_timeout_sec", 0.5);
    declare_parameter("publish_rate_hz", 50.0);
    declare_parameter("frame_id", "base_link");

    manual_.name = "manual";
    voice_.name = "voice";
    autonomy_.name = "autonomy";
    manual_.timeout_sec =
      get_parameter("manual_timeout_sec").as_double();
    voice_.timeout_sec =
      get_parameter("voice_timeout_sec").as_double();
    autonomy_.timeout_sec =
      get_parameter("autonomy_timeout_sec").as_double();
    frame_id_ = get_parameter("frame_id").as_string();

    const auto qos = rclcpp::QoS(10).reliable();
    manual_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      get_parameter("manual_cmd_topic").as_string(), qos,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr command) {
        storeCommand(manual_, *command);
      });
    voice_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      get_parameter("voice_cmd_topic").as_string(), qos,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr command) {
        storeCommand(voice_, *command);
      });
    autonomy_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      get_parameter("autonomy_cmd_topic").as_string(), qos,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr command) {
        storeCommand(autonomy_, *command);
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("estop_topic").as_string(), qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr state) {
        estop_active_ = state->data;
        if (estop_active_) {
          manual_.received = false;
          voice_.received = false;
          autonomy_.received = false;
          publishStop();
        }
      });
    output_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      get_parameter("output_cmd_topic").as_string(), qos);

    const double rate =
      std::max(1.0, get_parameter("publish_rate_hz").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&ChassisCommandRouterNode::routeOnce, this));

    RCLCPP_INFO(
      get_logger(),
      "chassis router started | manual > voice > autonomy -> %s",
      get_parameter("output_cmd_topic").as_string().c_str());
  }

private:
  struct SourceState {
    std::string name;
    geometry_msgs::msg::TwistStamped command;
    rclcpp::Time last_rx{0, 0, RCL_ROS_TIME};
    double timeout_sec = 0.5;
    bool received = false;
  };

  void storeCommand(
    SourceState& source,
    const geometry_msgs::msg::TwistStamped& command)
  {
    if (estop_active_) {
      return;
    }
    source.command = command;
    source.last_rx = now();
    source.received = true;
  }

  bool isFresh(const SourceState& source, const rclcpp::Time& time) const
  {
    return source.received &&
      (time - source.last_rx).seconds() <= source.timeout_sec;
  }

  void routeOnce()
  {
    const auto time = now();
    if (estop_active_) {
      publishStop();
      last_output_was_stop_ = true;
      return;
    }

    const SourceState* selected = nullptr;
    if (isFresh(manual_, time)) {
      selected = &manual_;
    } else if (isFresh(voice_, time)) {
      selected = &voice_;
    } else if (isFresh(autonomy_, time)) {
      selected = &autonomy_;
    }

    if (selected != nullptr) {
      auto command = selected->command;
      command.header.stamp = time;
      if (command.header.frame_id.empty()) {
        command.header.frame_id = frame_id_;
      }
      output_pub_->publish(command);
      last_output_was_stop_ = isStop(command.twist);
      return;
    }

    if (!last_output_was_stop_) {
      publishStop();
      last_output_was_stop_ = true;
    }
  }

  static bool isStop(const geometry_msgs::msg::Twist& twist)
  {
    return std::abs(twist.linear.x) < 1e-6 &&
      std::abs(twist.linear.y) < 1e-6 &&
      std::abs(twist.angular.z) < 1e-6;
  }

  void publishStop()
  {
    auto command = geometry_msgs::msg::TwistStamped();
    command.header.stamp = now();
    command.header.frame_id = frame_id_;
    output_pub_->publish(command);
  }

  SourceState manual_;
  SourceState voice_;
  SourceState autonomy_;
  std::string frame_id_;
  bool estop_active_ = false;
  bool last_output_was_stop_ = true;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    manual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    voice_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    autonomy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace external_control_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<external_control_pkg::ChassisCommandRouterNode>());
  rclcpp::shutdown();
  return 0;
}
