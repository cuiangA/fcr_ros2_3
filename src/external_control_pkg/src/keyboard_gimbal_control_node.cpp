/**
 * @file keyboard_gimbal_control_node.cpp
 * @brief Keyboard gimbal nudge publisher for manual bring-up.
 *
 * Keys:
 *   a/d: yaw left/right
 *   w/s: pitch up/down
 *   space/x: stop
 *   q: stop and quit
 */

#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace external_control_pkg {

class KeyboardGimbalControlNode : public rclcpp::Node {
public:
  explicit KeyboardGimbalControlNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("keyboard_gimbal_control_node", options)
  {
    declare_parameter("cmd_gimbal_topic", "/manual/cmd_gimbal");
    declare_parameter("publish_rate_hz", 30.0);
    declare_parameter("step_duration_sec", 0.18);
    declare_parameter("yaw_step_rate", 0.35);
    declare_parameter("pitch_step_rate", 0.25);
    declare_parameter("right_yaw_sign", 1.0);
    declare_parameter("up_pitch_sign", 1.0);
    declare_parameter("frame_id", "gimbal_link");

    cmd_gimbal_topic_ = get_parameter("cmd_gimbal_topic").as_string();
    publish_rate_hz_ = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    step_duration_sec_ = std::max(0.05, get_parameter("step_duration_sec").as_double());
    yaw_step_rate_ = std::abs(get_parameter("yaw_step_rate").as_double());
    pitch_step_rate_ = std::abs(get_parameter("pitch_step_rate").as_double());
    right_yaw_sign_ = sign(get_parameter("right_yaw_sign").as_double());
    up_pitch_sign_ = sign(get_parameter("up_pitch_sign").as_double());
    frame_id_ = get_parameter("frame_id").as_string();

    gimbal_pub_ = create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      cmd_gimbal_topic_, rclcpp::QoS(1).reliable());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&KeyboardGimbalControlNode::publishLoop, this));

    if (isatty(STDIN_FILENO)) {
      setupTerminal();
      input_thread_ = std::thread(&KeyboardGimbalControlNode::inputLoop, this);
      RCLCPP_INFO(
        get_logger(),
        "keyboard_gimbal_control_node started | a/d yaw, w/s pitch, space stop, q quit -> %s",
        cmd_gimbal_topic_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "stdin is not a TTY; keyboard control is disabled");
    }
  }

  ~KeyboardGimbalControlNode() override {
    stopped_ = true;
    publishStop();
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
    restoreTerminal();
  }

private:
  void setupTerminal() {
    if (tcgetattr(STDIN_FILENO, &old_termios_) != 0) {
      terminal_configured_ = false;
      return;
    }

    auto raw = old_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    terminal_configured_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
  }

  void restoreTerminal() {
    if (terminal_configured_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
      terminal_configured_ = false;
    }
  }

  void inputLoop() {
    while (rclcpp::ok() && !stopped_) {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(STDIN_FILENO, &read_fds);

      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;

      const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
      if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
        continue;
      }

      char key = '\0';
      if (read(STDIN_FILENO, &key, 1) == 1) {
        handleKey(key);
      }
    }
  }

  void handleKey(char key) {
    switch (key) {
    case 'a':
    case 'A':
      startNudge(-right_yaw_sign_ * yaw_step_rate_, 0.0);
      break;
    case 'd':
    case 'D':
      startNudge(right_yaw_sign_ * yaw_step_rate_, 0.0);
      break;
    case 'w':
    case 'W':
      startNudge(0.0, up_pitch_sign_ * pitch_step_rate_);
      break;
    case 's':
    case 'S':
      startNudge(0.0, -up_pitch_sign_ * pitch_step_rate_);
      break;
    case ' ':
    case 'x':
    case 'X':
      stopMotion();
      break;
    case 'q':
    case 'Q':
      stopMotion();
      rclcpp::shutdown();
      break;
    default:
      break;
    }
  }

  void startNudge(double yaw_rate, double pitch_rate) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_yaw_rate_ = yaw_rate;
    active_pitch_rate_ = pitch_rate;
    active_until_ = now() + rclcpp::Duration::from_seconds(step_duration_sec_);
    motion_active_ = true;
    publishCommandLocked(active_yaw_rate_, active_pitch_rate_);
  }

  void stopMotion() {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_active_ = false;
    publishStopLocked();
  }

  void publishLoop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!motion_active_) {
      return;
    }
    if (now() <= active_until_) {
      publishCommandLocked(active_yaw_rate_, active_pitch_rate_);
      return;
    }
    motion_active_ = false;
    publishStopLocked();
  }

  void publishCommandLocked(double yaw_rate, double pitch_rate) {
    auto cmd = vision_servo_msgs::msg::GimbalCmd();
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_id_;
    cmd.yaw_rate = static_cast<float>(yaw_rate);
    cmd.pitch_rate = static_cast<float>(pitch_rate);
    cmd.hold_yaw = std::abs(yaw_rate) < 1e-6;
    cmd.hold_pitch = std::abs(pitch_rate) < 1e-6;
    gimbal_pub_->publish(cmd);
  }

  void publishStopLocked() {
    auto cmd = vision_servo_msgs::msg::GimbalCmd();
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_id_;
    cmd.yaw_rate = 0.0f;
    cmd.pitch_rate = 0.0f;
    cmd.hold_yaw = true;
    cmd.hold_pitch = true;
    gimbal_pub_->publish(cmd);
  }

  void publishStop() {
    if (!gimbal_pub_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    publishStopLocked();
  }

  static double sign(double value) {
    return value >= 0.0 ? 1.0 : -1.0;
  }

  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread input_thread_;
  std::atomic<bool> stopped_{false};

  mutable std::mutex mutex_;
  std::string cmd_gimbal_topic_;
  std::string frame_id_;
  double publish_rate_hz_ = 30.0;
  double step_duration_sec_ = 0.18;
  double yaw_step_rate_ = 0.35;
  double pitch_step_rate_ = 0.25;
  double right_yaw_sign_ = 1.0;
  double up_pitch_sign_ = 1.0;
  bool motion_active_ = false;
  double active_yaw_rate_ = 0.0;
  double active_pitch_rate_ = 0.0;
  rclcpp::Time active_until_{0, 0, RCL_ROS_TIME};

  termios old_termios_{};
  bool terminal_configured_ = false;
};

}  // namespace external_control_pkg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<external_control_pkg::KeyboardGimbalControlNode>());
  rclcpp::shutdown();
  return 0;
}
