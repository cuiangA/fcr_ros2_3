/**
 * @file gimbal_driver_node.cpp
 * @brief DJI RS2 云台 lifecycle 驱动节点。
 *
 * 节点职责：
 *   1. lifecycle on_configure：创建并初始化云台硬件接口
 *   2. lifecycle on_activate：开始接收 /cmd_gimbal 并发布状态
 *   3. lifecycle on_deactivate/on_cleanup/on_shutdown：停止云台并释放资源
 *
 * 支持真实/仿真双模式，通过 use_sim 参数切换。
 * 真实模式使用 DJI R SDK 协议帧，并通过 SocketCAN 固定 ID 0x223/0x222 收发。
 */

#include "robot_platform_pkg/hardware_interfaces/gimbal_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <vision_servo_msgs/msg/gimbal_status.hpp>

namespace robot_platform_pkg {

class GimbalDriverNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit GimbalDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : LifecycleNode("gimbal_driver", options)
  {
    declare_parameter("autostart", true);
    declare_parameter("use_sim", false);
    declare_parameter("can_interface", "can0");
    declare_parameter("enable_command_timeout", true);
    declare_parameter("command_timeout_sec", 0.5);
    declare_parameter("max_yaw_rate", 1.0);
    declare_parameter("max_pitch_rate", 1.0);
    declare_parameter("status_publish_rate_hz", 10.0);
    declare_parameter("stop_command_burst_count", 3);
    declare_parameter("debug_position_yaw_deg", 5.0);
    declare_parameter("debug_position_pitch_deg", 0.0);
    declare_parameter("debug_position_duration_sec", 0.5);

    RCLCPP_INFO(get_logger(), "云台 lifecycle 驱动已创建，等待 configure/activate");
  }

  ~GimbalDriverNode() override
  {
    try {
      stop_and_shutdown_hardware();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "云台驱动析构清理失败: %s", e.what());
    }
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& /* previous_state */) override
  {
    RCLCPP_INFO(get_logger(), "配置云台驱动...");

    try {
      read_parameters();
      create_hardware_interface();

      auto reliable_qos = rclcpp::QoS(10).reliable();
      cmd_sub_ = create_subscription<vision_servo_msgs::msg::GimbalCmd>(
        "/cmd_gimbal", reliable_qos,
        std::bind(&GimbalDriverNode::cmd_callback, this, std::placeholders::_1));

      joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", reliable_qos);

      gimbal_status_pub_ = create_publisher<vision_servo_msgs::msg::GimbalStatus>(
        "/gimbal/status", rclcpp::QoS(10).reliable());

      debug_position_srv_ = create_service<std_srvs::srv::Trigger>(
        "/gimbal/debug_position",
        std::bind(
          &GimbalDriverNode::debug_position_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

      last_cmd_time_ = now();
      last_status_pub_time_ = now();
      has_active_cmd_ = false;

      RCLCPP_INFO(
        get_logger(),
        "云台驱动配置完成 (sim=%d, can=%s, max_yaw_rate=%.3f, max_pitch_rate=%.3f)",
        use_sim_, can_interface_.c_str(), max_yaw_rate_, max_pitch_rate_);
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "云台驱动配置失败: %s", e.what());
      reset_ros_entities();
      shutdown_hardware_only();
      return CallbackReturn::ERROR;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State& /* previous_state */) override
  {
    RCLCPP_INFO(get_logger(), "激活云台驱动...");

    if (!gimbal_ || !joint_state_pub_ || !gimbal_status_pub_) {
      RCLCPP_ERROR(get_logger(), "云台接口或发布器未配置，无法激活");
      return CallbackReturn::ERROR;
    }

    joint_state_pub_->on_activate();
    gimbal_status_pub_->on_activate();

    last_cmd_time_ = now();
    last_status_pub_time_ = now();
    has_active_cmd_ = false;

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&GimbalDriverNode::publish_state, this));

    RCLCPP_INFO(get_logger(), "云台驱动已激活");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) override
  {
    RCLCPP_INFO(get_logger(), "停用云台驱动...");

    if (state_timer_) {
      state_timer_->cancel();
      state_timer_.reset();
    }
    send_stop_command();

    if (joint_state_pub_) {
      joint_state_pub_->on_deactivate();
    }
    if (gimbal_status_pub_) {
      gimbal_status_pub_->on_deactivate();
    }

    RCLCPP_INFO(get_logger(), "云台驱动已停用");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /* previous_state */) override
  {
    RCLCPP_INFO(get_logger(), "清理云台驱动资源...");
    stop_and_shutdown_hardware();
    reset_ros_entities();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override
  {
    RCLCPP_INFO(
      get_logger(), "关闭云台驱动，来源状态: %s", previous_state.label().c_str());
    stop_and_shutdown_hardware();
    reset_ros_entities();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override
  {
    RCLCPP_ERROR(
      get_logger(), "云台驱动进入错误处理，来源状态: %s", previous_state.label().c_str());
    stop_and_shutdown_hardware();
    reset_ros_entities();
    return CallbackReturn::SUCCESS;
  }

private:
  void read_parameters()
  {
    use_sim_ = get_parameter("use_sim").as_bool();
    can_interface_ = get_parameter("can_interface").as_string();
    enable_command_timeout_ = get_parameter("enable_command_timeout").as_bool();
    command_timeout_sec_ = std::max(0.05, get_parameter("command_timeout_sec").as_double());
    max_yaw_rate_ = std::abs(get_parameter("max_yaw_rate").as_double());
    max_pitch_rate_ = std::abs(get_parameter("max_pitch_rate").as_double());

    const double status_rate =
      std::max(1.0, get_parameter("status_publish_rate_hz").as_double());
    status_publish_period_sec_ = 1.0 / status_rate;

    stop_command_burst_count_ =
      std::max(1, static_cast<int>(get_parameter("stop_command_burst_count").as_int()));
  }

  void create_hardware_interface()
  {
    if (use_sim_) {
      gimbal_ = make_simulated_gimbal();
    } else {
      gimbal_ = make_dji_rs2_gimbal();
    }

    if (!gimbal_) {
      throw std::runtime_error(
        "云台接口创建失败；请检查 make_dji_rs2_gimbal()/make_simulated_gimbal()");
    }
    if (!gimbal_->init(can_interface_)) {
      throw std::runtime_error("云台接口初始化失败");
    }
  }

  bool is_active_state()
  {
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  void debug_position_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (!is_active_state() || !gimbal_) {
      response->success = false;
      response->message = "gimbal_driver is not active";
      return;
    }

    const double yaw_deg = get_parameter("debug_position_yaw_deg").as_double();
    const double pitch_deg = get_parameter("debug_position_pitch_deg").as_double();
    const double duration_sec =
      std::max(0.1, get_parameter("debug_position_duration_sec").as_double());

    constexpr double DEG2RAD = M_PI / 180.0;
    gimbal_->sendPositionCommand(
      static_cast<float>(yaw_deg * DEG2RAD),
      static_cast<float>(pitch_deg * DEG2RAD),
      static_cast<float>(duration_sec));

    has_active_cmd_ = false;
    response->success = true;
    response->message = "sent absolute position command";
    RCLCPP_INFO(
      get_logger(),
      "已发送位置调试指令 yaw=%.2f deg, pitch=%.2f deg, duration=%.2f s",
      yaw_deg, pitch_deg, duration_sec);
  }

  /// 云台指令回调。只有 Active 状态才会真正转发给硬件。
  void cmd_callback(const vision_servo_msgs::msg::GimbalCmd::ConstSharedPtr& msg)
  {
    if (!is_active_state() || !gimbal_ || !joint_state_pub_) {
      return;
    }

    last_cmd_time_ = now();
    auto safe_cmd = clamp_command(*msg);
    has_active_cmd_ = !safe_cmd.hold_yaw || !safe_cmd.hold_pitch;
    gimbal_->sendCommand(safe_cmd);
  }

  /// 状态读取回调（100 Hz） — 从云台读取角度和角速度。
  void publish_state()
  {
    if (!is_active_state() || !gimbal_) {
      return;
    }

    enforce_command_timeout();

    float yaw = 0.0f;
    float pitch = 0.0f;
    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;
    gimbal_->readState(yaw, pitch, yaw_rate, pitch_rate);

    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = now();
    joint_state.name = {"gimbal_yaw_joint", "gimbal_pitch_joint"};
    joint_state.position = {static_cast<double>(yaw), static_cast<double>(pitch)};
    joint_state.velocity = {static_cast<double>(yaw_rate), static_cast<double>(pitch_rate)};
    joint_state_pub_->publish(joint_state);

    maybe_publish_status(yaw, pitch, yaw_rate, pitch_rate);
  }

  vision_servo_msgs::msg::GimbalCmd clamp_command(
    const vision_servo_msgs::msg::GimbalCmd& input) const
  {
    auto cmd = input;
    if (cmd.hold_yaw) {
      cmd.yaw_rate = 0.0f;
    } else {
      cmd.yaw_rate = static_cast<float>(
        std::clamp(static_cast<double>(cmd.yaw_rate), -max_yaw_rate_, max_yaw_rate_));
    }

    if (cmd.hold_pitch) {
      cmd.pitch_rate = 0.0f;
    } else {
      cmd.pitch_rate = static_cast<float>(
        std::clamp(static_cast<double>(cmd.pitch_rate), -max_pitch_rate_, max_pitch_rate_));
    }
    return cmd;
  }

  void maybe_publish_status(float yaw, float pitch, float yaw_rate, float pitch_rate)
  {
    if (!gimbal_status_pub_) {
      return;
    }

    const auto now_time = now();
    if ((now_time - last_status_pub_time_).seconds() < status_publish_period_sec_) {
      return;
    }
    last_status_pub_time_ = now_time;

    const auto snapshot = gimbal_->getStatus();
    auto status = vision_servo_msgs::msg::GimbalStatus();
    status.header.stamp = now_time;
    status.header.frame_id = "gimbal_link";
    status.yaw = yaw;
    status.pitch = pitch;
    status.yaw_rate = yaw_rate;
    status.pitch_rate = pitch_rate;
    status.connected = snapshot.connected;
    status.last_rx_age_sec = snapshot.last_rx_age_sec;
    status.tx_count = snapshot.tx_count;
    status.rx_count = snapshot.rx_count;
    status.crc_error_count = snapshot.crc_error_count;
    status.can_error_count = snapshot.can_error_count;
    status.parse_error_count = snapshot.parse_error_count;
    status.command_watchdog_enabled = enable_command_timeout_;
    status.active_command = has_active_cmd_;
    gimbal_status_pub_->publish(status);
  }

  void enforce_command_timeout()
  {
    if (!enable_command_timeout_ || !has_active_cmd_) {
      return;
    }

    const double elapsed = (now() - last_cmd_time_).seconds();
    if (elapsed <= command_timeout_sec_) {
      return;
    }

    send_stop_command();
    has_active_cmd_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "云台命令超时 %.3fs，已发送停止命令", elapsed);
  }

  void send_stop_command()
  {
    if (!gimbal_) {
      return;
    }

    auto stop = vision_servo_msgs::msg::GimbalCmd();
    stop.header.stamp = now();
    stop.header.frame_id = "gimbal_link";
    stop.yaw_rate = 0.0f;
    stop.pitch_rate = 0.0f;
    stop.hold_yaw = true;
    stop.hold_pitch = true;

    for (int i = 0; i < stop_command_burst_count_; ++i) {
      gimbal_->sendCommand(stop);
    }
    has_active_cmd_ = false;
  }

  void stop_and_shutdown_hardware()
  {
    if (!gimbal_) {
      return;
    }
    send_stop_command();
    shutdown_hardware_only();
  }

  void shutdown_hardware_only()
  {
    if (!gimbal_) {
      return;
    }
    gimbal_->shutdown();
    gimbal_.reset();
    has_active_cmd_ = false;
  }

  void reset_ros_entities()
  {
    if (state_timer_) {
      state_timer_->cancel();
      state_timer_.reset();
    }
    cmd_sub_.reset();
    debug_position_srv_.reset();
    joint_state_pub_.reset();
    gimbal_status_pub_.reset();
  }

  std::unique_ptr<IGimbalInterface> gimbal_;
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalCmd>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr debug_position_srv_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<vision_servo_msgs::msg::GimbalStatus>::SharedPtr
    gimbal_status_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_pub_time_{0, 0, RCL_ROS_TIME};
  std::string can_interface_{"can0"};
  double command_timeout_sec_ = 0.5;
  double max_yaw_rate_ = 1.0;
  double max_pitch_rate_ = 1.0;
  double status_publish_period_sec_ = 0.1;
  int stop_command_burst_count_ = 3;
  bool use_sim_ = false;
  bool enable_command_timeout_ = true;
  bool has_active_cmd_ = false;
};

}  // namespace robot_platform_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<robot_platform_pkg::GimbalDriverNode> node;
  try {
    node = std::make_shared<robot_platform_pkg::GimbalDriverNode>();

    if (node->get_parameter("autostart").as_bool()) {
      const auto configured_state = node->configure();
      if (configured_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        throw std::runtime_error("云台驱动 configure 未进入 inactive 状态");
      }

      const auto active_state = node->activate();
      if (active_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        throw std::runtime_error("云台驱动 activate 未进入 active 状态");
      }
    }

    rclcpp::spin(node->get_node_base_interface());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("gimbal_driver"), "云台驱动启动失败: %s", e.what());
    node.reset();
    rclcpp::shutdown();
    return 1;
  }

  node.reset();
  rclcpp::shutdown();
  return 0;
}
