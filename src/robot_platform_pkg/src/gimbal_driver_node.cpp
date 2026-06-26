/**
 * @file gimbal_driver_node.cpp
 * @brief 云台驱动节点 — DJI RS2 云台的速度控制与状态发布。
 *
 * 该节点通过 CAN 总线与 DJI RS2 云台通信：
 *   1. 订阅 /cmd_gimbal (GimbalCmd)，接收控制指令
 *   2. 将指令通过 IGimbalInterface 转发给底层硬件
 *   3. 以 100 Hz 频率读取云台状态（角度、角速度）
 *
 * 支持真实/仿真双模式，通过 use_sim 参数切换。
 * 真实模式使用 DJIRS2Protocol 将角速度编码为 CAN 帧。
 */

#include "robot_platform_pkg/hardware_interfaces/gimbal_interface.hpp"
#include "robot_platform_pkg/utils/can_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <stdexcept>

namespace robot_platform_pkg {

class GimbalDriverNode : public rclcpp::Node {
public:
  explicit GimbalDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("gimbal_driver", options)
  {
    // ── 1. 声明参数 ────────────────────────────────────────────────
    this->declare_parameter("use_sim", false);
    this->declare_parameter("can_interface", "can0");
    // CAN ID 默认使用 DJI RS2 协议的标准地址 0x201
    this->declare_parameter("can_id", static_cast<int>(DJIRS2Protocol::DEFAULT_CAN_ID));
    this->declare_parameter("enable_command_timeout", true);
    this->declare_parameter("command_timeout_sec", 0.5);

    bool use_sim = this->get_parameter("use_sim").as_bool();
    std::string can_if = this->get_parameter("can_interface").as_string();
    int can_id = this->get_parameter("can_id").as_int();
    enable_command_timeout_ = this->get_parameter("enable_command_timeout").as_bool();
    command_timeout_sec_ = this->get_parameter("command_timeout_sec").as_double();
    last_cmd_time_ = this->now();

    // ── 2. 创建云台硬件接口（真实或仿真） ──────────────────────────
    if (use_sim) {
      gimbal_ = make_simulated_gimbal();          // 仿真模式，无需硬件
    } else {
      gimbal_ = make_dji_rs2_gimbal();            // 真实 DJI RS2 云台
    }
    if (!gimbal_) {
      throw std::runtime_error(
        "DJI RS2 真实云台接口尚未实现；请使用 use_sim:=true 或补齐 make_dji_rs2_gimbal()");
    }
    if (!gimbal_->init(can_if, can_id)) {
      throw std::runtime_error("云台接口初始化失败");
    }

    // ── 3. 创建订阅者与定时器 ─────────────────────────────────────
    auto reliable_qos = rclcpp::QoS(10).reliable();

    // 订阅云台控制指令（RELIABLE QoS：不能丢控制帧）
    cmd_sub_ = this->create_subscription<vision_servo_msgs::msg::GimbalCmd>(
      "/cmd_gimbal", reliable_qos,
      std::bind(&GimbalDriverNode::cmd_callback, this, std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", reliable_qos);

    // 状态读取定时器：100 Hz（云台需要高频反馈以保证伺服精度）
    state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&GimbalDriverNode::publish_state, this));

    RCLCPP_INFO(get_logger(), "云台驱动已启动 (sim=%d)", use_sim);
  }

  ~GimbalDriverNode() override {
    if (gimbal_) {
      send_stop_command();
      gimbal_->shutdown();
    }
  }

private:
  /// 云台指令回调 — 直接转发给硬件接口
  void cmd_callback(const vision_servo_msgs::msg::GimbalCmd::ConstSharedPtr& msg) {
    last_cmd_time_ = this->now();
    has_active_cmd_ = true;
    gimbal_->sendCommand(*msg);
  }

  /// 状态读取回调（100 Hz） — 从云台读取角度和角速度
  void publish_state() {
    enforce_command_timeout();

    float yaw, pitch, yaw_rate, pitch_rate;
    gimbal_->readState(yaw, pitch, yaw_rate, pitch_rate);

    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.header.stamp = this->now();
    joint_state.name = {"gimbal_yaw_joint", "gimbal_pitch_joint"};
    joint_state.position = {static_cast<double>(yaw), static_cast<double>(pitch)};
    joint_state.velocity = {static_cast<double>(yaw_rate), static_cast<double>(pitch_rate)};
    joint_state_pub_->publish(joint_state);

    // TODO：真实 DJI RS2 接入后，应把 SDK 连接状态/错误码也发布到诊断话题。
  }

  void enforce_command_timeout() {
    if (!enable_command_timeout_ || !has_active_cmd_) {
      return;
    }
    const double elapsed = (this->now() - last_cmd_time_).seconds();
    if (elapsed <= command_timeout_sec_) {
      return;
    }
    send_stop_command();
    has_active_cmd_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "云台命令超时 %.3fs，已发送停止命令", elapsed);
  }

  void send_stop_command() {
    if (!gimbal_) {
      return;
    }
    auto stop = vision_servo_msgs::msg::GimbalCmd();
    stop.header.stamp = this->now();
    stop.header.frame_id = "gimbal_link";
    stop.yaw_rate = 0.0f;
    stop.pitch_rate = 0.0f;
    stop.hold_yaw = true;
    stop.hold_pitch = true;
    gimbal_->sendCommand(stop);
  }

  std::unique_ptr<IGimbalInterface> gimbal_;  ///< 云台硬件接口
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;   ///< 100 Hz 状态读取定时器
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  double command_timeout_sec_ = 0.5;
  bool enable_command_timeout_ = true;
  bool has_active_cmd_ = false;
};

}  // namespace robot_platform_pkg

/**
 * @brief 入口函数 — 初始化 ROS2、启动云台驱动节点、完成后关闭。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_platform_pkg::GimbalDriverNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("gimbal_driver"), "云台驱动启动失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
