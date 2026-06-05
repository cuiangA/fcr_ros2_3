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

    bool use_sim = this->get_parameter("use_sim").as_bool();
    std::string can_if = this->get_parameter("can_interface").as_string();
    int can_id = this->get_parameter("can_id").as_int();

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

    // 状态读取定时器：100 Hz（云台需要高频反馈以保证伺服精度）
    state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&GimbalDriverNode::publish_state, this));

    RCLCPP_INFO(get_logger(), "云台驱动已启动 (sim=%d)", use_sim);
  }

private:
  /// 云台指令回调 — 直接转发给硬件接口
  void cmd_callback(const vision_servo_msgs::msg::GimbalCmd::ConstSharedPtr& msg) {
    gimbal_->sendCommand(*msg);
  }

  /// 状态读取回调（100 Hz） — 从云台读取角度和角速度
  void publish_state() {
    float yaw, pitch, yaw_rate, pitch_rate;
    gimbal_->readState(yaw, pitch, yaw_rate, pitch_rate);
    // TODO：将云台状态发布为 PlatformState 或专用话题
  }

  std::unique_ptr<IGimbalInterface> gimbal_;  ///< 云台硬件接口
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalCmd>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;   ///< 100 Hz 状态读取定时器
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
