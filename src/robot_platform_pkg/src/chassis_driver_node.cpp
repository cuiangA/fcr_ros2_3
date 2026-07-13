/**
 * @file chassis_driver_node.cpp
 * @brief 底盘驱动节点 — LEKIWI 三全向轮底盘的速度控制与状态发布。
 *
 * 该节点是底盘控制的核心：
 *   1. 订阅 /cmd_vel (TwistStamped)，接收上层控制指令
 *   2. 对速度指令进行安全限幅（钳位到 max_linear/angular_velocity）
 *   3. 通过 IChassisInterface 将指令发送给底层硬件
 *   4. 以 50 Hz 频率发布底盘原始反馈（/chassis/odom_raw）
 *
 * 安全机制：
 *   - 速度钳位：防止超速飞车
 *   - 紧急停止：estop_active 时锁定电机输出
 *   - 超时保护：长时间未收到指令时自动发送零轮速
 *
 * 支持真实/仿真双模式，通过 use_sim 参数切换。
 */

#include "robot_platform_pkg/hardware_interfaces/chassis_interface.hpp"
#include "robot_platform_pkg/utils/math_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <stdexcept>
#include <chrono>

namespace robot_platform_pkg {

class ChassisDriverNode : public rclcpp::Node {
public:
  explicit ChassisDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("chassis_driver", options)
  {
    // ── 1. 声明参数 ────────────────────────────────────────────────
    this->declare_parameter("use_sim", false);
    this->declare_parameter("serial_device", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 1000000);
    this->declare_parameter("max_linear_velocity", 1.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("wheel_radius", 0.05);
    this->declare_parameter("base_radius", 0.125);
    this->declare_parameter("left_wheel_id", 7);
    this->declare_parameter("back_wheel_id", 8);
    this->declare_parameter("right_wheel_id", 9);
    this->declare_parameter("max_raw_wheel_velocity", 3000);
    this->declare_parameter("command_timeout_ms", 250);
    this->declare_parameter("enable_emergency_stop", true);

    bool use_sim = this->get_parameter("use_sim").as_bool();
    max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
    enable_estop_ = this->get_parameter("enable_emergency_stop").as_bool();
    std::string device = this->get_parameter("serial_device").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    command_timeout_ = std::chrono::milliseconds(
      this->get_parameter("command_timeout_ms").as_int());
    if (command_timeout_.count() <= 0) {
      throw std::invalid_argument("command_timeout_ms must be greater than zero");
    }

    // ── 2. 创建底盘硬件接口（真实或仿真） ──────────────────────────
    if (use_sim) {
      chassis_ = make_simulated_chassis();       // 仿真模式，无需硬件
    } else {
      LekiwiChassisConfig config;
      config.left_wheel_id = this->get_parameter("left_wheel_id").as_int();
      config.back_wheel_id = this->get_parameter("back_wheel_id").as_int();
      config.right_wheel_id = this->get_parameter("right_wheel_id").as_int();
      config.wheel_radius = this->get_parameter("wheel_radius").as_double();
      config.base_radius = this->get_parameter("base_radius").as_double();
      config.max_raw_velocity = this->get_parameter("max_raw_wheel_velocity").as_int();
      chassis_ = make_lekiwi_chassis(config);     // 真实 LEKIWI 底盘
    }
    if (!chassis_) {
      throw std::runtime_error(
        "LEKIWI 真实底盘接口创建失败");
    }
    if (!chassis_->init(device, baudrate)) {
      throw std::runtime_error("底盘接口初始化失败");
    }

    // ── 4. 创建订阅者与发布者 ─────────────────────────────────────
    auto reliable_qos = rclcpp::QoS(1).reliable().durability_volatile();

    // 订阅速度指令（RELIABLE QoS = 不能丢指令）
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", reliable_qos,
      std::bind(&ChassisDriverNode::cmd_vel_callback, this, std::placeholders::_1));

    // 发布底盘原始反馈。/odom 与 TF 由 odometry_node 统一发布。
    raw_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/chassis/odom_raw", rclcpp::QoS(10).reliable());

    // ── 5. 状态发布定时器（50 Hz = 20ms 间隔） ────────────────────
    state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ChassisDriverNode::publish_state, this));
    last_command_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "底盘驱动已启动 (sim=%d, limits: v=%.1f, ω=%.1f)",
                use_sim, max_linear_vel_, max_angular_vel_);
  }

  ~ChassisDriverNode() override {
    if (chassis_) {
      chassis_->emergencyStop();
      chassis_->shutdown();
    }
  }

private:
  /**
   * @brief 速度指令回调 — 安全限幅后转发给底盘硬件。
   *
   * 限幅策略：分别对 vx, vy, ω 钳位到 [−max, +max]，
   * 保证任何上层控制器的输出都不会超出底层硬件的物理极限。
   */
  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) {
    auto clamped = msg->twist;
    // 安全钳位：防止超出底盘物理极限
    clamped.linear.x = clamp(clamped.linear.x, -max_linear_vel_, max_linear_vel_);
    clamped.linear.y = clamp(clamped.linear.y, -max_linear_vel_, max_linear_vel_);
    clamped.angular.z = clamp(clamped.angular.z, -max_angular_vel_, max_angular_vel_);

    // 急停或正常指令转发
    if (enable_estop_ && estop_active_) {
      chassis_->emergencyStop();   // 紧急停车：所有电机立即制动
    } else {
      chassis_->sendCommand(clamped);  // 正常速度指令
    }
    last_command_time_ = std::chrono::steady_clock::now();
    timeout_stop_sent_ = false;
  }

  /**
   * @brief 状态发布定时器回调（50 Hz）。
   *
   * 从底盘硬件读取原始里程计/速度反馈，填充时间戳和坐标系信息后发布。
   * 注意：该节点不发布 /odom 和 TF，避免与融合里程计节点产生双源冲突。
   */
  void publish_state() {
    const auto now_steady = std::chrono::steady_clock::now();
    if (!timeout_stop_sent_ && now_steady - last_command_time_ > command_timeout_) {
      chassis_->emergencyStop();
      timeout_stop_sent_ = true;
      RCLCPP_WARN(get_logger(), "速度指令超时，底盘已停车");
    }
    auto raw_odom = chassis_->readOdometry();
    raw_odom.header.stamp = this->now();
    raw_odom.header.frame_id = "chassis_odom";
    raw_odom.child_frame_id = "base_link";
    raw_odom_pub_->publish(raw_odom);
  }

  // ── 硬件接口与运动学 ────────────────────────────────────────────
  std::unique_ptr<IChassisInterface> chassis_;           ///< 底盘硬件接口

  // ── ROS2 通信 ────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_odom_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // ── 安全限制 ────────────────────────────────────────────────────
  double max_linear_vel_, max_angular_vel_;  ///< 速度限幅值
  bool enable_estop_, estop_active_ = false;  ///< 急停使能与状态
  std::chrono::milliseconds command_timeout_{250};
  std::chrono::steady_clock::time_point last_command_time_;
  bool timeout_stop_sent_{false};
};

}  // namespace robot_platform_pkg

/**
 * @brief 入口函数 — 初始化 ROS2、启动底盘驱动节点、完成后关闭。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_platform_pkg::ChassisDriverNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("chassis_driver"), "底盘驱动启动失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
