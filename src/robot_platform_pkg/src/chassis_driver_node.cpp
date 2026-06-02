/**
 * @file chassis_driver_node.cpp
 * @brief 底盘驱动节点 — LEKIWI 三全向轮底盘的速度控制与状态发布。
 *
 * 该节点是底盘控制的核心：
 *   1. 订阅 /cmd_vel (TwistStamped)，接收上层控制指令
 *   2. 对速度指令进行安全限幅（钳位到 max_linear/angular_velocity）
 *   3. 通过 IChassisInterface 将指令发送给底层硬件
 *   4. 以 50 Hz 频率发布里程计（/odom）和 TF 变换
 *
 * 安全机制：
 *   - 速度钳位：防止超速飞车
 *   - 紧急停止：estop_active 时锁定电机输出
 *   - 超时保护：长时间未收到指令时自动减速（由硬件接口层实现）
 *
 * 支持真实/仿真双模式，通过 use_sim 参数切换。
 */

#include "robot_platform_pkg/hardware_interfaces/chassis_interface.hpp"
#include "robot_platform_pkg/kinematics/three_wheel_omni_kinematics.hpp"
#include "robot_platform_pkg/utils/math_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

namespace robot_platform_pkg {

class ChassisDriverNode : public rclcpp::Node {
public:
  explicit ChassisDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("chassis_driver", options)
  {
    // ── 1. 声明参数 ────────────────────────────────────────────────
    this->declare_parameter("use_sim", false);
    this->declare_parameter("serial_device", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("max_linear_velocity", 1.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("wheel_radius", 0.05);
    this->declare_parameter("base_radius", 0.15);
    this->declare_parameter("enable_emergency_stop", true);

    bool use_sim = this->get_parameter("use_sim").as_bool();
    max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
    enable_estop_ = this->get_parameter("enable_emergency_stop").as_bool();

    // ── 2. 创建底盘硬件接口（真实或仿真） ──────────────────────────
    if (use_sim) {
      chassis_ = make_simulated_chassis();       // 仿真模式，无需硬件
    } else {
      chassis_ = make_lekiwi_chassis();          // 真实 LEKIWI 底盘
      std::string device = this->get_parameter("serial_device").as_string();
      int baudrate = this->get_parameter("baudrate").as_int();
      chassis_->init(device, baudrate);          // 打开串口连接
    }

    // ── 3. 初始化运动学模型 ───────────────────────────────────────
    double wr = this->get_parameter("wheel_radius").as_double();
    double br = this->get_parameter("base_radius").as_double();
    kinematics_ = std::make_unique<ThreeWheelOmniKinematics>(wr, br);

    // ── 4. 创建订阅者与发布者 ─────────────────────────────────────
    auto reliable_qos = rclcpp::QoS(10).reliable();

    // 订阅速度指令（RELIABLE QoS = 不能丢指令）
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", reliable_qos,
      std::bind(&ChassisDriverNode::cmd_vel_callback, this, std::placeholders::_1));

    // 发布里程计（RELIABLE QoS）
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10).reliable());

    // TF 广播器（将 odom → base_link 变换发布到 TF 树）
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ── 5. 状态发布定时器（50 Hz = 20ms 间隔） ────────────────────
    state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ChassisDriverNode::publish_state, this));

    RCLCPP_INFO(get_logger(), "底盘驱动已启动 (sim=%d, limits: v=%.1f, ω=%.1f)",
                use_sim, max_linear_vel_, max_angular_vel_);
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
  }

  /**
   * @brief 状态发布定时器回调（50 Hz）。
   *
   * 从底盘硬件读取里程计数据，填充时间戳和坐标系信息，
   * 同时发布 /odom 话题和 TF 变换。
   */
  void publish_state() {
    auto odom = chassis_->readOdometry();   // 从硬件读取当前里程计
    odom.header.stamp = this->now();         // ROS 时间戳
    odom.header.frame_id = "odom";           // 全局参考坐标系
    odom.child_frame_id = "base_link";       // 机器人本体坐标系
    odom_pub_->publish(odom);

    // 广播 TF：odom → base_link
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  // ── 硬件接口与运动学 ────────────────────────────────────────────
  std::unique_ptr<IChassisInterface> chassis_;           ///< 底盘硬件接口
  std::unique_ptr<ThreeWheelOmniKinematics> kinematics_; ///< 运动学模型

  // ── ROS2 通信 ────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // ── 安全限制 ────────────────────────────────────────────────────
  double max_linear_vel_, max_angular_vel_;  ///< 速度限幅值
  bool enable_estop_, estop_active_ = false;  ///< 急停使能与状态
};

}  // namespace robot_platform_pkg

/**
 * @brief 入口函数 — 初始化 ROS2、启动底盘驱动节点、完成后关闭。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_platform_pkg::ChassisDriverNode>());
  rclcpp::shutdown();
  return 0;
}
