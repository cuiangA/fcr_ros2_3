/**
 * @file velocity_commander_node.cpp
 * @brief 速度指令节点 — 订阅相机速度并将其分发为底盘 + 云台指令。
 *
 * 该节点与 servo_manager_node 的关系：
 *   servo_manager_node 直接输出底盘 Twisted 和云台 GimbalCmd（当前架构）
 *   velocity_commander_node 是备用/测试路径：接收原始相机速度，
 *   进行限幅后分发到 /cmd_vel 和 /cmd_gimbal 话题。
 *
 * 设计用途：
 *   - 测试：直接发送原始速度验证底盘/云台响应
 *   - 备用通道：控制器部署在外部进程时通过 /servo/camera_velocity 通信
 */

#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "servo_control_pkg/qos.hpp"

namespace servo_control_pkg {

class VelocityCommanderNode : public rclcpp::Node {
public:
  explicit VelocityCommanderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("velocity_commander", options)
  {
    // ── 1. 声明参数 ────────────────────────────────────────────────
    this->declare_parameter("max_linear_velocity", 1.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("gimbal_rate_limit", M_PI);  // 云台角速度限幅 (rad/s)

    max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
    gimbal_rate_limit_ = this->get_parameter("gimbal_rate_limit").as_double();

    // ── 2. 订阅者：相机速度 ────────────────────────────────────────
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/servo/camera_velocity", qos::control_cmd(),
      std::bind(&VelocityCommanderNode::velocity_callback, this, std::placeholders::_1));

    // ── 3. 发布者 ─────────────────────────────────────────────────
    chassis_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", qos::control_cmd());

    gimbal_pub_ = this->create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      "/cmd_gimbal", qos::control_cmd());

    RCLCPP_INFO(get_logger(), "速度指令节点就绪");
  }

private:
  /**
   * @brief 相机速度回调 — 限幅后发布底盘和云台指令。
   *
   * Vector3 的语义：
   *   x = 前向速度 (m/s)   → chassis_twist.linear.x
   *   y = 横向速度 (m/s)   → chassis_twist.linear.y
   *   z = 角速度 (rad/s)    → chassis_twist.angular.z
   *
   * @note 当前仅处理底盘指令，云台指令在完整实现中应从专用的 Twist 消息读取
   */
  void velocity_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr& msg) {
    auto now = this->now();

    // ── 底盘指令（平移 + 偏航） ───────────────────────────────────
    auto twist = geometry_msgs::msg::TwistStamped();
    twist.header.stamp = now;
    twist.header.frame_id = "base_link";
    // 安全限幅：防止超出底盘物理极限
    twist.twist.linear.x = std::clamp(msg->vector.x, -max_linear_vel_, max_linear_vel_);
    twist.twist.linear.y = std::clamp(msg->vector.y, -max_linear_vel_, max_linear_vel_);
    twist.twist.angular.z = std::clamp(msg->vector.z, -max_angular_vel_, max_angular_vel_);
    chassis_pub_->publish(twist);

    // ── 云台指令（旋转角速度通过专用通道发送） ────────────────────
    // TODO：在完整实现中，从专用的 CameraVelocity Twist 消息读取角速度分量
    //       当前架构中云台指令由 servo_manager_node 直接发布
  }

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr gimbal_pub_;

  double max_linear_vel_, max_angular_vel_, gimbal_rate_limit_;
};

}  // namespace servo_control_pkg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<servo_control_pkg::VelocityCommanderNode>());
  rclcpp::shutdown();
  return 0;
}
