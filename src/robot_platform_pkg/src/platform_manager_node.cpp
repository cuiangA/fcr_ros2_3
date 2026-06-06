/**
 * @file platform_manager_node.cpp
 * @brief 平台管理节点 — 聚合底盘、云台、IMU 状态为统一的 PlatformState 消息。
 *
 * 该节点是整个机器人平台的信息中枢：
 *   1. 订阅 /odom（里程计）和 /imu/data（IMU），缓存最新状态
 *   2. 以配置频率将聚合后的 PlatformState 发布到 /platform/state
 *   3. 使用 TRANSIENT_LOCAL QoS：迟加入的订阅者也能获取最新状态
 *
 * PlatformState 是下游控制器（视觉伺服、导航）的标准化输入，
 * 包含：
 *   - 底盘位姿 (x, y, yaw) 和速度 (vx, vy, ω)
 *   - 云台状态（角度、角速度）
 *   - IMU 原始数据（角速度、线加速度、方向四元数）
 *   - 各子系统连接状态标志
 *
 * 通过聚合这些信息，下游节点只需订阅一个话题即可获取完整平台状态，
 * 简化节点间的依赖关系。
 */

#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/platform_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace robot_platform_pkg {

class PlatformManagerNode : public rclcpp::Node {
public:
  explicit PlatformManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("platform_manager", options)
  {
    // ── 1. 声明参数 ────────────────────────────────────────────────
    this->declare_parameter("rate_hz", 50.0);
    double rate = this->get_parameter("rate_hz").as_double();

    auto reliable_qos = rclcpp::QoS(10).reliable();
    auto sensor_qos = rclcpp::SensorDataQoS();

    // ── 2. 订阅里程计和 IMU ───────────────────────────────────────
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", reliable_qos,
      std::bind(&PlatformManagerNode::odom_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos,  // 传感器 QoS：允许丢帧，保证低延迟
      std::bind(&PlatformManagerNode::imu_callback, this, std::placeholders::_1));

    // 订阅关节状态（用于读取云台角度）
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).reliable(),
      std::bind(&PlatformManagerNode::joint_state_callback, this, std::placeholders::_1));

    // ── 3. 发布 PlatformState（TRANSIENT_LOCAL = 迟加入者也能获取） ─
    state_pub_ = this->create_publisher<vision_servo_msgs::msg::PlatformState>(
      "/platform/state",
      rclcpp::QoS(10).reliable().transient_local());

    // ── 4. 定时发布 ───────────────────────────────────────────────
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&PlatformManagerNode::publish_state, this));

    RCLCPP_INFO(get_logger(), "平台管理节点已启动 (%.1fHz)", rate);
  }

private:
  /// 里程计回调 — 缓存最新里程计并提取偏航角
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    last_odom_ = *msg;
    // 从四元数提取偏航角（yaw）
    // 公式：yaw = atan2(2(qw*qz + qx*qy), 1 - 2(qy² + qz²))
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  }

  /// IMU 回调 — 缓存最新 IMU 数据
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    last_imu_ = *msg;
  }

  /// 关节状态回调 — 从 /joint_states 中提取云台偏航/俯仰角度
  void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "gimbal_yaw_joint") {
        gimbal_yaw_ = msg->position[i];
      } else if (msg->name[i] == "gimbal_pitch_joint") {
        gimbal_pitch_ = msg->position[i];
      }
    }
  }

  /**
   * @brief 状态发布回调 — 将缓存的底盘/IMU 状态聚合为 PlatformState 并发布。
   *
   * 连接状态标志（chassis/gimbal/imu connected）在当前实现中固定为 true。
   * 生产环境中应通过心跳检测（heartbeat）动态判断各子系统是否在线。
   */
  void publish_state() {
    vision_servo_msgs::msg::PlatformState state;
    state.header.stamp = this->now();
    state.header.frame_id = last_odom_.header.frame_id.empty()
      ? "odom" : last_odom_.header.frame_id;  // chassis_pose 位于 odom frame

    // ── 底盘状态（来自里程计） ────────────────────────────────────
    state.chassis_pose[0] = last_odom_.pose.pose.position.x;
    state.chassis_pose[1] = last_odom_.pose.pose.position.y;
    state.chassis_pose[2] = current_yaw_;                        // 偏航角 (rad)
    state.chassis_velocity[0] = last_odom_.twist.twist.linear.x;
    state.chassis_velocity[1] = last_odom_.twist.twist.linear.y;
    state.chassis_velocity[2] = last_odom_.twist.twist.angular.z;

    // ── IMU 数据 ──────────────────────────────────────────────────
    state.angular_velocity[0] = last_imu_.angular_velocity.x;
    state.angular_velocity[1] = last_imu_.angular_velocity.y;
    state.angular_velocity[2] = last_imu_.angular_velocity.z;
    state.linear_acceleration[0] = last_imu_.linear_acceleration.x;
    state.linear_acceleration[1] = last_imu_.linear_acceleration.y;
    state.linear_acceleration[2] = last_imu_.linear_acceleration.z;
    state.orientation = last_imu_.orientation;  // IMU 融合后的姿态四元数

    // ── 云台状态（来自关节状态） ──────────────────────────────────
    state.gimbal_yaw = gimbal_yaw_;
    state.gimbal_pitch = gimbal_pitch_;
    state.gimbal_yaw_rate = 0.0f;    // TODO: 从关节速度计算
    state.gimbal_pitch_rate = 0.0f;  // TODO: 从关节速度计算

    // ── 子系统连接状态 ────────────────────────────────────────────
    // TODO：生产环境中通过心跳包检测各子系统的真实连接状态
    state.chassis_connected = true;
    state.gimbal_connected = true;
    state.imu_connected = true;

    state_pub_->publish(state);
  }

  // ── 订阅者与发布者 ──────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<vision_servo_msgs::msg::PlatformState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ── 状态缓存 ────────────────────────────────────────────────────
  nav_msgs::msg::Odometry last_odom_;  ///< 最新里程计数据
  sensor_msgs::msg::Imu last_imu_;     ///< 最新 IMU 数据
  double current_yaw_ = 0.0;            ///< 从里程计四元数提取的当前偏航角 (rad)
  double gimbal_yaw_ = 0.0;             ///< 云台偏航角 (rad)，来自 /joint_states
  double gimbal_pitch_ = 0.0;           ///< 云台俯仰角 (rad)，来自 /joint_states
};

}  // namespace robot_platform_pkg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_platform_pkg::PlatformManagerNode>());
  rclcpp::shutdown();
  return 0;
}
