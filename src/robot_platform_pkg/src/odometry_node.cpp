/**
 * @file odometry_node.cpp
 * @brief 里程计节点 — 融合轮速和 IMU 航向估计机器人位姿。
 *
 * 工作原理（航位推算 dead reckoning）：
 *   1. 接收底盘速度指令（/cmd_vel）和 IMU 数据（/imu/data）
 *   2. 利用 IMU 的偏航角（yaw）确定当前朝向
 *   3. 对速度积分 dt 得到位移增量 Δx, Δy
 *   4. 累积位移更新全局位姿（x, y, yaw）
 *   5. 按配置频率发布 /odom 和 TF（odom → base_link）
 *
 * 因为全向底盘可独立产生 X/Y 方向速度，所以需完整的三自由度
 * 航位推算（x, y, yaw），而非差速底盘的二自由度。
 *
 * @note 纯航位推算会随时间漂移（无全局校正），长期使用需结合
 *       视觉里程计或 GPS 等外部观测进行校正。
 */

#include "robot_platform_pkg/hardware_interfaces/odometry_interface.hpp"
#include "robot_platform_pkg/kinematics/three_wheel_omni_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace robot_platform_pkg {

class OdometryNode : public rclcpp::Node {
public:
  explicit OdometryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("odometry_node", options),
      x_(0), y_(0), yaw_(0)  // 里程计原点初始化
  {
    // ── 1. 声明参数 ────────────────────────────────────────────────
    this->declare_parameter("wheel_radius", 0.05);
    this->declare_parameter("base_radius", 0.15);
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("publish_rate", 50.0);

    double wr = this->get_parameter("wheel_radius").as_double();
    double br = this->get_parameter("base_radius").as_double();
    kinematics_ = std::make_unique<ThreeWheelOmniKinematics>(wr, br);
    odometry_ = make_omni_wheel_odometry();  // 创建里程计实例

    double rate = this->get_parameter("publish_rate").as_double();

    // ── 2. 订阅速度指令和 IMU 数据 ────────────────────────────────
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", rclcpp::QoS(10).reliable(),
      std::bind(&OdometryNode::cmd_vel_callback, this, std::placeholders::_1));

    // IMU 使用 SensorDataQoS (BEST_EFFORT)：允许偶尔丢帧，保证低延迟
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&OdometryNode::imu_callback, this, std::placeholders::_1));

    // ── 3. 发布者和 TF 广播器 ─────────────────────────────────────
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10).reliable());

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ── 4. 定时发布（频率由 publish_rate 参数决定） ───────────────
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&OdometryNode::publish_odom, this));

    last_time_ = this->now();  // 记录初始时刻用于 dt 计算

    RCLCPP_INFO(get_logger(), "里程计节点已启动 (%.1fHz)", rate);
  }

private:
  /// 缓存最新的速度指令
  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) {
    last_cmd_vel_ = msg->twist;
  }

  /// 缓存最新的 IMU 数据
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    last_imu_ = *msg;
  }

  /**
   * @brief 里程计发布回调 — 执行航位推算并发布结果。
   *
   * dt 计算：当前时间 - 上次更新时间。若 dt <= 0（可能由于时钟跳变），
   * 回退到 0.02s（50 Hz 的默认周期）。
   */
  void publish_odom() {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0) dt = 0.02;  // 防止时钟异常导致的零或负 dt
    last_time_ = now;

    // 执行里程计更新（航位推算）
    auto odom = odometry_->update(last_cmd_vel_, last_imu_, dt);

    // 填充时间戳和坐标系信息
    odom.header.stamp = now;
    odom.header.frame_id = this->get_parameter("odom_frame").as_string();
    odom.child_frame_id = this->get_parameter("base_frame").as_string();

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

  // ── 运动学与里程计 ──────────────────────────────────────────────
  std::unique_ptr<ThreeWheelOmniKinematics> kinematics_;
  std::unique_ptr<IOdometryInterface> odometry_;

  // ── ROS2 通信 ────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ── 状态变量 ────────────────────────────────────────────────────
  double x_, y_, yaw_;                         ///< 累积位姿（航位推算）
  geometry_msgs::msg::Twist last_cmd_vel_;     ///< 最新速度指令缓存
  sensor_msgs::msg::Imu last_imu_;             ///< 最新 IMU 数据缓存
  rclcpp::Time last_time_;                     ///< 上次更新时间
};

}  // namespace robot_platform_pkg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_platform_pkg::OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
