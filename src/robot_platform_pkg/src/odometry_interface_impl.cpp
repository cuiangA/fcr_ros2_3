/**
 * @file odometry_interface_impl.cpp
 * @brief 里程计接口的工厂函数实现。
 */

#include "robot_platform_pkg/hardware_interfaces/odometry_interface.hpp"
#include <cmath>
#include <iostream>

namespace robot_platform_pkg {

// ── 全向轮里程计实现 ──────────────────────────────────────────────
class OmniWheelOdometry : public IOdometryInterface {
public:
  OmniWheelOdometry() : x_(0), y_(0), yaw_(0) {}

  nav_msgs::msg::Odometry update(
      const nav_msgs::msg::Odometry& chassis_odom_raw,
      const sensor_msgs::msg::Imu& imu,
      double dt) override {
    // 航位推算：使用底盘原始反馈中的速度，而不是直接使用上层 /cmd_vel。
    // 使用 IMU 的偏航角作为朝向参考
    const auto& chassis_twist = chassis_odom_raw.twist.twist;
    double vx = chassis_twist.linear.x * dt;
    double vy = chassis_twist.linear.y * dt;

    // 从 IMU 四元数提取偏航角
    double qw = imu.orientation.w, qx = imu.orientation.x;
    double qy = imu.orientation.y, qz = imu.orientation.z;
    if (std::abs(qw) + std::abs(qx) + std::abs(qy) + std::abs(qz) < 1e-6) {
      qw = 1.0;
      qx = qy = qz = 0.0;
    }
    yaw_ = std::atan2(2 * (qw * qz + qx * qy),
                      1 - 2 * (qy * qy + qz * qz));

    // 将底盘速度转换到世界坐标系（odom frame）
    x_ += vx * std::cos(yaw_) - vy * std::sin(yaw_);
    y_ += vx * std::sin(yaw_) + vy * std::cos(yaw_);

    // 构建里程计消息
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = imu.orientation;
    if (std::abs(imu.orientation.w) + std::abs(imu.orientation.x) +
        std::abs(imu.orientation.y) + std::abs(imu.orientation.z) < 1e-6) {
      odom.pose.pose.orientation.w = 1.0;
    }
    odom.twist.twist = chassis_twist;
    return odom;
  }

  void reset() override {
    x_ = y_ = yaw_ = 0.0;
  }

  void getPose(double& x, double& y, double& yaw) const override {
    x = x_; y = y_; yaw = yaw_;
  }

private:
  double x_, y_, yaw_;
};

std::unique_ptr<IOdometryInterface> make_omni_wheel_odometry() {
  return std::make_unique<OmniWheelOdometry>();
}

}  // namespace robot_platform_pkg
