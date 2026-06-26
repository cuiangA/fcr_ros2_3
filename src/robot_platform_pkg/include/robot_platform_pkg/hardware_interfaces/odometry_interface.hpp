/**
 * @file odometry_interface.hpp
 * @brief 里程计抽象接口 — 融合轮速和 IMU 航向估计机器人位姿。
 *
 * IOdometryInterface 定义了里程计计算的统一契约。
 * 输入为底盘原始里程计/速度反馈和 IMU 方向数据，输出为 odom 坐标系下的机器人位姿。
 * 使用航位推算（dead reckoning）：通过对速度积分计算位置变化。
 */

#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>

namespace robot_platform_pkg {

class IOdometryInterface {
public:
  virtual ~IOdometryInterface() = default;

  /**
   * @brief 根据底盘原始反馈和 IMU 航向更新里程计估计。
   * @param chassis_odom_raw 底盘原始里程计/速度反馈
   * @param imu              IMU 数据（提供航向角 yaw）
   * @param dt               时间间隔 (s)
   * @return 更新后的里程计消息
   */
  virtual nav_msgs::msg::Odometry update(
    const nav_msgs::msg::Odometry& chassis_odom_raw,
    const sensor_msgs::msg::Imu& imu,
    double dt) = 0;

  /// 将里程计重置到原点（x=0, y=0, yaw=0）
  virtual void reset() = 0;

  /**
   * @brief 获取当前位姿估计。
   * @param[out] x   X 坐标 (m)
   * @param[out] y   Y 坐标 (m)
   * @param[out] yaw 偏航角 (rad)
   */
  virtual void getPose(double& x, double& y, double& yaw) const = 0;
};

// ── 工厂函数 ──────────────────────────────────────────────────────

/// 创建全向轮里程计实例
std::unique_ptr<IOdometryInterface> make_omni_wheel_odometry();

}  // namespace robot_platform_pkg
