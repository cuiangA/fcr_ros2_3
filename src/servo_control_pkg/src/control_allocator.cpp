/**
 * @file control_allocator.cpp
 * @brief 控制分配器实现 — 将相机 6-DOF 速度分解为底盘 + 云台指令。
 *
 * PRIORITY 分配策略：
 *   1. 平移分量（vx, vy, vz）全部由底盘执行（云台仅负责旋转）
 *   2. 旋转分量（ωx, ωy, ωz）按 allocation_ratio 分配：
 *      - (1 - ratio) 部分由云台执行（快速响应、高精度）
 *      - ratio 部分由底盘执行（低频大角度转动）
 *   3. 云台指令经过一阶低通滤波器平滑（α = 0.3）
 *   4. 当云台角速度接近零时（< 0.001 rad/s），设置 hold 标志锁定当前位置
 *
 * 速度向量约定：
 *   camera_velocity = [vx, vy, vz, wx, wy, wz]，位于 ROS optical frame：
 *   x 右、y 下、z 前。底盘命令位于 base_link：x 前、y 左、z 上。
 *
 *   base_x   =  camera_z
 *   base_y   = -camera_x
 *   base_yaw = -camera_wy
 *   gimbal_pitch ≈ -camera_wx
 *   camera_wz 为光轴 roll，当前 MVP 暂不分配给执行器。
 */

#include "servo_control_pkg/control_allocator.hpp"
#include <cmath>

namespace servo_control_pkg {

ControlAllocator::ControlAllocator()
  : strategy_(AllocationStrategy::PRIORITY),
    gimbal_yaw_limit_(M_PI), gimbal_pitch_limit_(M_PI_2),
    chassis_linear_limit_(1.0), chassis_angular_limit_(2.0),
    allocation_ratio_(0.5), prev_gimbal_yaw_(0), prev_gimbal_pitch_(0),
    smoothing_alpha_(0.3)
{}

void ControlAllocator::configure(
    double gimbal_yaw_limit, double gimbal_pitch_limit,
    double chassis_linear_limit, double chassis_angular_limit,
    double allocation_ratio) {
  gimbal_yaw_limit_ = gimbal_yaw_limit;
  gimbal_pitch_limit_ = gimbal_pitch_limit;
  chassis_linear_limit_ = chassis_linear_limit;
  chassis_angular_limit_ = chassis_angular_limit;
  // 钳位 allocation_ratio 到 [0, 1]
  allocation_ratio_ = std::clamp(allocation_ratio, 0.0, 1.0);
}

ControlAllocation ControlAllocator::allocate(
    const Eigen::Matrix<double, 6, 1>& camera_velocity,
    const vision_servo_msgs::msg::PlatformState& platform_state,
    double dt) {
  (void)platform_state;
  (void)dt;

  ControlAllocation cmd;

  // ── 坐标映射：camera_optical_link -> base_link ──────────────────
  // optical frame: x right, y down, z forward
  // base_link:     x forward, y left, z up
  const double base_forward = camera_velocity(2);
  const double base_lateral = -camera_velocity(0);
  const double yaw_rate = -camera_velocity(4);
  const double pitch_rate = -camera_velocity(3);

  // 云台取 (1 - ratio) 的旋转分量（快速响应、高精度）
  cmd.gimbal_yaw_rate = std::clamp(yaw_rate * (1.0 - allocation_ratio_),
                                   -gimbal_yaw_limit_, gimbal_yaw_limit_);
  cmd.gimbal_pitch_rate = std::clamp(pitch_rate * (1.0 - allocation_ratio_),
                                     -gimbal_pitch_limit_, gimbal_pitch_limit_);

  // 底盘取 ratio 的偏航旋转分量（云台角度限位不足时由底盘转动补足）
  cmd.chassis_twist.angular.z = std::clamp(yaw_rate * allocation_ratio_,
                                           -chassis_angular_limit_, chassis_angular_limit_);

  // ── 平移分配：全部由底盘执行 ────────────────────────────────────
  // camera_velocity(1) 是相机垂向平移，平面底盘无法执行，当前 MVP 忽略。
  // camera_velocity(5) 是光轴 roll，当前 MVP 也不分配。
  cmd.chassis_twist.linear.x = std::clamp(base_forward,
                                          -chassis_linear_limit_, chassis_linear_limit_);
  cmd.chassis_twist.linear.y = std::clamp(base_lateral,
                                          -chassis_linear_limit_, chassis_linear_limit_);

  // ── 云台指令平滑（一阶低通滤波器） ──────────────────────────────
  // 防止相邻帧指令跳变导致云台抖动
  cmd.gimbal_yaw_rate = smoothing_alpha_ * cmd.gimbal_yaw_rate +
                        (1.0 - smoothing_alpha_) * prev_gimbal_yaw_;
  cmd.gimbal_pitch_rate = smoothing_alpha_ * cmd.gimbal_pitch_rate +
                          (1.0 - smoothing_alpha_) * prev_gimbal_pitch_;

  prev_gimbal_yaw_ = cmd.gimbal_yaw_rate;
  prev_gimbal_pitch_ = cmd.gimbal_pitch_rate;

  // 当指令接近零时锁定云台位置（防止漂移）
  cmd.hold_yaw = (std::abs(cmd.gimbal_yaw_rate) < 0.001);
  cmd.hold_pitch = (std::abs(cmd.gimbal_pitch_rate) < 0.001);

  return cmd;
}

}  // namespace servo_control_pkg
