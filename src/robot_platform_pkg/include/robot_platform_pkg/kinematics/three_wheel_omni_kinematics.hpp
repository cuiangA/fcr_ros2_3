/**
 * @file three_wheel_omni_kinematics.hpp
 * @brief LEKIWI 三全向轮运动学模型（120° 对称布局）。
 *
 * 三个全向轮以 0° / 120° / 240° 均匀分布在底盘圆周上。
 * 每个轮子可独立驱动，通过速度合成产生任意方向的平动和转动。
 *
 * 核心公式：
 *   逆运动学：给定底盘期望速度 (vx, vy, ω)，计算各轮转速
 *     w_i = (1/r) * [ -sin(α_i), cos(α_i), L ] · [vx, vy, ω]^T
 *   正运动学：给定各轮转速，推算底盘实际速度
 *     通过矩阵伪逆（completeOrthogonalDecomposition）求解
 *
 * 其中 r = 轮半径, L = 底盘半径, α_i = 轮安装角度
 */

#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace robot_platform_pkg {

class ThreeWheelOmniKinematics {
public:
  /**
   * @brief 构造函数 — 预计算正/逆运动学矩阵。
   * @param wheel_radius 全向轮半径 (m)，默认 0.05
   * @param base_radius  底盘半径（轮心到几何中心距离，m），默认 0.15
   */
  ThreeWheelOmniKinematics(double wheel_radius = 0.05,
                           double base_radius = 0.15)
    : wheel_radius_(wheel_radius), base_radius_(base_radius)
  {
    // 三个轮子的安装角度（机体坐标系下，弧度制）
    wheel_angles_ << 0.0, 2.0 * M_PI / 3.0, 4.0 * M_PI / 3.0;

    // 正运动学矩阵 M_forward = inv(M_inverse)
    computeMatrices();
  }

  /**
   * @brief 逆运动学：底盘速度 → 各轮转速。
   * @param twist [vx, vy, ω]^T（m/s, m/s, rad/s）
   * @return [w1, w2, w3]^T（rad/s）
   */
  Eigen::Vector3d inverse(const Eigen::Vector3d& twist) const {
    return M_inverse_ * twist;
  }

  /**
   * @brief 正运动学：各轮转速 → 底盘速度。
   * @param wheel_velocities [w1, w2, w3]^T（rad/s）
   * @return [vx, vy, ω]^T（m/s, m/s, rad/s）
   */
  Eigen::Vector3d forward(const Eigen::Vector3d& wheel_velocities) const {
    return M_forward_ * wheel_velocities;
  }

  double getWheelRadius() const { return wheel_radius_; }
  double getBaseRadius() const { return base_radius_; }

private:
  /// 预计算正/逆运动学矩阵（仅在构造函数中调用一次）
  void computeMatrices() {
    M_inverse_ = Eigen::Matrix3d::Zero();

    // 逆运动学矩阵（3x3）：
    // 对每个安装在 α_i 角度的轮子：
    //   w_i = (1/r) * [ -sin(α_i), cos(α_i), L ] · [vx, vy, ω]^T
    for (int i = 0; i < 3; ++i) {
      double alpha = wheel_angles_(i);
      M_inverse_(i, 0) = -std::sin(alpha);
      M_inverse_(i, 1) =  std::cos(alpha);
      M_inverse_(i, 2) =  base_radius_;
    }
    M_inverse_ /= wheel_radius_;

    // 正运动学 = 逆运动学的伪逆（completeOrthogonalDecomposition 比 SVD 更快）
    M_forward_ = M_inverse_.completeOrthogonalDecomposition().pseudoInverse();
  }

  double wheel_radius_;          ///< 全向轮半径 (m)
  double base_radius_;            ///< 底盘半径 (m)
  Eigen::Vector3d wheel_angles_;  ///< 各轮安装角度 (rad)
  Eigen::Matrix3d M_inverse_;     ///< 逆运动学矩阵（底盘速度 → 轮速）
  Eigen::Matrix3d M_forward_;     ///< 正运动学矩阵（轮速 → 底盘速度）
};

}  // namespace robot_platform_pkg
