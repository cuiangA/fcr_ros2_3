/**
 * @file mpc_controller.hpp
 * @brief MPC（模型预测控制）视觉伺服 — 论文扩展占位。
 *
 * 核心思想：在有限预测时域内优化未来 N 步的控制序列，
 * 在满足速度/视野约束的前提下最小化特征误差。
 *
 * 优化问题（每步求解）：
 *   min  Σ ||s_k - s*||²_Q + ||u_k||²_R
 *   s.t. s_{k+1} = s_k + L · u_k · dt  （特征运动模型）
 *        u_min ≤ u_k ≤ u_max            （速度约束）
 *        s_k ∈ image_bounds             （视野约束：特征点不超出图像边界）
 *
 * 当前为架构占位——完整实现需集成 QP 求解器（qpOASES / OSQP）。
 */

#pragma once

#include "servo_control_pkg/servo_controller_base.hpp"
#include <deque>

namespace servo_control_pkg {

class MPCController : public ServoControllerBase {
public:
  explicit MPCController(const rclcpp::NodeOptions& options);

  std::optional<Eigen::Matrix<double, 6, 1>> computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt) override;

  std::string getControllerType() const override { return "MPC"; }

private:
  /// 在预测时域上求解 MPC 优化问题
  Eigen::Matrix<double, 6, 1> solveMPC(
    const Eigen::Matrix<double, 6, 1>& current_state,
    const Eigen::Matrix<double, 6, 1>& target_state);

  /// 系统动力学模型（简单积分器 + 视觉特征模型）
  Eigen::Matrix<double, 6, 6> predictState(
    const Eigen::Matrix<double, 6, 1>& state,
    const Eigen::Matrix<double, 6, 1>& control,
    double dt);

  int horizon_;                                    ///< 预测时域步数 N
  double control_cost_weight_;                     ///< 控制代价权重 R（惩罚大速度）
  std::deque<Eigen::Matrix<double, 6, 1>> state_history_; ///< 状态历史（用于调试和学习）
};

}  // namespace servo_control_pkg
