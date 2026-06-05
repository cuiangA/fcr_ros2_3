/**
 * @file ibvs_controller.cpp
 * @brief IBVS 控制器实现 — 控制律 v = -λ · L⁺ · (s - s*)。
 *
 * 每次迭代的步骤：
 *   1. 从当前目标提取图像特征 s
 *   2. 计算特征误差 e = s - s*
 *   3. 若 ||e|| < tolerance → 已收敛，返回零速度
 *   4. 计算当前深度处的交互矩阵 L
 *   5. 求解 L⁺（通过 completeOrthogonalDecomposition 伪逆）
 *   6. 计算控制律 v = -λ · L⁺ · e
 *   7. 对 v 进行限幅（线速度和角速度分别钳位）
 *
 * 自适应增益策略：
 *   误差大 → 大增益（快速收敛）
 *   误差小 → 小增益（精确调节，避免超调）
 *   在 error_threshold_slow 区间内线性插值
 */

#include "servo_control_pkg/ibvs_controller.hpp"
#include <Eigen/SVD>
#include <pluginlib/class_list_macros.hpp>

namespace servo_control_pkg {

IBVSController::IBVSController(const rclcpp::NodeOptions& options)
  : ServoControllerBase("ibvs_controller", options),
    gain_max_(1.0), gain_min_(0.1),
    error_threshold_slow_(0.05), use_adaptive_gain_(true)
{
  // ── 声明 IBVS 专属参数 ──────────────────────────────────────────
  this->declare_parameter("gain_max", 1.0);
  this->declare_parameter("gain_min", 0.1);
  this->declare_parameter("error_threshold_slow", 0.05);
  this->declare_parameter("use_adaptive_gain", true);

  gain_max_ = this->get_parameter("gain_max").as_double();
  gain_min_ = this->get_parameter("gain_min").as_double();
  error_threshold_slow_ = this->get_parameter("error_threshold_slow").as_double();
  use_adaptive_gain_ = this->get_parameter("use_adaptive_gain").as_bool();
}

void IBVSController::configureFromNode(const rclcpp::Node& node) {
  ServoControllerBase::configureFromNode(node);

  auto get_double = [&node](const std::string& name, double fallback) {
    double value = fallback;
    if (node.has_parameter(name)) {
      node.get_parameter(name, value);
    }
    return value;
  };
  auto get_bool = [&node](const std::string& name, bool fallback) {
    bool value = fallback;
    if (node.has_parameter(name)) {
      node.get_parameter(name, value);
    }
    return value;
  };

  gain_max_ = get_double("gain_max", gain_max_);
  gain_min_ = get_double("gain_min", gain_min_);
  error_threshold_slow_ = get_double("error_threshold_slow", error_threshold_slow_);
  use_adaptive_gain_ = get_bool("use_adaptive_gain", use_adaptive_gain_);
}

std::optional<Eigen::Matrix<double, 6, 1>> IBVSController::computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt) {
  (void)dt;
  // 未标定 → 无法计算
  if (!initialized_ || !goal_configured_) {
    last_camera_velocity_.setZero();
    return std::nullopt;
  }

  // ── 步骤 1：提取当前特征 ────────────────────────────────────────
  current_features_ = extractFeatures(target);
  current_depth_ = target.position[2];  // Z 坐标 = 深度

  // ── 步骤 2：计算特征误差 ────────────────────────────────────────
  feature_error_ = current_features_ - goal_.desired_features;

  // ── 步骤 3：检查收敛 ────────────────────────────────────────────
  double error_norm = feature_error_.norm();
  if (error_norm < goal_.feature_tolerance) {
    last_camera_velocity_.setZero();
    return Eigen::Matrix<double, 6, 1>::Zero();  // 已收敛，输出零速度
  }

  // ── 步骤 4：计算交互矩阵 ────────────────────────────────────────
  interaction_matrix_ = computeInteractionMatrix(current_features_, current_depth_);

  // ── 步骤 5-6：计算控制律 v = -λ · L⁺ · e ──────────────────────
  double lambda = use_adaptive_gain_ ? computeAdaptiveGain(error_norm) : lambda_gain_;

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(
    interaction_matrix_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 6, 1> singular_inv;
  const auto singular_values = svd.singularValues();
  for (int i = 0; i < singular_values.size(); ++i) {
    singular_inv(i) = singular_values(i) > 1e-6 ? 1.0 / singular_values(i) : 0.0;
  }

  Eigen::Matrix<double, 6, 1> velocity =
    -lambda * svd.matrixV() * singular_inv.asDiagonal() *
    svd.matrixU().transpose() * feature_error_;

  // ── 步骤 7：限幅 ────────────────────────────────────────────────
  velocity = clampVelocity(velocity);

  iteration_count_++;
  last_camera_velocity_ = velocity;
  return velocity;
}

Eigen::Matrix<double, 6, 1> IBVSController::clampVelocity(
    const Eigen::Matrix<double, 6, 1>& v) {
  Eigen::Matrix<double, 6, 1> clamped = v;

  // 线速度限幅：保持方向，缩放幅值
  double linear_norm = v.head<3>().norm();
  if (linear_norm > max_linear_vel_) {
    clamped.head<3>() *= max_linear_vel_ / linear_norm;  // 按比例缩放到限幅值
  }

  // 角速度限幅：同理
  double angular_norm = v.tail<3>().norm();
  if (angular_norm > max_angular_vel_) {
    clamped.tail<3>() *= max_angular_vel_ / angular_norm;
  }

  return clamped;
}

double IBVSController::computeAdaptiveGain(double error_norm) {
  if (!use_adaptive_gain_) return lambda_gain_;

  // 线性插值：λ(e) = λ_min + (λ_max - λ_min) · min(e / e_thresh, 1.0)
  // 误差为 0 时增益 = λ_min，误差 ≥ e_thresh 时增益 = λ_max
  return gain_min_ + (gain_max_ - gain_min_) *
         std::min(error_norm / error_threshold_slow_, 1.0);
}

}  // namespace servo_control_pkg

// 注册为 pluginlib 插件，使其可通过 ClassLoader 动态加载
PLUGINLIB_EXPORT_CLASS(servo_control_pkg::IBVSController,
                       servo_control_pkg::ServoControllerBase)
