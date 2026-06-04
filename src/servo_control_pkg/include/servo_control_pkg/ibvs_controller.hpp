/**
 * @file ibvs_controller.hpp
 * @brief IBVS（基于图像的视觉伺服）控制器。
 *
 * 核心思想：直接在图像特征空间中定义误差和控制律。
 *
 * 控制律：v_camera = -λ · L⁺ · (s - s*)
 *   其中 s  = 当前图像特征向量（6 维归一化坐标）
 *        s* = 期望特征向量（示教阶段记录）
 *        L⁺  = 交互矩阵（图像雅可比）的伪逆
 *        λ  = 自适应控制增益
 *
 * 自适应增益：
 *   误差大时使用 gain_max，加速收敛
 *   误差小时使用 gain_min，避免超调和振荡
 *   在 error_threshold_slow 区间内线性插值
 *
 * 优势：对相机标定误差鲁棒，计算高效（无需 3D 重建）
 * 劣势：收敛轨迹可能在图像空间中直线、笛卡尔空间中弯曲
 */

#pragma once

#include "servo_control_pkg/servo_controller_base.hpp"
#include <Eigen/Dense>

namespace servo_control_pkg {

class IBVSController : public ServoControllerBase {
public:
  IBVSController() : ServoControllerBase("ibvs_controller", rclcpp::NodeOptions()) {}
  explicit IBVSController(const rclcpp::NodeOptions& options);

  std::optional<Eigen::Matrix<double, 6, 1>> computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt) override;

  std::string getControllerType() const override { return "IBVS"; }

private:
  /// 对速度向量进行限幅（线速度和角速度分别钳位）
  Eigen::Matrix<double, 6, 1> clampVelocity(const Eigen::Matrix<double, 6, 1>& v);

  /**
   * @brief 根据误差范数计算自适应增益。
   *
   * 公式：λ(e) = λ_min + (λ_max - λ_min) · min(||e|| / e_thresh, 1.0)
   * 误差大于阈值时使用最大增益，误差趋零时使用最小增益。
   */
  double computeAdaptiveGain(double error_norm);

  // ── IBVS 专属参数 ───────────────────────────────────────────────
  double gain_max_;              ///< 自适应增益最大值
  double gain_min_;              ///< 自适应增益最小值
  double error_threshold_slow_;  ///< 误差阈值（低于此值降低增益，避免超调）
  bool use_adaptive_gain_;       ///< 是否启用自适应增益
};

}  // namespace servo_control_pkg
