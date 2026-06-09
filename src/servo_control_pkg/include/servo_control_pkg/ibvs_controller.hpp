/**
 * @file ibvs_controller.hpp
 * @brief IBVS（基于图像的视觉伺服）控制器。
 *
 * 核心思想：直接在图像特征空间中定义误差和控制律。
 *
 * 控制律：v_camera = -λ · L⁺ · (s - s*)
 *   其中 s  = 当前图像特征向量（bbox 3 个点：左上、右下、右上，均为归一化坐标）
 *        s* = 期望图像特征（默认位于图像中心并满足期望尺度）
 *        L⁺  = 交互矩阵（图像雅可比）的阻尼伪逆
 *        λ  = 自适应控制增益
 *
 * 特征向量定义（6 维）：
 *   [x_lt, y_lt, x_rb, y_rb, x_rt, y_rt]
 *   归一化坐标 x_n = (u - cx) / fx,  y_n = (v - cy) / fy
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
  /// 默认构造（委托到带参构造），供 pluginlib 发现
  IBVSController() : IBVSController(rclcpp::NodeOptions()) {}

  /**
   * @brief 带参数构造，由 pluginlib::ClassLoader 调用。
   * @param options ROS2 节点选项（包含参数覆盖）
   */
  explicit IBVSController(const rclcpp::NodeOptions& options);

  /**
   * @brief 主控制迭代：根据当前目标观测计算相机速度。
   *
   * 控制流程：提取特征 → 计算误差 → 检查收敛 → 构建交互矩阵 →
   *            阻尼伪逆 → v = -λ·L⁺·e → 限幅
   *
   * @param target 当前跟踪目标（含 bbox 用于特征提取、position[2] 用于深度）
   * @param dt     距上次更新的时间间隔 (s)，IBVS 当前未使用（无状态依赖）
   * @return 相机系 6-DOF 速度 [vx,vy,vz, ωx,ωy,ωz]^T，未初始化或收敛则返回 nullopt
   */
  std::optional<Eigen::Matrix<double, 6, 1>> computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt) override;

  /**
   * @brief 从宿主节点拉取 IBVS 专属参数。
   * @param node servo_manager 节点的引用
   */
  void configureFromNode(const rclcpp::Node& node) override;

  /// @return 控制器类型标识字符串 "IBVS"
  std::string getControllerType() const override { return "IBVS"; }

private:
  /**
   * @brief 对 6-DOF 速度向量进行限幅（线速度和角速度分别钳位）。
   *
   * 线速度分量 (vx,vy,vz) 的模长不超过 max_linear_vel_，
   * 角速度分量 (ωx,ωy,ωz) 的模长不超过 max_angular_vel_。
   * 保持方向不变，仅缩放幅值。
   *
   * @param v 未经限幅的 6-DOF 速度向量
   * @return 限幅后的速度向量
   */
  Eigen::Matrix<double, 6, 1> clampVelocity(const Eigen::Matrix<double, 6, 1>& v);

  /**
   * @brief 根据误差范数计算自适应增益 λ。
   *
   * 公式：λ(e) = λ_min + (λ_max - λ_min) · min(||e|| / e_thresh, 1.0)
   *
   *       λ
   *  λ_max ┤─────────────
   *        │         ／
   *        │       ／
   *  λ_min ┤─────┘
   *        └──────┬──────→ ||e||
   *            e_thresh
   *
   *   误差 ≥ e_thresh → 满增益 λ_max（快速收敛）
   *   误差 → 0       → 最小增益 λ_min（精细调节，避免超调）
   *
   * @param error_norm 当前特征误差的 L2 范数
   * @return 自适应增益值 λ（无量纲），范围为 [λ_min, λ_max]
   */
  double computeAdaptiveGain(double error_norm);

  // ── 自适应增益参数 ───────────────────────────────────────────────
  double gain_max_;              ///< 最大增益 λ_max（无量纲），大误差时使用，加速收敛
  double gain_min_;              ///< 最小增益 λ_min（无量纲），小误差时使用，避免超调
  double error_threshold_slow_;  ///< 减速阈值（特征误差范数），低于此值线性降低增益
  double svd_damping_;           ///< SVD 阻尼系数 μ，L⁺ = V·diag(σ/(σ²+μ²))·U^T，防奇异
  bool use_adaptive_gain_;       ///< 是否启用自适应增益（关闭时固定使用基类 lambda_gain_）
};

}  // namespace servo_control_pkg
