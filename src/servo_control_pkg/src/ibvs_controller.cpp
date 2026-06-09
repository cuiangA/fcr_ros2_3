/**
 * @file ibvs_controller.cpp
 * @brief IBVS 控制器实现 — 控制律 v = -λ · L⁺ · (s - s*)。
 *
 * 每次迭代的步骤：
 *   1. 从当前目标提取图像特征 s
 *   2. 计算特征误差 e = s - s*
 *   3. 若 ||e|| < tolerance → 已收敛，返回零速度
 *   4. 计算当前深度处的交互矩阵 L
 *   5. 通过 SVD 阻尼伪逆求解 L⁺
 *      L⁺ = V · diag(σᵢ/(σᵢ² + μ²)) · U^T
 *   6. 计算控制律 v = -λ · L⁺ · e
 *   7. 对 v 进行限幅（线速度和角速度分别钳位）
 *
 * 特征向量（6 维）：
 *   s = [x_lt, y_lt, x_rb, y_rb, x_rt, y_rt]
 *   归一化图像坐标：x_n = (u - cx)/fx, y_n = (v - cy)/fy
 *   3 个点 → 完整 6×6 交互矩阵（每个点贡献 2×6 子矩阵）
 *
 * 交互矩阵（单点，深度 Z）：
 *   L(x, y, Z) = [ -1/Z,  0,     x/Z,  xy,   -(1+x²),  y
 *                   0,    -1/Z,  y/Z,  1+y²,  -xy,     -x ]
 *
 * 自适应增益策略：
 *   误差大 → 大增益（快速收敛）
 *   误差小 → 小增益（精确调节，避免超调）
 *   在 error_threshold_slow 区间内线性插值
 */

#include "servo_control_pkg/ibvs_controller.hpp"
#include <Eigen/SVD>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>

namespace servo_control_pkg {

// ══════════════════════════════════════════════════════════════════════════════
// 构造 / 配置
// ══════════════════════════════════════════════════════════════════════════════

IBVSController::IBVSController(const rclcpp::NodeOptions& options)
  : ServoControllerBase("ibvs_controller", options),
    gain_max_(1.0),              // λ_max 默认值（会被 YAML 覆盖）
    gain_min_(0.1),              // λ_min 默认值
    error_threshold_slow_(0.05), // 减速阈值默认值（特征误差 L2 范数）
    svd_damping_(0.01),          // SVD 阻尼系数 μ 默认值
    use_adaptive_gain_(true)     // 默认启用自适应增益
{
  // ── 声明 IBVS 专属参数到 ROS 参数服务器 ──────────────────────────
  this->declare_parameter("gain_max", 1.0);               // λ_max
  this->declare_parameter("gain_min", 0.1);               // λ_min
  this->declare_parameter("error_threshold_slow", 0.05);  // 减速阈值
  this->declare_parameter("svd_damping", 0.01);            // 阻尼系数 μ
  this->declare_parameter("use_adaptive_gain", true);      // 自适应开关

  // 从参数服务器读取当前值（YAML 可能已覆盖）
  gain_max_ = this->get_parameter("gain_max").as_double();
  gain_min_ = this->get_parameter("gain_min").as_double();
  error_threshold_slow_ = this->get_parameter("error_threshold_slow").as_double();
  svd_damping_ = this->get_parameter("svd_damping").as_double();
  use_adaptive_gain_ = this->get_parameter("use_adaptive_gain").as_bool();
}

void IBVSController::configureFromNode(const rclcpp::Node& node) {
  ServoControllerBase::configureFromNode(node);

  // 辅助 lambda：从 node 读取 double 参数，不存在则保留当前值
  auto get_double = [&node](const std::string& name, double fallback) {
    double value = fallback;  // 先设为回退值
    if (node.has_parameter(name)) {
      node.get_parameter(name, value);  // 存在则覆盖
    }
    return value;
  };

  // 辅助 lambda：同上，bool 版本
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
  svd_damping_ = get_double("svd_damping", svd_damping_);
  use_adaptive_gain_ = get_bool("use_adaptive_gain", use_adaptive_gain_);
}

// ══════════════════════════════════════════════════════════════════════════════
// 核心控制律
// ══════════════════════════════════════════════════════════════════════════════

std::optional<Eigen::Matrix<double, 6, 1>> IBVSController::computeVelocity(
    const vision_servo_msgs::msg::Target& target,  // 当前帧目标观测
    double dt) {                                    // 时间步长 (s)，当前未使用
  (void)dt;  // 显式标记未使用，IBVS 控制律不依赖时间步长

  // 前置检查：必须已完成标定、已设置伺服目标
  if (!initialized_ || !goal_configured_) {
    last_camera_velocity_.setZero();  // 未就绪 → 输出零速度
    return std::nullopt;
  }

  // ── 步骤 1：提取当前图像特征 ────────────────────────────────────
  // extractFeatures() 从 bbox 提取 3 个真实图像点（左上、右下、右上），
  // 使用针孔模型归一化坐标，因此后续可直接套用点特征 IBVS 交互矩阵。
  current_features_ = extractFeatures(target);

  // 安全检查：特征包含 NaN 或 Inf → 跳过本帧
  if (!current_features_.allFinite()) {
    last_camera_velocity_.setZero();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "IBVS 特征包含非有限值，输出零速度");
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  // 获取目标深度（相机系 z 坐标）
  current_depth_ = target.position[2];  // priority: 3D position 的 z
  if (!std::isfinite(current_depth_) || current_depth_ <= 0.0) {
    // 深度无效或非正 → 回退到期望深度或 1.0m
    current_depth_ = goal_.desired_depth > 0.0 ? goal_.desired_depth : 1.0;
  }

  // ── 步骤 2：计算特征误差 e = s - s* ────────────────────────────
  feature_error_ = current_features_ - goal_.desired_features;

  // ── 步骤 3：检查收敛 ────────────────────────────────────────────
  double error_norm = feature_error_.norm();  // L2 范数，收敛判断指标
  if (error_norm < goal_.feature_tolerance) {
    last_camera_velocity_.setZero();
    return Eigen::Matrix<double, 6, 1>::Zero();  // 已收敛，输出零速度
  }

  // ── 步骤 4：计算交互矩阵 L（6×6 图像雅可比） ───────────────────
  // 对 3 个特征点各构建 2×6 子矩阵，堆叠为 6×6 方阵。
  // 深度 Z 取 current_depth_（假设 3 个 bbox 点近似共面）。
  interaction_matrix_ = computeInteractionMatrix(current_features_, current_depth_);

  // ── 步骤 5：计算自适应增益 λ ────────────────────────────────────
  // use_adaptive_gain_=true → λ 在 [λ_min, λ_max] 内随误差缩放
  // use_adaptive_gain_=false → λ = lambda_gain_（基类固定增益）
  double lambda = use_adaptive_gain_ ? computeAdaptiveGain(error_norm) : lambda_gain_;

  // ── 步骤 6：SVD 阻尼伪逆 + 控制律 v = -λ·L⁺·e ────────────────
  //
  // 使用 JacobiSVD 分解 L = U · Σ · V^T，则：
  //   L⁺ = V · Σ⁺ · U^T
  //   阻尼伪逆：σ⁺ = σ / (σ² + μ²)
  //
  // 相比硬阈值截断（1/σ, σ>ε），阻尼伪逆在特征点退化或深度噪声大时
  // 更平滑，避免指令突变。
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(
    interaction_matrix_,                            // 6×6 交互矩阵 L
    Eigen::ComputeFullU | Eigen::ComputeFullV);     // 需要完整的 U 和 V

  // 构建阻尼伪逆的奇异值向量 Σ⁺
  Eigen::Matrix<double, 6, 1> singular_inv;  // 6 个奇异值的阻尼伪逆 σᵢ/(σᵢ²+μ²)
  const auto singular_values = svd.singularValues();  // Σ = diag(σ₁...σ₆)
  for (int i = 0; i < singular_values.size(); ++i) {
    const double sigma = singular_values(i);           // 第 i 个奇异值
    const double damping = std::max(0.0, svd_damping_); // μ ≥ 0
    // 阻尼伪逆：σ/(σ²+μ²)，σ→0 时趋近 0/μ²=0（而非发散到无穷）
    singular_inv(i) =
      sigma > 1e-9 ? sigma / (sigma * sigma + damping * damping) : 0.0;
  }

  // 计算控制律：v = -λ · (V · Σ⁺ · U^T) · e
  //   分解为右结合：先 U^T·e，再 Σ⁺·(U^T·e)，最后 V·(Σ⁺·U^T·e)
  // velocity: 相机系 6-DOF 速度 [vx,vy,vz, ωx,ωy,ωz]
  Eigen::Matrix<double, 6, 1> velocity =
    -lambda * svd.matrixV() * singular_inv.asDiagonal() *
    svd.matrixU().transpose() * feature_error_;

  // ── 步骤 7：限幅（线速度和角速度分别钳位到 max_linear/angular_vel） ─
  velocity = clampVelocity(velocity);

  // 安全检查：非有限值保护
  if (!velocity.allFinite()) {
    velocity.setZero();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "IBVS 计算得到非有限速度，已强制清零");
  }

  iteration_count_++;                       // 累计控制迭代次数
  last_camera_velocity_ = velocity;         // 缓存本次输出（用于状态上报和调试）
  return velocity;
}

// ══════════════════════════════════════════════════════════════════════════════
// 速度限幅
// ══════════════════════════════════════════════════════════════════════════════

Eigen::Matrix<double, 6, 1> IBVSController::clampVelocity(
    const Eigen::Matrix<double, 6, 1>& v) {  // 未经限幅的 6-DOF 速度

  Eigen::Matrix<double, 6, 1> clamped = v;  // 限幅后的输出，默认拷贝

  // 线速度限幅：保持方向，缩放幅值
  double linear_norm = v.head<3>().norm();   // ||[vx, vy, vz]||
  if (linear_norm > max_linear_vel_) {
    // 按比例缩放到 max_linear_vel_，方向不变
    clamped.head<3>() *= max_linear_vel_ / linear_norm;
  }

  // 角速度限幅：同理
  double angular_norm = v.tail<3>().norm();  // ||[ωx, ωy, ωz]||
  if (angular_norm > max_angular_vel_) {
    clamped.tail<3>() *= max_angular_vel_ / angular_norm;
  }

  return clamped;
}

// ══════════════════════════════════════════════════════════════════════════════
// 自适应增益
// ══════════════════════════════════════════════════════════════════════════════

double IBVSController::computeAdaptiveGain(
    double error_norm) {  // 当前特征误差的 L2 范数 ||s - s*||

  // 固定增益模式：直接返回基类的 lambda_gain_
  if (!use_adaptive_gain_) return lambda_gain_;

  // 自适应增益公式：
  //   λ(e) = λ_min + (λ_max - λ_min) · min(||e|| / e_thresh, 1.0)
  //
  // 解释：
  //   ||e|| = 0       → λ = λ_min  （精确调节，无超调）
  //   ||e|| ≥ e_thresh → λ = λ_max  （快速收敛）
  //   ||e|| ∈ (0, e_thresh) → 线性插值
  //
  // min(error_norm / error_threshold_slow_, 1.0): 归一化误差比例 [0, 1]
  return gain_min_ + (gain_max_ - gain_min_) *
         std::min(error_norm / error_threshold_slow_, 1.0);
}

}  // namespace servo_control_pkg

// 注册为 pluginlib 插件，使其可通过 ClassLoader 动态加载和热切换
PLUGINLIB_EXPORT_CLASS(servo_control_pkg::IBVSController,
                       servo_control_pkg::ServoControllerBase)
