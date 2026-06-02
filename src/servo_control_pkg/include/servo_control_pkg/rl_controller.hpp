/**
 * @file rl_controller.hpp
 * @brief RL（强化学习）视觉伺服控制器 — 论文扩展占位。
 *
 * 核心思想：将视觉伺服建模为马尔可夫决策过程（MDP），
 * 使用深度强化学习策略 π(a|s) 直接输出相机速度指令。
 *
 * 观测空间（12 维）：
 *   [特征误差(6), 当前深度(1), 上一帧速度(3), 收敛标志(1), 时间步(1)]
 *
 * 动作空间（6 维）：
 *   [vx, vy, vz, ωx, ωy, ωz]（连续控制，相机会速度快指令）
 *
 * 奖励信号：
 *   r = -||特征误差||² - w_u·||u||² + r_convergence（收敛奖励）
 *
 * 支持的推理后端：
 *   - ONNX Runtime（通用，跨平台）
 *   - TorchScript（PyTorch 导出）
 *   - Stable-Baselines3（训练框架）
 */

#pragma once

#include "servo_control_pkg/servo_controller_base.hpp"

namespace servo_control_pkg {

class RLController : public ServoControllerBase {
public:
  explicit RLController(const rclcpp::NodeOptions& options);

  std::optional<Eigen::Matrix<double, 6, 1>> computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt) override;

  std::string getControllerType() const override { return "RL"; }

  bool initialize(double fx, double fy, double cx, double cy,
                  int width, int height) override;

private:
  /// 加载训练好的 RL 策略模型（ONNX 或 TorchScript 格式）
  void loadPolicy(const std::string& model_path);

  /// 执行策略推理：observation → action
  Eigen::Matrix<double, 6, 1> inferPolicy(const Eigen::Matrix<double, 12, 1>& observation);

  /// 从目标状态和伺服状态构建 12 维观测向量
  Eigen::Matrix<double, 12, 1> buildObservation(
    const vision_servo_msgs::msg::Target& target);

  std::string policy_path_;   ///< 策略模型文件路径
  std::string policy_type_;   ///< 模型格式："onnx" / "torchscript" / "sb3"
};

}  // namespace servo_control_pkg
