/**
 * @file pbvs_controller.hpp
 * @brief PBVS（基于位置的视觉伺服）控制器。
 *
 * 核心思想：先从图像特征重建 3D 位姿，再在笛卡尔空间中定义误差和控制律。
 *
 * 控制律（解耦）：
 *   v_translation = -K_t · e_translation
 *   v_rotation    = -K_r · e_rotation
 *   其中 e = current_pose ⊖ desired_pose
 *
 * 3D 重建通过针孔模型反投影实现（需要相机内参和深度值）。
 *
 * 优势：收敛轨迹在笛卡尔空间中为直线，预测性好
 * 劣势：需要精确的相机标定和 3D 模型，对标定误差敏感
 */

#pragma once

#include "servo_control_pkg/servo_controller_base.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace servo_control_pkg {

class PBVSController : public ServoControllerBase {
public:
  PBVSController() : ServoControllerBase("pbvs_controller", rclcpp::NodeOptions()) {}
  explicit PBVSController(const rclcpp::NodeOptions& options);

  std::optional<Eigen::Matrix<double, 6, 1>> computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt) override;

  std::string getControllerType() const override { return "PBVS"; }

  void configureFromNode(const rclcpp::Node& node) override;

  bool setGoalFromTarget(
    const vision_servo_msgs::msg::Target& target,
    double desired_depth,
    double feature_tolerance) override;

  bool initialize(double fx, double fy, double cx, double cy,
                  int width, int height) override;

private:
  /**
   * @brief 从图像特征重建 3D 位姿。
   *
   * 简化方法：使用边界框中心 + 深度值反投影到相机坐标系。
   * 实际生产中可使用 PnP（Perspective-n-Point）算法获得更精确的位姿。
   */
  Eigen::Isometry3d reconstructPose(const Eigen::Matrix<double, 6, 1>& features, double depth);

  /// 优先使用 Target.position，缺失时回退到 bbox + depth 反投影。
  Eigen::Isometry3d targetToPose(const vision_servo_msgs::msg::Target& target);

  /**
   * @brief 计算当前位姿与期望位姿之间的笛卡尔误差。
   * @return 6 维误差向量 [ex, ey, ez, eωx, eωy, eωz]^T
   */
  Eigen::Matrix<double, 6, 1> computePoseError(const Eigen::Isometry3d& current_pose);

  // ── PBVS 参数 ──────────────────────────────────────────────────
  double translational_gain_;    ///< 平动控制增益 K_t
  double rotational_gain_;       ///< 转动控制增益 K_r（通常较小以保证稳定）
  Eigen::Isometry3d desired_pose_;  ///< 期望位姿（示教阶段记录）
  Eigen::Isometry3d current_pose_;  ///< 当前位姿
  bool desired_pose_set_;        ///< 是否已设置期望位姿
};

}  // namespace servo_control_pkg
