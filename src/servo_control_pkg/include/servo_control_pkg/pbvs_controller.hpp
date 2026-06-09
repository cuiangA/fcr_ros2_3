/**
 * @file pbvs_controller.hpp
 * @brief PBVS（基于位置的视觉伺服）控制器。
 *
 * 核心思想：先从图像特征重建 3D 位姿，再在笛卡尔空间中定义误差和控制律。
 *
 * 控制律（2026-06 修正）：
 *   平动: v_trans = +K_t · (P_current - P_desired)
 *         运动学 dP/dt = -v_c 决定了需要用正增益才能收敛
 *   转动: ω = +K_r · angle · rotation_axis
 *         从光轴对准目标方向的几何叉积直接计算，不通过姿态矩阵分解
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
  /// 默认构造函数：使用空 NodeOptions，pluginlib 通过带参构造加载
  PBVSController() : ServoControllerBase("pbvs_controller", rclcpp::NodeOptions()) {}

  /**
   * @brief 带参数构造，由 pluginlib::ClassLoader 调用。
   * @param options ROS2 节点选项（包含参数覆盖）
   */
  explicit PBVSController(const rclcpp::NodeOptions& options);

  /**
   * @brief 主控制迭代：根据当前目标观测计算相机速度。
   * @param target 当前跟踪目标（包含 3D 位置和边界框）
   * @param dt     距上次更新的时间间隔 (s)，PBVS 当前未使用（P 控制器无状态依赖）
   * @return 相机系 6-DOF 速度 [vx,vy,vz, ωx,ωy,ωz]^T，未初始化则返回 nullopt
   */
  std::optional<Eigen::Matrix<double, 6, 1>> computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt) override;

  /// @return 控制器类型标识字符串 "PBVS"（用于日志和模式切换）
  std::string getControllerType() const override { return "PBVS"; }

  /**
   * @brief 从宿主节点拉取参数。
   * @param node servo_manager 节点的引用，PBVS 专属参数从其读取
   * @note 由 servo_manager 在加载/切换插件后显式调用。
   */
  void configureFromNode(const rclcpp::Node& node) override;

  /**
   * @brief 用当前目标观测生成一个伺服目标。
   * @param target           当前目标观测（3D 位置 + bbox）
   * @param desired_depth    期望深度 (m)，目标将始终被保持在相机正前方此距离
   * @param feature_tolerance 收敛阈值，误差范数低于此值视为已跟踪到位
   * @return 是否成功设置目标
   */
  bool setGoalFromTarget(
    const vision_servo_msgs::msg::Target& target,
    double desired_depth,
    double feature_tolerance) override;

  /**
   * @brief 使用相机内参初始化控制器。
   * @param fx, fy 焦距（像素），来自 CameraInfo::k[0], k[4]
   * @param cx, cy 主点（像素），来自 CameraInfo::k[2], k[5]
   * @param width, height 图像分辨率（像素）
   * @return 初始化是否成功
   */
  bool initialize(double fx, double fy, double cx, double cy,
                  int width, int height) override;

private:
  /**
   * @brief 从图像特征重建 3D 位姿（回退方案，当 Target.position 不可用时）。
   *
   * 简化方法：使用边界框中心 + 深度值反投影到相机坐标系。
   * 实际生产中可使用 PnP（Perspective-n-Point）算法获得更精确的位姿。
   *
   * @param features 6 维归一化图像特征 [x_lt, y_lt, x_rb, y_rb, x_rt, y_rt]
   * @param depth    目标深度 (m)，非正数时结果无效
   * @return 相机系下的 3D 位姿（仅平移有意义，旋转设为单位阵）
   */
  Eigen::Isometry3d reconstructPose(const Eigen::Matrix<double, 6, 1>& features, double depth);

  /**
   * @brief 从 Target 消息构建相机系 3D 位姿。
   *
   * 优先使用 Target.position（3D 坐标），缺失时回退到 bbox + depth 反投影。
   * 旋转部分编码了从相机到目标的方向（偏航 + 俯仰角）。
   *
   * @param target 感知管线输出的目标消息
   * @return 相机系下的目标位姿（平移 = 目标坐标，旋转 = 指向目标的姿态）
   */
  Eigen::Isometry3d targetToPose(const vision_servo_msgs::msg::Target& target);

  /**
   * @brief 计算当前位姿与期望位姿之间的笛卡尔误差。
   *
   * 平动误差 = P_current - P_desired（直接减法）
   * 旋转误差 = axis-angle(R_current · R_desired^T)，返回 angle · axis 形式
   *
   * @param current_pose 当前目标在相机系下的位姿
   * @return 6 维误差向量 [ex, ey, ez, eωx, eωy, eωz]^T
   *         — 注意：调用方 computeVelocity() 只用前 3 维（平动），
   *           后 3 维（旋转）已改为从目标方向直接计算
   */
  Eigen::Matrix<double, 6, 1> computePoseError(const Eigen::Isometry3d& current_pose);

  // ── PBVS 控制参数 ──────────────────────────────────────────────
  double translational_gain_;    ///< 平动控制增益 K_t（无量纲），v_trans = +K_t · e_trans
  double rotational_gain_;       ///< 转动控制增益 K_r（无量纲），ω = +K_r · angle · axis

  // ── 位姿状态 ───────────────────────────────────────────────────
  Eigen::Isometry3d desired_pose_;  ///< 期望位姿：平动 = (0, 0, desired_depth)，旋转 = Identity
  Eigen::Isometry3d current_pose_;  ///< 当前目标在相机系下的位姿（每帧由 targetToPose() 更新）
  bool desired_pose_set_;           ///< 是否已通过 setGoalFromTarget() 设置了有效的期望位姿
};

}  // namespace servo_control_pkg
