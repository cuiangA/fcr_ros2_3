/**
 * @file servo_controller_base.hpp
 * @brief 所有视觉伺服控制器的抽象基类（IBVS、PBVS、MPC、RL）。
 *
 * 采用策略模式（Strategy Pattern）：
 *   所有控制器继承 ServoControllerBase，实现自己的 computeVelocity()。
 *   servo_manager_node 通过 pluginlib 动态加载具体的控制器实现。
 *
 * 核心方法：
 *   - computeVelocity()：纯虚函数，子类必须实现——给定目标观测，计算相机速度
 *   - computeInteractionMatrix()：计算图像雅可比（交互矩阵），IBVS 控制律的基础
 *   - extractFeatures()：从 Target 消息提取归一化图像特征向量
 *
 * 特征向量定义（6 维）：
 *   s = [x1, y1, x2, y2, log(area), aspect_ratio]
 *   前 4 维为归一化图像坐标（除以 fx/fy），后 2 维描述边界框的尺度和形状。
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>
#include <vision_servo_msgs/msg/servo_state.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace servo_control_pkg {

/// 视觉伺服的期望特征配置
struct ServoGoal {
  Eigen::Matrix<double, 6, 1> desired_features;  ///< 期望特征向量（6 维归一化图像特征）
  double desired_depth;                            ///< 期望目标深度 (m)，-1 = 保持当前深度
  double feature_tolerance;                        ///< 收敛阈值（特征误差范数小于此值视为已收敛）
  double max_linear_velocity;                      ///< 最大线速度限制 (m/s)
  double max_angular_velocity;                     ///< 最大角速度限制 (rad/s)
};

/**
 * @class ServoControllerBase
 * @brief 视觉伺服控制器抽象基类。
 *
 * 所有控制器插件（IBVS / PBVS / MPC / RL）均继承此类。
 * 子类只需实现 computeVelocity() 和 getControllerType() 即可。
 *
 * 控制律通用框架（以 IBVS 为例）：
 *   v = -λ · L⁺ · (s_current - s_desired)
 *   其中 v = 相机速度 [vx,vy,vz,ωx,ωy,ωz]^T
 *        λ = 控制增益
 *        L⁺ = 交互矩阵的伪逆
 *        s  = 图像特征向量
 */
class ServoControllerBase : public rclcpp::Node {
public:
  explicit ServoControllerBase(const std::string& node_name,
                               const rclcpp::NodeOptions& options);

  virtual ~ServoControllerBase() = default;

  /**
   * @brief 使用相机内参初始化控制器。
   * @param fx, fy 焦距（像素）
   * @param cx, cy 主点（像素）
   * @param width, height 图像分辨率
   */
  virtual bool initialize(double fx, double fy, double cx, double cy,
                          int width, int height);

  /**
   * @brief 设置期望的视觉特征（示教模式：teaching-by-showing）。
   * @param desired 期望特征向量（6 维归一化图像特征）
   * @param depth   期望深度 (m)，-1 表示保持当前深度
   */
  virtual void setDesiredFeatures(const Eigen::Matrix<double, 6, 1>& desired, double depth);

  /**
   * @brief 主控制迭代：根据当前目标观测计算相机速度。
   * @param target 当前跟踪目标（包含 3D 位置和边界框）
   * @param dt     距上次更新的时间间隔 (s)
   * @return 相机坐标系下的速度向量 [vx,vy,vz,ωx,ωy,ωz]^T，
   *         若已收敛或未初始化则返回 nullopt
   */
  virtual std::optional<Eigen::Matrix<double, 6, 1>> computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt) = 0;

  /// 获取控制器类型名称（用于日志和调试）
  virtual std::string getControllerType() const = 0;

  /// 获取当前伺服状态（误差范数、迭代次数、状态机状态）
  vision_servo_msgs::msg::ServoState getServoState() const;

protected:
  /**
   * @brief 计算交互矩阵（图像雅可比）在给定特征点和深度处的值。
   *
   * 对 3 个点特征（每点 2 坐标 = 6 维特征），构建完整的 6×6 矩阵：
   *   L = [L1; L2; L3]
   *   每个 Li(x, y, Z) = [ -1/Z, 0, x/Z, xy, -(1+x²), y
   *                        0, -1/Z, y/Z, 1+y², -xy, -x ]
   *
   * @param features 当前特征向量（6 维归一化图像坐标）
   * @param depth    特征点的深度 (m)
   * @return 6×6 交互矩阵
   */
  Eigen::Matrix<double, 6, 6> computeInteractionMatrix(
    const Eigen::Matrix<double, 6, 1>& features, double depth);

  /**
   * @brief 从 Target 消息中提取归一化图像特征向量。
   *
   * 特征向量定义：
   *   s[0-1] = (x_min - cx) / fx, (y_min - cy) / fy  → 归一化左上角
   *   s[2-3] = (x_max - cx) / fx, (y_max - cy) / fy  → 归一化右下角
   *   s[4]   = log(area + ε)                           → 对数面积（尺度不变性）
   *   s[5]   = (w / (h + ε))                            → 宽高比
   */
  Eigen::Matrix<double, 6, 1> extractFeatures(const vision_servo_msgs::msg::Target& target);

  // ── 相机内参 ────────────────────────────────────────────────────
  double fx_, fy_, cx_, cy_;   ///< 针孔模型内参
  int width_, height_;          ///< 图像分辨率

  // ── 当前状态 ────────────────────────────────────────────────────
  ServoGoal goal_;                                     ///< 期望目标配置
  Eigen::Matrix<double, 6, 1> current_features_;       ///< 当前特征向量
  Eigen::Matrix<double, 6, 1> feature_error_;          ///< 特征误差 s - s*
  Eigen::Matrix<double, 6, 6> interaction_matrix_;     ///< 当前交互矩阵 L
  double current_depth_;                               ///< 当前目标深度 (m)
  bool initialized_;                                   ///< 标定是否完成
  int iteration_count_;                                ///< 控制迭代计数

  // ── 控制参数 ────────────────────────────────────────────────────
  double lambda_gain_;     ///< 指数衰减增益（IBVS 的 λ）
  double max_linear_vel_;  ///< 最大线速度 (m/s)
  double max_angular_vel_; ///< 最大角速度 (rad/s)
};

}  // namespace servo_control_pkg
