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
 *   s = [x_lt, y_lt, x_rb, y_rb, x_rt, y_rt]
 *   即目标 bbox 的左上、右下、右上 3 个点，均为归一化图像坐标。
 *   这样 6 维特征可以严格对应 3 个点特征的 6×6 IBVS 交互矩阵。
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
#include <optional>
#include <string>

namespace servo_control_pkg {

/// 视觉伺服的期望特征配置
struct ServoGoal {
  Eigen::Matrix<double, 6, 1> desired_features;  ///< 期望特征向量（3 个归一化 bbox 点）
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
   * @brief 从宿主节点读取控制器参数。
   *
   * 控制器由 pluginlib 创建，不会自动接收 servo_manager 的 YAML 参数。
   * servo_manager 在加载或切换插件后显式调用该方法下发参数。
   */
  virtual void configureFromNode(const rclcpp::Node& node);

  /**
   * @brief 设置期望的视觉特征（示教模式：teaching-by-showing）。
   * @param desired 期望特征向量（3 个归一化 bbox 点）
   * @param depth   期望深度 (m)，-1 表示保持当前深度
   */
  virtual void setDesiredFeatures(const Eigen::Matrix<double, 6, 1>& desired, double depth);

  /**
   * @brief 用当前目标观测生成一个伺服目标。
   * @param target 当前目标观测
   * @param desired_depth 期望深度，<=0 表示保持当前尺度/深度
   * @param feature_tolerance 本次任务收敛阈值，<=0 使用参数默认值
   */
  virtual bool setGoalFromTarget(
    const vision_servo_msgs::msg::Target& target,
    double desired_depth,
    double feature_tolerance);

  bool hasGoal() const { return goal_configured_; }
  double getCurrentErrorNorm() const { return feature_error_.norm(); }
  bool isConverged() const;

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
   *   s[0-1] = (x_min - cx) / fx, (y_min - cy) / fy  → 左上角
   *   s[2-3] = (x_max - cx) / fx, (y_max - cy) / fy  → 右下角
   *   s[4-5] = (x_max - cx) / fx, (y_min - cy) / fy  → 右上角
   *
   * 注意：这里不再混入面积或宽高比。IBVS 的点特征交互矩阵要求每两维
   * 都是一个真实图像点 (x, y)，否则 L 的物理意义会被破坏。
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
  bool goal_configured_;                               ///< 是否已有有效伺服目标
  int iteration_count_;                                ///< 控制迭代计数
  Eigen::Matrix<double, 6, 1> last_camera_velocity_;   ///< 最近一次控制输出

  // ── 控制参数 ────────────────────────────────────────────────────
  double lambda_gain_;     ///< 指数衰减增益（IBVS 的 λ）
  double max_linear_vel_;  ///< 最大线速度 (m/s)
  double max_angular_vel_; ///< 最大角速度 (rad/s)
};

}  // namespace servo_control_pkg
