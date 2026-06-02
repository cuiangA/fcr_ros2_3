/**
 * @file servo_controller_base.cpp
 * @brief 视觉伺服控制器基类实现 — 参数声明、交互矩阵计算、特征提取、状态查询。
 *
 * 交互矩阵 L 是 IBVS 控制律的核心：v = -λ · L⁺ · e
 * 对 3 个点特征（每点 2 坐标 = 6 维特征向量），
 * L 是 6×6 的分块矩阵 L = [L1; L2; L3]。
 *
 * 每个 2×6 子块 Li(x, y, Z) 的解析形式为：
 *   L = [ -1/Z,   0,   x/Z,   xy,   -(1+x²),  y
 *         0,    -1/Z,  y/Z,  1+y²,    -xy,    -x ]
 *
 * 其中 (x, y) 是归一化图像坐标（已除以焦距）。
 */

#include "servo_control_pkg/servo_controller_base.hpp"
#include <cmath>

namespace servo_control_pkg {

ServoControllerBase::ServoControllerBase(const std::string& node_name,
                                         const rclcpp::NodeOptions& options)
  : Node(node_name, options),
    fx_(0), fy_(0), cx_(0), cy_(0), width_(0), height_(0),
    current_depth_(0), initialized_(false), iteration_count_(0),
    lambda_gain_(0.5), max_linear_vel_(1.0), max_angular_vel_(2.0)
{
  // ── 声明基类参数（所有控制器共享） ──────────────────────────────
  this->declare_parameter("lambda_gain", 0.5);
  this->declare_parameter("max_linear_velocity", 1.0);
  this->declare_parameter("max_angular_velocity", 2.0);
  this->declare_parameter("feature_tolerance", 0.01);
  this->declare_parameter("desired_depth", 2.0);

  lambda_gain_ = this->get_parameter("lambda_gain").as_double();
  max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
  max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
  goal_.feature_tolerance = this->get_parameter("feature_tolerance").as_double();
  goal_.desired_depth = this->get_parameter("desired_depth").as_double();
  goal_.max_linear_velocity = max_linear_vel_;
  goal_.max_angular_velocity = max_angular_vel_;
}

bool ServoControllerBase::initialize(double fx, double fy, double cx, double cy,
                                     int width, int height) {
  fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
  width_ = width; height_ = height;
  initialized_ = true;
  RCLCPP_INFO(get_logger(), "控制器已初始化: fx=%.2f, fy=%.2f, "
              "cx=%.2f, cy=%.2f, %dx%d", fx, fy, cx, cy, width, height);
  return true;
}

void ServoControllerBase::setDesiredFeatures(
    const Eigen::Matrix<double, 6, 1>& desired, double depth) {
  goal_.desired_features = desired;
  // 仅在指定正深度时更新期望深度（-1 表示保持当前值）
  if (depth > 0) goal_.desired_depth = depth;
  RCLCPP_INFO(get_logger(), "期望特征已设置, depth=%.2f", goal_.desired_depth);
}

Eigen::Matrix<double, 6, 6> ServoControllerBase::computeInteractionMatrix(
    const Eigen::Matrix<double, 6, 1>& features, double depth) {
  // 防止除零：深度为 0 或负值时回退到 1.0m
  if (depth <= 0) depth = 1.0;

  // Lambda 函数：计算单个特征点 (x, y) 在深度 Z 处的 2×6 子交互矩阵
  auto Li = [&](double x, double y, double Z) -> Eigen::Matrix<double, 2, 6> {
    Eigen::Matrix<double, 2, 6> L;
    double inv_Z = 1.0 / Z;
    // 经典 IBVS 交互矩阵解析形式
    L << -inv_Z, 0, x * inv_Z, x * y, -(1 + x * x), y,
         0, -inv_Z, y * inv_Z, 1 + y * y, -x * y, -x;
    return L;
  };

  // 构建 6×6 分块矩阵：L = [L1; L2; L3]
  Eigen::Matrix<double, 6, 6> L;
  L.block<2,6>(0,0) = Li(features(0), features(1), depth);  // 点 1: (x_min, y_min)
  L.block<2,6>(2,0) = Li(features(2), features(3), depth);  // 点 2: (x_max, y_max)
  L.block<2,6>(4,0) = Li(features(4), features(5), depth);  // 点 3: (log(area), aspect)

  return L;
}

Eigen::Matrix<double, 6, 1> ServoControllerBase::extractFeatures(
    const vision_servo_msgs::msg::Target& target) {
  // 特征向量定义（6 维归一化图像特征）：
  //   s[0-1]：归一化边界框左上角 (x_min, y_min)
  //   s[2-3]：归一化边界框右下角 (x_max, y_max)
  //   s[4]：  log(面积 + ε) —— 尺度不变性
  //   s[5]：  宽高比 —— 形状变化检测
  Eigen::Matrix<double, 6, 1> f;
  // 归一化：将像素坐标除以焦距，转换为无量纲归一化图像坐标
  f(0) = (target.bbox[0] - cx_) / fx_;  // x_min 归一化
  f(1) = (target.bbox[1] - cy_) / fy_;  // y_min 归一化
  f(2) = (target.bbox[2] - cx_) / fx_;  // x_max 归一化
  f(3) = (target.bbox[3] - cy_) / fy_;  // y_max 归一化
  double area = std::abs((target.bbox[2] - target.bbox[0]) * (target.bbox[3] - target.bbox[1]));
  f(4) = std::log(area + 1e-6);          // 对数面积（+ε 防止 log(0)）
  f(5) = (target.bbox[2] - target.bbox[0]) / (target.bbox[3] - target.bbox[1] + 1e-6); // 宽高比
  return f;
}

vision_servo_msgs::msg::ServoState ServoControllerBase::getServoState() const {
  vision_servo_msgs::msg::ServoState state;
  // 填充当前内部状态
  state.norm_error = feature_error_.norm();          // 当前特征误差范数
  state.desired_norm_error = goal_.feature_tolerance; // 收敛阈值
  state.iteration_count = iteration_count_;

  // 状态机判定：
  //   IDLE → CONVERGING → TRACKING（误差在容差内时进入跟踪稳态）
  if (!initialized_) {
    state.state = vision_servo_msgs::msg::ServoState::IDLE;
  } else if (state.norm_error < goal_.feature_tolerance) {
    state.state = vision_servo_msgs::msg::ServoState::TRACKING;  // 已收敛，维持跟踪
  } else {
    state.state = vision_servo_msgs::msg::ServoState::CONVERGING; // 正在收敛
  }
  return state;
}

}  // namespace servo_control_pkg
