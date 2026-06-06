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
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>

namespace servo_control_pkg {

namespace {

bool bboxLooksNormalized(const vision_servo_msgs::msg::Target& target) {
  return std::all_of(target.bbox.begin(), target.bbox.end(), [](float value) {
    return std::isfinite(value) && value >= 0.0f && value <= 1.0f;
  });
}

}  // namespace

ServoControllerBase::ServoControllerBase(const std::string& node_name,
                                         const rclcpp::NodeOptions& options)
  : Node(node_name, options),
    fx_(0), fy_(0), cx_(0), cy_(0), width_(0), height_(0),
    current_depth_(0), initialized_(false), goal_configured_(false), iteration_count_(0),
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

  goal_.desired_features.setZero();
  current_features_.setZero();
  feature_error_.setZero();
  interaction_matrix_.setZero();
  last_camera_velocity_.setZero();
}

void ServoControllerBase::configureFromNode(const rclcpp::Node& node) {
  auto get_double = [&node](const std::string& name, double fallback) {
    double value = fallback;
    if (node.has_parameter(name)) {
      node.get_parameter(name, value);
    }
    return value;
  };

  lambda_gain_ = get_double("lambda_gain", lambda_gain_);
  max_linear_vel_ = get_double("max_linear_velocity", max_linear_vel_);
  max_angular_vel_ = get_double("max_angular_velocity", max_angular_vel_);
  goal_.feature_tolerance = get_double("feature_tolerance", goal_.feature_tolerance);
  goal_.desired_depth = get_double("desired_depth", goal_.desired_depth);
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
  goal_configured_ = true;
  iteration_count_ = 0;
  RCLCPP_INFO(get_logger(), "期望特征已设置, depth=%.2f", goal_.desired_depth);
}

bool ServoControllerBase::setGoalFromTarget(
    const vision_servo_msgs::msg::Target& target,
    double desired_depth,
    double feature_tolerance) {
  if (!initialized_) {
    RCLCPP_WARN(get_logger(), "控制器尚未初始化相机内参，无法设置伺服目标");
    return false;
  }

  const bool normalized_bbox = bboxLooksNormalized(target);
  const double x_min = normalized_bbox ? target.bbox[0] * width_ : target.bbox[0];
  const double y_min = normalized_bbox ? target.bbox[1] * height_ : target.bbox[1];
  const double x_max = normalized_bbox ? target.bbox[2] * width_ : target.bbox[2];
  const double y_max = normalized_bbox ? target.bbox[3] * height_ : target.bbox[3];
  const double bbox_w = x_max - x_min;
  const double bbox_h = y_max - y_min;
  if (!std::isfinite(bbox_w) || !std::isfinite(bbox_h) ||
      bbox_w <= 1.0 || bbox_h <= 1.0) {
    RCLCPP_WARN(get_logger(), "目标 bbox 无效，拒绝设置伺服目标");
    return false;
  }

  auto desired_target = target;
  if (feature_tolerance > 0.0) {
    goal_.feature_tolerance = feature_tolerance;
  }

  const double current_depth = target.position[2];
  if (desired_depth > 0.0 && current_depth > 0.0) {
    const double scale = std::clamp(current_depth / desired_depth, 0.1, 10.0);
    const double desired_w = std::max(1.0, bbox_w * scale);
    const double desired_h = std::max(1.0, bbox_h * scale);

    // IBVS 的期望图像特征直接定义在图像中心：云台负责把目标中心拉回
    // 主点附近，尺度由 desired_depth 决定。这比“记录当前目标中心”为期望
    // 更适合跟拍闭环，否则目标偏在画面左/右时也会被视为已达目标。
    desired_target.bbox = {
      static_cast<float>(cx_ - 0.5 * desired_w),
      static_cast<float>(cy_ - 0.5 * desired_h),
      static_cast<float>(cx_ + 0.5 * desired_w),
      static_cast<float>(cy_ + 0.5 * desired_h)
    };
    desired_target.center = {static_cast<float>(cx_), static_cast<float>(cy_)};
    desired_target.position[2] = static_cast<float>(desired_depth);
  }

  setDesiredFeatures(extractFeatures(desired_target), desired_depth);
  return true;
}

bool ServoControllerBase::isConverged() const {
  return initialized_ && goal_configured_ &&
         feature_error_.norm() < goal_.feature_tolerance;
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
  L.block<2,6>(4,0) = Li(features(4), features(5), depth);  // 点 3: (x_max, y_min)

  return L;
}

Eigen::Matrix<double, 6, 1> ServoControllerBase::extractFeatures(
    const vision_servo_msgs::msg::Target& target) {
  // 特征向量定义（6 维 = 3 个点 × 2）：
  //   s[0-1]：bbox 左上角
  //   s[2-3]：bbox 右下角
  //   s[4-5]：bbox 右上角
  //
  // 早期版本把 s[4-5] 写成 log(area) 和 aspect_ratio，但交互矩阵
  // computeInteractionMatrix() 使用的是“点特征”解析式。现在三组特征
  // 全部是真实图像点，L 的每个 2×6 子块都有明确物理意义。
  Eigen::Matrix<double, 6, 1> f;
  const bool normalized_bbox = bboxLooksNormalized(target);
  const double x_min = normalized_bbox ? target.bbox[0] * width_ : target.bbox[0];
  const double y_min = normalized_bbox ? target.bbox[1] * height_ : target.bbox[1];
  const double x_max = normalized_bbox ? target.bbox[2] * width_ : target.bbox[2];
  const double y_max = normalized_bbox ? target.bbox[3] * height_ : target.bbox[3];

  // 像素坐标通过相机内参归一化为针孔模型坐标：
  // x = (u - cx) / fx, y = (v - cy) / fy。
  f(0) = (x_min - cx_) / fx_;
  f(1) = (y_min - cy_) / fy_;
  f(2) = (x_max - cx_) / fx_;
  f(3) = (y_max - cy_) / fy_;
  f(4) = (x_max - cx_) / fx_;
  f(5) = (y_min - cy_) / fy_;
  return f;
}

vision_servo_msgs::msg::ServoState ServoControllerBase::getServoState() const {
  vision_servo_msgs::msg::ServoState state;
  // 填充当前内部状态
  state.norm_error = feature_error_.norm();          // 当前特征误差范数
  state.desired_norm_error = goal_.feature_tolerance; // 收敛阈值
  state.iteration_count = iteration_count_;
  const auto now = this->now();
  const int64_t now_ns = now.nanoseconds();
  state.last_update.sec = static_cast<int32_t>(now_ns / 1000000000LL);
  state.last_update.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);

  for (size_t i = 0; i < 6; ++i) {
    state.feature_error[i] = static_cast<float>(feature_error_(i));
    state.camera_velocity[i] = static_cast<float>(last_camera_velocity_(i));
  }

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(interaction_matrix_);
  const auto singular_values = svd.singularValues().cwiseAbs();
  const double min_sv = singular_values.minCoeff();
  const double max_sv = singular_values.maxCoeff();
  state.condition_number = (min_sv > 1e-9)
    ? static_cast<float>(max_sv / min_sv)
    : std::numeric_limits<float>::infinity();

  // 状态机判定：
  //   IDLE → CONVERGING → TRACKING（误差在容差内时进入跟踪稳态）
  if (!initialized_) {
    state.state = vision_servo_msgs::msg::ServoState::IDLE;
  } else if (!goal_configured_) {
    state.state = vision_servo_msgs::msg::ServoState::IDLE;
  } else if (state.norm_error < goal_.feature_tolerance) {
    state.state = vision_servo_msgs::msg::ServoState::TRACKING;  // 已收敛，维持跟踪
  } else {
    state.state = vision_servo_msgs::msg::ServoState::CONVERGING; // 正在收敛
  }
  return state;
}

}  // namespace servo_control_pkg
