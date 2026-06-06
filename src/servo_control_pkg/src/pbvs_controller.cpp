/**
 * @file pbvs_controller.cpp
 * @brief PBVS 控制器实现 — 解耦平动/转动控制律。
 *
 * 与 IBVS 的区别：
 *   IBVS 在图像空间中工作，控制律涉及交互矩阵的伪逆
 *   PBVS 先在笛卡尔空间中重建 3D 位姿，再使用解耦的 P 控制器
 *
 * 控制律：
 *   v_translation = -K_t · (current_position - desired_position)
 *   v_rotation    = -K_r · (current_orientation ⊖ desired_orientation)
 *
 * 旋转误差通过轴角表示（AngleAxis），角度 × 轴 = 旋转速度方向
 */

#include "servo_control_pkg/pbvs_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <string>

namespace servo_control_pkg {

PBVSController::PBVSController(const rclcpp::NodeOptions& options)
  : ServoControllerBase("pbvs_controller", options),
    translational_gain_(0.5), rotational_gain_(0.3),
    desired_pose_set_(false)
{
  // ── 声明 PBVS 专属参数 ──────────────────────────────────────────
  this->declare_parameter("translational_gain", 0.5);
  this->declare_parameter("rotational_gain", 0.3);

  translational_gain_ = this->get_parameter("translational_gain").as_double();
  rotational_gain_ = this->get_parameter("rotational_gain").as_double();
}

void PBVSController::configureFromNode(const rclcpp::Node& node) {
  ServoControllerBase::configureFromNode(node);

  auto get_double = [&node](const std::string& name, double fallback) {
    double value = fallback;
    if (node.has_parameter(name)) {
      node.get_parameter(name, value);
    }
    return value;
  };

  translational_gain_ = get_double("translational_gain", translational_gain_);
  rotational_gain_ = get_double("rotational_gain", rotational_gain_);
}

bool PBVSController::initialize(double fx, double fy, double cx, double cy,
                                 int width, int height) {
  if (!ServoControllerBase::initialize(fx, fy, cx, cy, width, height)) return false;

  // 示教模式（teaching-by-showing）：首次调用时记录当前位置作为期望位姿
  // 用户将机器人移动到期望位置后触发标定，系统自动记录
  if (!desired_pose_set_) {
    desired_pose_ = Eigen::Isometry3d::Identity();
    desired_pose_set_ = true;
  }
  return true;
}

bool PBVSController::setGoalFromTarget(
    const vision_servo_msgs::msg::Target& target,
    double desired_depth,
    double feature_tolerance) {
  if (!ServoControllerBase::setGoalFromTarget(target, desired_depth, feature_tolerance)) {
    return false;
  }

  desired_pose_ = targetToPose(target);
  if (desired_depth > 0.0) {
    desired_pose_.translation().z() = desired_depth;
  }
  desired_pose_set_ = true;
  return true;
}

std::optional<Eigen::Matrix<double, 6, 1>> PBVSController::computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt) {
  (void)dt;
  if (!initialized_ || !goal_configured_ || !desired_pose_set_) {
    last_camera_velocity_.setZero();
    return std::nullopt;
  }

  // ── 步骤 1：提取特征 + 重建 3D 位姿 ────────────────────────────
  current_features_ = extractFeatures(target);
  current_pose_ = targetToPose(target);

  // ── 步骤 2：计算笛卡尔误差 ──────────────────────────────────────
  Eigen::Matrix<double, 6, 1> pose_error = computePoseError(current_pose_);
  feature_error_ = pose_error;

  // ── 步骤 3：检查收敛 ────────────────────────────────────────────
  if (pose_error.norm() < goal_.feature_tolerance) {
    last_camera_velocity_.setZero();
    return Eigen::Matrix<double, 6, 1>::Zero();  // 已收敛
  }

  // ── 步骤 4：解耦控制律 v = -K * e ──────────────────────────────
  Eigen::Matrix<double, 6, 1> velocity;
  // 前 3 维：平动速度 = -K_t · 位置误差
  velocity.head<3>() = -translational_gain_ * pose_error.head<3>();
  // 深度轴 (z, index 2): 移动机器人上向前运动会减小目标在相机系中的 z 坐标
  // （靠近目标），因此深度控制需要反向符号，避免正反馈导致机器人加速跑偏。
  velocity(2) = translational_gain_ * pose_error(2);
  // 后 3 维：转动速度 = -K_r · 旋转误差（轴角表示）
  velocity.tail<3>() = -rotational_gain_ * pose_error.tail<3>();

  iteration_count_++;
  last_camera_velocity_ = velocity;
  return velocity;
}

Eigen::Isometry3d PBVSController::reconstructPose(
    const Eigen::Matrix<double, 6, 1>& features, double depth) {
  // 简化的 3D 重建：从边界框中心 + 深度反投影
  // 实际生产中应使用 PnP（Perspective-n-Point）算法获得更精确的旋转估计
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  // 边界框中心在归一化图像坐标中的位置
  double cx_norm = (features(0) + features(2)) / 2.0;  // 平均 x
  double cy_norm = (features(1) + features(3)) / 2.0;  // 平均 y
  // 反投影到相机坐标系：X = x_norm * Z, Y = y_norm * Z, Z = depth
  pose.translation() << cx_norm * depth, cy_norm * depth, depth;

  // 方向估计：当前为简化版，假设无旋转
  // 完整实现可根据边界框的宽高比和倾斜度估计物体朝向
  pose.linear() = Eigen::Matrix3d::Identity();

  return pose;
}

Eigen::Isometry3d PBVSController::targetToPose(
    const vision_servo_msgs::msg::Target& target) {
  const bool has_position =
    std::isfinite(target.position[0]) &&
    std::isfinite(target.position[1]) &&
    std::isfinite(target.position[2]) &&
    target.position[2] > 0.0f;

  if (has_position) {
    double tx = target.position[0];  // 相机系: 右
    double ty = target.position[1];  // 相机系: 下
    double tz = target.position[2];  // 相机系: 前

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() << tx, ty, tz;

    // 计算朝向目标的偏航角和俯仰角
    // 偏航: 目标在水平面上的方位角
    double yaw = std::atan2(tx, tz);
    // 俯仰: 目标在垂直面上的仰角 (正值 = 目标在下方)
    double pitch = std::atan2(ty, std::sqrt(tx * tx + tz * tz));

    // 旋转矩阵: 先偏航 (绕相机系 Z 轴), 再俯仰 (绕相机系 Y 轴负向)
    Eigen::AngleAxisd rot_yaw(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rot_pitch(-pitch, Eigen::Vector3d::UnitY());
    pose.linear() = (rot_yaw * rot_pitch).toRotationMatrix();

    return pose;
  }

  return reconstructPose(extractFeatures(target), target.position[2]);
}

Eigen::Matrix<double, 6, 1> PBVSController::computePoseError(
    const Eigen::Isometry3d& current_pose) {
  Eigen::Matrix<double, 6, 1> error;

  // 平动误差：当前位置 - 期望位置
  error.head<3>() = current_pose.translation() - desired_pose_.translation();

  // 旋转误差（轴角表示）：R_error = R_current * R_desired^T
  // 轴角 = angle · axis，angle ∈ [0, π]，axis 为单位旋转轴
  Eigen::AngleAxisd aa(current_pose.linear() * desired_pose_.linear().transpose());
  error.tail<3>() = aa.angle() * aa.axis();  // 旋转速度方向 = 误差旋转轴

  return error;
}

}  // namespace servo_control_pkg

// 注册为 pluginlib 插件
PLUGINLIB_EXPORT_CLASS(servo_control_pkg::PBVSController,
                       servo_control_pkg::ServoControllerBase)
