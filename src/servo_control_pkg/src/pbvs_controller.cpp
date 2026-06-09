/**
 * @file pbvs_controller.cpp
 * @brief PBVS 控制器实现 — 解耦平动/转动控制律。
 *
 * 与 IBVS 的区别：
 *   IBVS 在图像空间中工作，控制律涉及交互矩阵的伪逆
 *   PBVS 先在笛卡尔空间中重建 3D 位姿，再使用解耦的 P 控制器
 *
 * 控制律（2026-06 修正）：
 *   平动: v_trans = +K_t · (P_current - P_desired)
 *     — 运动学 dP_c/dt = -v_c，收敛要求 v_c = +K*e
 *   转动: ω = +K_r · angle · rotation_axis
 *     — 从相机光轴对准目标方向的几何叉积直接计算
 *
 * 坐标系约定：
 *   相机光心系 (ROS optical frame): x 右, y 下, z 前（光轴方向）
 *   base_link:                      x 前, y 左, z 上
 *   控制分配器负责两者之间的坐标映射
 */

#include "servo_control_pkg/pbvs_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <string>

namespace servo_control_pkg {

// ══════════════════════════════════════════════════════════════════════════════
// 构造 / 配置
// ══════════════════════════════════════════════════════════════════════════════

PBVSController::PBVSController(const rclcpp::NodeOptions& options)
  : ServoControllerBase("pbvs_controller", options),
    translational_gain_(0.5),   // 平动增益默认值（会被 YAML / configureFromNode 覆盖）
    rotational_gain_(0.3),      // 转动增益默认值（同理可被覆盖）
    desired_pose_set_(false)    // 尚未设置期望位姿，需要 setGoalFromTarget() 激活
{
  // ── 声明 PBVS 专属参数到 ROS 参数服务器 ──────────────────────────
  // declare_parameter 的第二个参数是"未设置时的回退默认值"。
  // YAML 中 servo_manager.ros__parameters 下同名键会覆盖此默认值。
  this->declare_parameter("translational_gain", 0.5);  // K_t，平动 P 控制器增益
  this->declare_parameter("rotational_gain", 0.3);     // K_r，转动 P 控制器增益

  // 从参数服务器读取当前值（可能是 YAML 覆盖后的值）
  translational_gain_ = this->get_parameter("translational_gain").as_double();
  rotational_gain_ = this->get_parameter("rotational_gain").as_double();
}

void PBVSController::configureFromNode(const rclcpp::Node& node) {
  ServoControllerBase::configureFromNode(node);

  // 辅助 lambda：从 node 读取参数，若不存在则返回 fallback 值
  auto get_double = [&node](const std::string& name, double fallback) {
    double value = fallback;  // 先设为回退值
    if (node.has_parameter(name)) {
      node.get_parameter(name, value);  // 存在则覆盖
    }
    return value;
  };

  translational_gain_ = get_double("translational_gain", translational_gain_);
  rotational_gain_ = get_double("rotational_gain", rotational_gain_);
}

// ══════════════════════════════════════════════════════════════════════════════
// 生命周期管理
// ══════════════════════════════════════════════════════════════════════════════

bool PBVSController::initialize(
    double fx,    // 焦距 x (像素)，CameraInfo::k[0]
    double fy,    // 焦距 y (像素)，CameraInfo::k[4]
    double cx,    // 主点 x (像素)，CameraInfo::k[2]
    double cy,    // 主点 y (像素)，CameraInfo::k[5]
    int width,    // 图像宽度 (像素)
    int height) { // 图像高度 (像素)
  if (!ServoControllerBase::initialize(fx, fy, cx, cy, width, height)) return false;

  // 示教模式（teaching-by-showing）：首次初始化时将期望位姿设为原点。
  // 实际上期望位姿由 setGoalFromTarget() 在每次跟踪启动时动态配置，
  // 此处仅防止 desired_pose_ 处于未初始化状态。
  if (!desired_pose_set_) {
    desired_pose_ = Eigen::Isometry3d::Identity();  // 平动 (0,0,0)，旋转 I
    desired_pose_set_ = true;
  }
  return true;
}

bool PBVSController::setGoalFromTarget(
    const vision_servo_msgs::msg::Target& target,  // 当前目标观测
    double desired_depth,                          // 期望深度 (m)，目标保持在相机正前方此距离
    double feature_tolerance) {                    // 收敛阈值，误差范数 < 此值视为跟踪到位
  if (!ServoControllerBase::setGoalFromTarget(target, desired_depth, feature_tolerance)) {
    return false;
  }

  // 对移动目标跟踪，期望始终是将目标保持在相机正前方且深度匹配。
  // 使用 (0, 0, desired_depth) 而非目标当前的 (x, y)，否则控制器会试图
  // 回到目标首次出现的位置，而不是持续跟踪。
  desired_pose_ = Eigen::Isometry3d::Identity();  // 旋转 = I（期望相机正对目标）
  desired_pose_.translation().z() = desired_depth; // 仅深度轴为非零期望值
  desired_pose_set_ = true;
  return true;
}

// ══════════════════════════════════════════════════════════════════════════════
// 核心控制律
// ══════════════════════════════════════════════════════════════════════════════

std::optional<Eigen::Matrix<double, 6, 1>> PBVSController::computeVelocity(
    const vision_servo_msgs::msg::Target& target,  // 当前帧目标观测
    double dt) {                                    // 时间步长 (s)，PBVS 未使用（无状态依赖）
  (void)dt;  // 显式标记未使用，P 控制器不依赖时间步长

  // 前置检查：必须已完成标定、已设置目标、已配置期望位姿
  if (!initialized_ || !goal_configured_ || !desired_pose_set_) {
    last_camera_velocity_.setZero();  // 未就绪 → 输出零速度
    return std::nullopt;
  }

  // ── 步骤 1：提取特征 + 重建 3D 位姿 ────────────────────────────
  current_features_ = extractFeatures(target);   // 6 维归一化图像特征 [左上有下右上]
  current_pose_ = targetToPose(target);          // 相机系 3D 位姿（平移 + 朝向）

  // ── 步骤 2：计算笛卡尔误差 ──────────────────────────────────────
  // pose_error: 6 维 [ex,ey,ez, eωx,eωy,eωz]
  //   前 3 维 = P_current - P_desired，直接用于平动控制
  //   后 3 维 = axis-angle(R_current·R_desired^T)，仅用于收敛判断（不再用于转动控制）
  Eigen::Matrix<double, 6, 1> pose_error = computePoseError(current_pose_);
  feature_error_ = pose_error;  // 缓存误差向量，供 getServoState() 上报

  // ── 步骤 3：检查收敛 ────────────────────────────────────────────
  if (pose_error.norm() < goal_.feature_tolerance) {
    last_camera_velocity_.setZero();
    return Eigen::Matrix<double, 6, 1>::Zero();  // 已收敛，输出零速度
  }

  // ── 步骤 4：解耦控制律 v = +K * e ──────────────────────────────
  //
  // 运动学推导（相机系中静止世界点的运动）：
  //   对纯平动：dP_c/dt = -v_c
  //   设 dP_c/dt = -K * (P_c - P_desired)  →  v_c = +K * (P_c - P_desired)
  //   因此平动轴均使用正增益。若用负增益会导致正反馈发散。
  //
  // velocity: 相机系 6-DOF 速度 [vx,vy,vz, ωx,ωy,ωz]
  //   vx,vy,vz:  线速度 (m/s)，相机光心系
  //   ωx,ωy,ωz: 角速度 (rad/s)，相机光心系
  Eigen::Matrix<double, 6, 1> velocity;

  // 平动速度 [vx, vy, vz] = +K_t * [ex, ey, ez]
  velocity.head<3>() = translational_gain_ * pose_error.head<3>();

  // 转动速度 [ωx, ωy, ωz]：直接从目标方向计算。
  //
  // 原因：targetToPose() 将方向编码为 R_z(yaw)*R_y(pitch)，axis-angle
  // 分解对纯水平偏移主要产生 ωz（光轴滚动），而控制分配器只用 ωx,ωy。
  //
  // 几何方法：相机光轴 (0,0,1) 对准目标方向 d = (tx,ty,tz)/|t|
  //   旋转轴 = (0,0,1) × d = (-ty, tx, 0)  （归一化后）
  //   旋转角 = acos(tz / |t|)
  //   ω = K_r * angle * axis
  {
    // ── 目标在相机光心系下的 3D 坐标 ──
    double tx = target.position[0];  // 相机系 x: 右为正
    double ty = target.position[1];  // 相机系 y: 下为正
    double tz = target.position[2];  // 相机系 z: 前为正（深度）

    // 目标到相机的欧氏距离
    double t_norm = std::sqrt(tx*tx + ty*ty + tz*tz);

    if (t_norm > 1e-9 && tz > 0.0) {
      // 光轴与目标方向之间的夹角（单位 rad）
      // clamp 防止浮点精度导致 acos 参数越界
      double angle = std::acos(std::clamp(tz / t_norm, -1.0, 1.0));

      // 叉积 (0,0,1) × (tx,ty,tz)/t_norm = (-ty/t_norm, tx/t_norm, 0)
      double ax = -ty / t_norm;  // 旋转轴 x 分量（→ ωx，俯仰方向）
      double ay =  tx / t_norm;  // 旋转轴 y 分量（→ ωy，偏航方向）

      // 旋转轴在 xy 平面内的模长（排除退化情况：目标恰好在光轴上）
      double axy = std::sqrt(ax*ax + ay*ay);

      if (axy > 1e-9) {
        // 归一化 + 缩放：ωi = K_r * angle * axis_i / |axis_xy|
        velocity(3) = rotational_gain_ * angle * ax / axy;  // ωx → 控制分配器映射为俯仰
        velocity(4) = rotational_gain_ * angle * ay / axy;  // ωy → 控制分配器映射为偏航
      } else {
        // 目标恰好在光轴正前方，无需旋转
        velocity(3) = 0.0;  // ωx = 0
        velocity(4) = 0.0;  // ωy = 0
      }
      // ωz（光轴滚动）当前 MVP 不分配给任何执行器
      velocity(5) = 0.0;
    } else {
      // 深度无效或目标在原点 → 无法计算方向，旋转清零
      velocity.tail<3>().setZero();  // [ωx, ωy, ωz] = [0, 0, 0]
    }
  }

  iteration_count_++;                       // 累计控制迭代次数
  last_camera_velocity_ = velocity;         // 缓存本次输出（用于状态上报）
  return velocity;
}

// ══════════════════════════════════════════════════════════════════════════════
// 3D 重建
// ══════════════════════════════════════════════════════════════════════════════

Eigen::Isometry3d PBVSController::reconstructPose(
    const Eigen::Matrix<double, 6, 1>& features,  // 6 维归一化图像特征
    double depth) {                                // 目标深度 (m)，<=0 时结果无效
  // 简化的 3D 重建：从边界框中心 + 深度反投影
  // 实际生产中应使用 PnP（Perspective-n-Point）算法获得更精确的旋转估计
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  // bbox 中心在归一化图像坐标中的位置
  // features: [x_lt, y_lt, x_rb, y_rb, x_rt, y_rt]
  double cx_norm = (features(0) + features(2)) / 2.0;  // bbox 水平中心归一化坐标
  double cy_norm = (features(1) + features(3)) / 2.0;  // bbox 垂直中心归一化坐标

  // 反投影：X_cam = x_norm * Z,  Y_cam = y_norm * Z,  Z_cam = depth
  pose.translation() << cx_norm * depth, cy_norm * depth, depth;

  // 方向估计：简化版假设目标无旋转（正面朝向相机）
  // 完整实现可根据边界框的宽高比和倾斜度估计物体朝向
  pose.linear() = Eigen::Matrix3d::Identity();  // 3×3 单位旋转矩阵

  return pose;
}

Eigen::Isometry3d PBVSController::targetToPose(
    const vision_servo_msgs::msg::Target& target) {  // 感知管线输出的目标消息

  // 检查 Target.position 是否有效（3 个分量均为有限值，且深度 > 0）
  const bool has_position =
    std::isfinite(target.position[0]) &&  // x 坐标是否有限
    std::isfinite(target.position[1]) &&  // y 坐标是否有限
    std::isfinite(target.position[2]) &&  // z 坐标是否有限
    target.position[2] > 0.0f;            // 深度必须为正（目标在相机前方）

  if (has_position) {
    // ── 使用感知管线的 3D 位置（优先路径） ─────────────────────────
    double tx = target.position[0];  // 相机系 x 坐标 (m)，右为正
    double ty = target.position[1];  // 相机系 y 坐标 (m)，下为正
    double tz = target.position[2];  // 相机系 z 坐标 (m)，前为正（深度）

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() << tx, ty, tz;  // 平移部分：目标在相机系下的位置

    // ── 计算朝向目标的偏航角和俯仰角 ──────────────────────────────
    // yaw:   目标在 xz 平面（水平面）上的方位角，atan2(x, z)
    // pitch: 目标在垂直面上的仰角，atan2(y, sqrt(x²+z²))
    //        正值 = 目标在相机下方（光学系 y 轴向下）
    double yaw = std::atan2(tx, tz);
    double pitch = std::atan2(ty, std::sqrt(tx * tx + tz * tz));

    // 旋转矩阵：先偏航（绕相机系 Z 轴），再俯仰（绕相机系 Y 轴负向）
    // 注：此旋转编码了"从相机指向目标的方向"，用于 computePoseError
    //     中的旋转误差计算。但 computeVelocity() 已改用直接几何法。
    Eigen::AngleAxisd rot_yaw(yaw, Eigen::Vector3d::UnitZ());         // R_z(yaw)
    Eigen::AngleAxisd rot_pitch(-pitch, Eigen::Vector3d::UnitY());    // R_y(-pitch)
    pose.linear() = (rot_yaw * rot_pitch).toRotationMatrix();         // R = R_z * R_y

    return pose;
  }

  // 回退路径：Target.position 不可用 → 用 bbox + depth 反投影
  return reconstructPose(extractFeatures(target), target.position[2]);
}

Eigen::Matrix<double, 6, 1> PBVSController::computePoseError(
    const Eigen::Isometry3d& current_pose) {  // 当前目标在相机系下的位姿

  Eigen::Matrix<double, 6, 1> error;  // 6 维误差 [ex,ey,ez, eωx,eωy,eωz]

  // 平动误差 = P_current - P_desired
  // desired_pose_.translation() = (0, 0, desired_depth)
  // 实际效果: error.head<3>() = (tx, ty, tz - desired_depth)
  error.head<3>() = current_pose.translation() - desired_pose_.translation();

  // 旋转误差（轴角表示）
  //   R_error = R_current · R_desired^T （desired 为 Identity 时 = R_current）
  //   axis-angle: 旋转角 angle ∈ [0, π]，旋转轴 axis 为单位向量
  //   error.tail<3>() = angle · axis  （幅值 = 旋转角度，方向 = 旋转轴）
  //
  // 注意：computeVelocity() 已不再使用此后 3 维做转动控制，
  //       但 pose_error.norm() 仍包含它，影响收敛判断。
  Eigen::AngleAxisd aa(current_pose.linear() * desired_pose_.linear().transpose());
  error.tail<3>() = aa.angle() * aa.axis();

  return error;
}

}  // namespace servo_control_pkg

// 注册为 pluginlib 插件，使其可通过 ClassLoader 动态加载和热切换
PLUGINLIB_EXPORT_CLASS(servo_control_pkg::PBVSController,
                       servo_control_pkg::ServoControllerBase)
