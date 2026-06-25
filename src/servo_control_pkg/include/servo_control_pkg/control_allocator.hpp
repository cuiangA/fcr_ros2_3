/**
 * @file control_allocator.hpp
 * @brief 控制分配器 — 将相机 6-DOF 速度分解为底盘 + 云台指令。
 *
 * 核心问题：视觉伺服控制器输出的是 camera_optical_link 下的 6 自由度速度
 * [vx,vy,vz,ωx,ωy,ωz]，但实际执行器命令位于 base_link：
 *   - 底盘（3 自由度：vx, vy, ω）—— 平面运动
 *   - 云台（2 自由度：yaw_rate, pitch_rate）—— 旋转
 *
 * 当前 MVP 使用 ROS optical frame 到 base_link 的固定轴映射：
 *   base_x = camera_z, base_y = -camera_x, base_yaw = -camera_wy
 *   gimbal_pitch ≈ -camera_wx，camera_wz(roll) 暂不分配。
 *
 * 分配策略（PRIORITY 模式）：
 *   - 旋转由云台优先处理：云台响应快、精度高，适合高频小角度调节
 *   - 底盘处理低频大角度旋转和全部平移
 *   - allocation_ratio 控制旋转在底盘/云台之间的分配比例
 *
 * 指令经过一阶低通滤波器（α = 0.3）平滑，防止指令跳变。
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <vision_servo_msgs/msg/platform_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <Eigen/Dense>
#include <string>

namespace servo_control_pkg {

/// 控制分配结果：底盘 Twist + 云台角速度
struct ControlAllocation {
  geometry_msgs::msg::Twist chassis_twist;  ///< 底盘速度指令
  double gimbal_yaw_rate;                    ///< 云台偏航角速度 (rad/s)
  double gimbal_pitch_rate;                  ///< 云台俯仰角速度 (rad/s)
  bool hold_yaw;                             ///< 是否保持偏航位置（锁定）
  bool hold_pitch;                           ///< 是否保持俯仰位置（锁定）
};

/**
 * @class ControlAllocator
 * @brief 将相机速度按优先级分配给底盘和云台。
 *
 * 分配策略枚举：
 *   - PRIORITY： 云台优先处理旋转，底盘处理剩余
 *   - WEIGHTED： 按权重线性分配
 *   - OPTIMIZATION：通过 QP 求解最优分配（考虑约束）
 */
class ControlAllocator {
public:
  ControlAllocator();

  /**
   * @brief 配置分配器参数。
   * @param gimbal_yaw_limit     云台偏航角限位 (rad)
   * @param gimbal_pitch_limit   云台俯仰角限位 (rad)
   * @param chassis_linear_limit 底盘线速度限制 (m/s)
   * @param chassis_angular_limit 底盘角速度限制 (rad/s)
   * @param allocation_ratio     分配比例 (0=纯云台, 1=纯底盘)
   */
  void configure(double gimbal_yaw_limit, double gimbal_pitch_limit,
                 double chassis_linear_limit, double chassis_angular_limit,
                 double allocation_ratio, double unwind_gain = 0.3,
                 double smoothing_alpha = 0.7);

  /**
   * @brief 执行控制分配。
   * @param camera_velocity 相机坐标系 6-DOF 速度 [vx,vy,vz,ωx,ωy,ωz]
   * @param platform_state  当前平台状态（用于前馈/补偿）
   * @param dt              时间间隔 (s)
   * @return 分配后的底盘和云台指令
   */
  ControlAllocation allocate(const Eigen::Matrix<double, 6, 1>& camera_velocity,
                             const vision_servo_msgs::msg::PlatformState& platform_state,
                             double dt);

private:
  enum class AllocationStrategy { PRIORITY, WEIGHTED, OPTIMIZATION };

  AllocationStrategy strategy_;       ///< 当前分配策略
  double gimbal_yaw_limit_, gimbal_pitch_limit_;    ///< 云台限位
  double chassis_linear_limit_, chassis_angular_limit_; ///< 底盘限位
  double allocation_ratio_;           ///< 分配比例 (0~1)
  double unwind_gain_;               ///< 底盘回中增益（追云台偏角）

  // ── 平滑滤波器 ──────────────────────────────────────────────────
  double prev_gimbal_yaw_, prev_gimbal_pitch_;  ///< 上一帧云台指令（用于平滑）
  double smoothing_alpha_;                       ///< 平滑系数（一阶低通滤波器）
};

}  // namespace servo_control_pkg
