/**
 * @file math_utils.hpp
 * @brief 数学工具函数 — 钳位、角度归一化、低通滤波等通用工具。
 *
 * 所有函数均为头文件内联实现，无需链接额外的库。
 * 这些工具在底盘驱动、里程计、控制器等多个节点中共享使用。
 */

#pragma once

#include <cmath>
#include <algorithm>

namespace robot_platform_pkg {

/**
 * @brief 将值钳位到 [min_val, max_val] 范围内。
 *
 * 用于速度指令的安全限幅，防止超速。
 */
template <typename T>
inline T clamp(T value, T min_val, T max_val) {
  return std::min(std::max(value, min_val), max_val);
}

/**
 * @brief 将角度归一化到 [-π, π] 区间。
 *
 * 使用 while 循环而非 fmod 是为了处理超出 ±2π 范围的任意角度值。
 * 对于大多数正常角度偏差（±π 附近），只需 1 次循环。
 */
inline double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/**
 * @brief 计算两个角度之间的最短有向距离。
 * @param from 起始角度 (rad)
 * @param to   目标角度 (rad)
 * @return 最短路径的角度差，范围 [-π, π]
 */
inline double angularDistance(double from, double to) {
  return normalizeAngle(to - from);
}

/**
 * @class LowPassFilter
 * @brief 一阶低通滤波器（指数平滑）。
 *
 * 传递函数：y[n] = α·x[n] + (1-α)·y[n-1]
 *
 * α 越接近 1 → 响应越快但噪声抑制越弱
 * α 越接近 0 → 输出越平滑但延迟越大
 * 默认 α = 0.3 对于 50 Hz IMU 数据效果较好。
 */
class LowPassFilter {
public:
  explicit LowPassFilter(double alpha = 0.3) : alpha_(alpha), initialized_(false), value_(0) {}

  /// 输入原始值，返回滤波后值
  double update(double raw) {
    if (!initialized_) { value_ = raw; initialized_ = true; }  // 首次直接用原始值初始化
    else { value_ = alpha_ * raw + (1.0 - alpha_) * value_; }
    return value_;
  }

  /// 重置滤波器状态（下次 update 会重新初始化）
  void reset() { initialized_ = false; }

  double value() const { return value_; }

private:
  double alpha_;       ///< 平滑系数 (0, 1]
  bool initialized_;   ///< 是否已初始化
  double value_;       ///< 当前滤波值
};

}  // namespace robot_platform_pkg
