#pragma once

#include <algorithm>
#include <cmath>

namespace servo_control_pkg::mvp_yaw_control {

inline double wrapped_angle_error(double current, double reference)
{
  return std::remainder(current - reference, 2.0 * M_PI);
}

inline double base_yaw_command(
  double relative_gimbal_yaw,
  double gain,
  double direction_sign,
  double deadband,
  double max_rate)
{
  if (!std::isfinite(relative_gimbal_yaw) ||
      std::abs(relative_gimbal_yaw) < std::abs(deadband)) {
    return 0.0;
  }
  return std::clamp(
    direction_sign * gain * relative_gimbal_yaw,
    -std::abs(max_rate), std::abs(max_rate));
}

}  // namespace servo_control_pkg::mvp_yaw_control
