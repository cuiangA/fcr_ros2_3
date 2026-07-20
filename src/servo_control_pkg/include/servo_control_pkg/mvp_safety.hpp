#pragma once

#include <vision_servo_msgs/msg/target.hpp>

#include <cmath>

namespace servo_control_pkg::mvp_safety {

inline bool target_visible_for_control(const vision_servo_msgs::msg::Target& target)
{
  using Target = vision_servo_msgs::msg::Target;
  const bool state_allowed =
    target.tracking_state == Target::TRACKING_STATE_UNTRACKED ||
    target.tracking_state == Target::TRACKING_STATE_CONFIRMED;
  return target.visible && state_allowed;
}

inline bool metric_depth_valid(
  const vision_servo_msgs::msg::Target& target,
  double min_confidence,
  double min_depth,
  double max_depth)
{
  const double depth = target.position[2];
  return std::isfinite(depth) && depth >= min_depth && depth <= max_depth &&
    std::isfinite(target.depth_confidence) &&
    target.depth_confidence >= min_confidence;
}

}  // namespace servo_control_pkg::mvp_safety
