#pragma once

#include <optional>

#include <vision_servo_msgs/msg/target.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

namespace servo_control_pkg {

inline bool is_actionable_target(const vision_servo_msgs::msg::Target& target)
{
  using Target = vision_servo_msgs::msg::Target;
  return target.visible &&
         (target.tracking_state == Target::TRACKING_STATE_CONFIRMED ||
          target.tracking_state == Target::TRACKING_STATE_UNTRACKED);
}

inline std::optional<vision_servo_msgs::msg::Target> select_actionable_target(
    const vision_servo_msgs::msg::TargetArray& targets)
{
  if (targets.tracking_id >= 0) {
    for (const auto& target : targets.targets) {
      if (target.id == targets.tracking_id && is_actionable_target(target)) {
        return target;
      }
    }
    return std::nullopt;
  }

  for (const auto& target : targets.targets) {
    if (is_actionable_target(target)) {
      return target;
    }
  }
  return std::nullopt;
}

}  // namespace servo_control_pkg
