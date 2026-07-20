#include "perception_pkg/target_selector.hpp"

#include <algorithm>
#include <utility>

namespace perception_pkg {

TargetSelector::TargetSelector(bool auto_select, std::string class_filter)
  : auto_select_(auto_select), class_filter_(std::move(class_filter))
{}

int TargetSelector::update(const vision_servo_msgs::msg::TargetArray& tracks)
{
  latest_tracks_ = tracks;
  const bool active_exists = std::any_of(
      latest_tracks_.targets.begin(), latest_tracks_.targets.end(),
      [this](const auto& target) { return target.id == active_id_; });
  if (!active_exists) {
    active_id_ = -1;
  }
  if (enabled_ && auto_select_ && active_id_ < 0) {
    active_id_ = select_best_visible();
  }
  return enabled_ ? active_id_ : -1;
}

bool TargetSelector::request(
    int target_id, const std::string& class_name, bool enable, std::string& message)
{
  if (!enable) {
    enabled_ = false;
    active_id_ = -1;
    if (!class_name.empty()) {
      class_filter_ = class_name;
    }
    message = "Target lock disabled; 2D tracking continues publishing";
    return true;
  }
  if (target_id < -1) {
    message = "target_id must be -1 for auto-select or a non-negative confirmed track ID";
    return false;
  }
  if (target_id >= 0) {
    const auto target = std::find_if(
        latest_tracks_.targets.begin(), latest_tracks_.targets.end(),
        [target_id](const auto& candidate) { return candidate.id == target_id; });
    if (target == latest_tracks_.targets.end()) {
      message = "Requested target_id is not a currently confirmed track";
      return false;
    }
    if (target->tracking_state !=
            vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED &&
        target->tracking_state !=
            vision_servo_msgs::msg::Target::TRACKING_STATE_LOST) {
      message = "Requested target_id is still tentative";
      return false;
    }
    if (!class_name.empty() && target->class_name != class_name) {
      message = "Requested target_id does not match class_name";
      return false;
    }
  }

  enabled_ = true;
  if (!class_name.empty()) {
    class_filter_ = class_name;
  }
  active_id_ = target_id;
  if (active_id_ == -1 && auto_select_) {
    active_id_ = select_best_visible();
  }
  if (active_id_ >= 0) {
    message = "Target locked";
  } else if (auto_select_) {
    message = "Auto-select enabled; awaiting target";
  } else {
    message = "Target lock enabled; awaiting a manual target_id";
  }
  return true;
}

int TargetSelector::select_best_visible() const
{
  int best_id = -1;
  float best_score = -1.0F;
  for (const auto& target : latest_tracks_.targets) {
    if (!target.visible ||
        target.tracking_state !=
            vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED ||
        (!class_filter_.empty() && target.class_name != class_filter_)) {
      continue;
    }
    const float area = std::max(0.0F, target.width) * std::max(0.0F, target.height);
    const float score = area * std::max(0.0F, target.confidence);
    if (score > best_score) {
      best_score = score;
      best_id = target.id;
    }
  }
  return best_id;
}

}  // namespace perception_pkg
