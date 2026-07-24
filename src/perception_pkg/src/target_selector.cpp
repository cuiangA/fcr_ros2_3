#include "perception_pkg/target_selector.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>

namespace perception_pkg {

namespace {

uint64_t stamp_to_ns(const builtin_interfaces::msg::Time& stamp)
{
  return stamp.sec >= 0
      ? static_cast<uint64_t>(stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(stamp.nanosec)
      : 0;
}

}  // namespace

TargetSelector::TargetSelector(
    bool auto_select, std::string class_filter,
    double lost_timeout_seconds, bool allow_auto_switch,
    double switch_delay_seconds)
  : auto_select_(auto_select),
    class_filter_(std::move(class_filter)),
    lost_timeout_seconds_(lost_timeout_seconds),
    allow_auto_switch_(allow_auto_switch),
    switch_delay_seconds_(switch_delay_seconds)
{}

int TargetSelector::update(const vision_servo_msgs::msg::TargetArray& tracks)
{
  latest_tracks_ = tracks;
  const uint64_t now = stamp_to_ns(tracks.header.stamp);

  if (!enabled_) {
    active_id_ = -1;
    state_ = State::NO_TARGET;
    return -1;
  }

  const int track_index = find_track_by_id(active_id_);

  // No active ID or ID was deleted by the tracker
  if (active_id_ < 0 || track_index < 0) {
    if (active_id_ >= 0 && track_index < 0) {
      id_deleted_time_ns_ = now;
      if (manual_lock_ || !auto_select_) {
        state_ = State::WAIT_REACQUIRE;
        switch_reason_ = "Locked ID deleted, waiting for reacquisition";
        active_id_ = -1;
        return -1;
      }
    }

    if (auto_select_) {
      active_id_ = select_best_visible();
      if (active_id_ >= 0) {
        state_ = State::LOCKED_VISIBLE;
        switch_reason_ = "Auto-selected best visible target";
        last_visible_time_ns_ = now;
        manual_lock_ = false;
      } else {
        active_id_ = -1;
        state_ = State::NO_TARGET;
        switch_reason_ = "No visible target available";
      }
    } else {
      active_id_ = -1;
      state_ = State::NO_TARGET;
      switch_reason_ = "No target locked";
    }
    return active_id_;
  }

  // Active ID exists in current tracks
  const auto& target = latest_tracks_.targets[track_index];

  if (target.visible &&
      target.tracking_state == vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED) {
    last_visible_time_ns_ = now;
    state_ = State::LOCKED_VISIBLE;
    switch_reason_ = "Target visible and confirmed";
    return active_id_;
  }

  // Target is not visible: LOST or TENTATIVE
  if (target.tracking_state == vision_servo_msgs::msg::Target::TRACKING_STATE_LOST ||
      target.tracking_state == vision_servo_msgs::msg::Target::TRACKING_STATE_TENTATIVE ||
      !target.visible) {

    if (last_visible_time_ns_ == 0) {
      last_visible_time_ns_ = now;
    }
    const double elapsed_since_visible =
        static_cast<double>(now - last_visible_time_ns_) * 1e-9;
    const double total_timeout = lost_timeout_seconds_ + switch_delay_seconds_;

    if (elapsed_since_visible < lost_timeout_seconds_) {
      // Within recovery window — hold the ID
      state_ = State::LOCKED_PERSON_LOST;
      switch_reason_ = "Person lost, within recovery timeout";
      return active_id_;
    }

    if (elapsed_since_visible < total_timeout) {
      // Past recovery window but within switch delay — keep ID, waiting
      state_ = State::WAIT_REACQUIRE;
      switch_reason_ = "Past recovery timeout, waiting before switch";
      return active_id_;
    }

    // Past both timeouts
    if (allow_auto_switch_ && auto_select_ && !manual_lock_) {
      const int new_id = select_best_visible();
      if (new_id >= 0 && new_id != active_id_) {
        active_id_ = new_id;
        state_ = State::SWITCH_ALLOWED;
        switch_reason_ = "Switched to new target after timeout";
        last_visible_time_ns_ = now;
        manual_lock_ = false;
        return active_id_;
      }
    }

    state_ = State::WAIT_REACQUIRE;
    switch_reason_ = "Person lost, no auto-switch";
    return active_id_;
  }

  return active_id_;
}

bool TargetSelector::request(
    int target_id, const std::string& class_name, bool enable, std::string& message)
{
  if (!enable) {
    enabled_ = false;
    active_id_ = -1;
    state_ = State::NO_TARGET;
    manual_lock_ = false;
    switch_reason_ = "Selector disabled by request";
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
  manual_lock_ = (target_id >= 0);

  if (active_id_ == -1 && auto_select_) {
    active_id_ = select_best_visible();
  }

  if (active_id_ >= 0) {
    state_ = State::LOCKED_VISIBLE;
    switch_reason_ = manual_lock_ ? "Manually locked to target" : "Auto-selected target";
    message = manual_lock_ ? "Target locked (manual)" : "Target locked (auto)";
  } else {
    state_ = State::NO_TARGET;
    if (!auto_select_) {
      switch_reason_ = "Target lock enabled; awaiting a manual target_id";
      message = "Target lock enabled; awaiting a manual target_id";
    } else {
      switch_reason_ = "Auto-select enabled; awaiting target";
      message = "Auto-select enabled; awaiting target";
    }
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

int TargetSelector::find_track_by_id(int id) const
{
  for (size_t i = 0; i < latest_tracks_.targets.size(); ++i) {
    if (latest_tracks_.targets[i].id == id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::string TargetSelector::state_name() const
{
  switch (state_) {
    case State::NO_TARGET:          return "NO_TARGET";
    case State::LOCKED_VISIBLE:     return "LOCKED_VISIBLE";
    case State::LOCKED_FACE_VISIBLE: return "LOCKED_FACE_VISIBLE";
    case State::LOCKED_FACE_LOST:   return "LOCKED_FACE_LOST";
    case State::LOCKED_BODY_ONLY:   return "LOCKED_BODY_ONLY";
    case State::LOCKED_PERSON_LOST: return "LOCKED_PERSON_LOST";
    case State::WAIT_REACQUIRE:     return "WAIT_REACQUIRE";
    case State::SWITCH_ALLOWED:     return "SWITCH_ALLOWED";
    default:                        return "UNKNOWN";
  }
}

}  // namespace perception_pkg
