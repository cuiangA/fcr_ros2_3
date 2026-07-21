#pragma once

#include <cstdint>
#include <string>

#include <vision_servo_msgs/msg/target_array.hpp>

namespace perception_pkg {

class TargetSelector {
public:
  enum class State : uint8_t {
    NO_TARGET = 0,
    LOCKED_VISIBLE = 1,
    LOCKED_FACE_VISIBLE = 2,
    LOCKED_FACE_LOST = 3,
    LOCKED_BODY_ONLY = 4,
    LOCKED_PERSON_LOST = 5,
    WAIT_REACQUIRE = 6,
    SWITCH_ALLOWED = 7
  };

  TargetSelector(
    bool auto_select = true,
    std::string class_filter = "person",
    double lost_timeout_seconds = 2.5,
    bool allow_auto_switch = false,
    double switch_delay_seconds = 0.0);

  int update(const vision_servo_msgs::msg::TargetArray& tracks);
  bool request(int target_id, const std::string& class_name, bool enable, std::string& message);

  int active_id() const { return active_id_; }
  bool enabled() const { return enabled_; }
  State state() const { return state_; }
  std::string state_name() const;
  std::string switch_reason() const { return switch_reason_; }
  bool manual_lock() const { return manual_lock_; }

private:
  int select_best_visible() const;
  int find_track_by_id(int id) const;

  bool auto_select_ = true;
  std::string class_filter_;
  bool enabled_ = true;
  int active_id_ = -1;
  State state_ = State::NO_TARGET;
  bool manual_lock_ = false;
  double lost_timeout_seconds_ = 2.5;
  bool allow_auto_switch_ = false;
  double switch_delay_seconds_ = 0.0;
  std::string switch_reason_;
  uint64_t last_visible_time_ns_ = 0;
  uint64_t last_lost_time_ns_ = 0;
  uint64_t id_deleted_time_ns_ = 0;
  vision_servo_msgs::msg::TargetArray latest_tracks_;
};

}  // namespace perception_pkg
