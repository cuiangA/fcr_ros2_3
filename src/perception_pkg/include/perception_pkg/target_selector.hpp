#pragma once

#include <string>

#include <vision_servo_msgs/msg/target_array.hpp>

namespace perception_pkg {

/** Pure state machine for selecting one confirmed 2D track. */
class TargetSelector {
public:
  TargetSelector(bool auto_select = true, std::string class_filter = "person");

  int update(const vision_servo_msgs::msg::TargetArray& tracks);
  bool request(int target_id, const std::string& class_name, bool enable, std::string& message);

  int active_id() const { return active_id_; }
  bool enabled() const { return enabled_; }

private:
  int select_best_visible() const;

  bool auto_select_ = true;
  std::string class_filter_;
  bool enabled_ = true;
  int active_id_ = -1;
  vision_servo_msgs::msg::TargetArray latest_tracks_;
};

}  // namespace perception_pkg
