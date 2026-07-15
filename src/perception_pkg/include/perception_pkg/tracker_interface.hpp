#pragma once

#include <cstdint>
#include <string>

#include <vision_servo_msgs/msg/target_array.hpp>

namespace perception_pkg {

/**
 * Common startup-selected 2D tracker interface.
 *
 * Keeping the ROS node dependent on this small interface makes it possible to
 * compare ByteTrack with the previous IoU tracker without changing topics,
 * messages, target selection, or downstream control code.
 */
class TrackerInterface {
public:
  virtual ~TrackerInterface() = default;

  virtual void update(
      const vision_servo_msgs::msg::TargetArray& detections) = 0;
  virtual vision_servo_msgs::msg::TargetArray get_tracks(
      uint64_t timestamp, const std::string& frame_id) = 0;
  virtual const char* name() const noexcept = 0;
};

}  // namespace perception_pkg
