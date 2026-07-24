#pragma once

#include <string>
#include <unordered_map>

namespace external_control_pkg {

enum class VoiceTarget {
  Unknown,
  System,
  Gimbal,
  Chassis,
  Camera,
  Autonomy,
};

inline const char* targetName(VoiceTarget target)
{
  switch (target) {
    case VoiceTarget::System:
      return "system";
    case VoiceTarget::Gimbal:
      return "gimbal";
    case VoiceTarget::Chassis:
      return "chassis";
    case VoiceTarget::Camera:
      return "camera";
    case VoiceTarget::Autonomy:
      return "autonomy";
    default:
      return "unknown";
  }
}

inline VoiceTarget intentTarget(const std::string& intent)
{
  static const std::unordered_map<std::string, VoiceTarget> targets = {
    {"emergency_stop", VoiceTarget::System},
    {"stop_all", VoiceTarget::System},
    {"status_query", VoiceTarget::System},

    {"gimbal_nudge_left", VoiceTarget::Gimbal},
    {"gimbal_nudge_right", VoiceTarget::Gimbal},
    {"gimbal_nudge_up", VoiceTarget::Gimbal},
    {"gimbal_nudge_down", VoiceTarget::Gimbal},
    {"gimbal_stop", VoiceTarget::Gimbal},
    {"gimbal_home", VoiceTarget::Gimbal},
    {"gimbal_speed_up", VoiceTarget::Gimbal},
    {"gimbal_speed_down", VoiceTarget::Gimbal},
    {"gimbal_adjust_parameter", VoiceTarget::Gimbal},
    {"query_gimbal_status", VoiceTarget::Gimbal},

    {"chassis_move_forward", VoiceTarget::Chassis},
    {"chassis_move_backward", VoiceTarget::Chassis},
    {"chassis_move_left", VoiceTarget::Chassis},
    {"chassis_move_right", VoiceTarget::Chassis},
    {"chassis_turn_left", VoiceTarget::Chassis},
    {"chassis_turn_right", VoiceTarget::Chassis},
    {"chassis_stop", VoiceTarget::Chassis},
    {"chassis_speed_up", VoiceTarget::Chassis},
    {"chassis_speed_down", VoiceTarget::Chassis},
    {"chassis_adjust_parameter", VoiceTarget::Chassis},
    {"distance_adjust", VoiceTarget::Chassis},
    {"query_chassis_status", VoiceTarget::Chassis},

    {"camera_take_photo", VoiceTarget::Camera},
    {"camera_start_recording", VoiceTarget::Camera},
    {"camera_stop_recording", VoiceTarget::Camera},
    {"query_camera_status", VoiceTarget::Camera},

    {"start_following", VoiceTarget::Autonomy},
    {"stop_following", VoiceTarget::Autonomy},
    {"pause_following", VoiceTarget::Autonomy},
    {"resume_following", VoiceTarget::Autonomy},
    {"switch_target", VoiceTarget::Autonomy},
    {"start_orbit", VoiceTarget::Autonomy},
    {"start_dolly", VoiceTarget::Autonomy},
    {"stop_cinematic", VoiceTarget::Autonomy},
    {"query_camera_motion_status", VoiceTarget::Autonomy},

    // Accepted aliases used by the existing gimbal bridge.
    {"home_gimbal", VoiceTarget::Gimbal},
    {"stop_gimbal", VoiceTarget::Gimbal},
    {"gimbal_right", VoiceTarget::Gimbal},
    {"gimbal_left", VoiceTarget::Gimbal},
    {"gimbal_up", VoiceTarget::Gimbal},
    {"gimbal_down", VoiceTarget::Gimbal},
    {"turn_gimbal_right", VoiceTarget::Gimbal},
    {"turn_gimbal_left", VoiceTarget::Gimbal},
    {"tilt_gimbal_up", VoiceTarget::Gimbal},
    {"tilt_gimbal_down", VoiceTarget::Gimbal},
    {"speed_up", VoiceTarget::Gimbal},
    {"speed_down", VoiceTarget::Gimbal},
  };

  const auto found = targets.find(intent);
  return found == targets.end() ? VoiceTarget::Unknown : found->second;
}

inline bool isAmbiguousStopIntent(const std::string& intent)
{
  return intent == "stop" || intent == "stop_current_action";
}

}  // namespace external_control_pkg
