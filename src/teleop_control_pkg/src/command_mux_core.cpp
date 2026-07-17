#include "teleop_control_pkg/command_mux_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace teleop_control_pkg
{

CommandMuxCore::CommandMuxCore(CommandMuxConfig config) : config_(config)
{
  if (config_.heartbeat_timeout_ms <= 0 || config_.command_timeout_ms <= 0 ||
    config_.zero_dwell_ms < 0 || config_.max_linear_x <= 0.0 ||
    config_.max_linear_y <= 0.0 || config_.max_angular_z <= 0.0 ||
    config_.max_accel_x <= 0.0 || config_.max_accel_y <= 0.0 ||
    config_.max_accel_yaw <= 0.0)
  {
    throw std::invalid_argument("command mux limits and timeouts must be positive");
  }
}

void CommandMuxCore::set_mode(ControlMode mode, int64_t now_ms)
{
  if (mode_ == mode) {
    return;
  }
  mode_ = mode;
  active_source_ = CommandSource::kStop;
  pending_source_ = CommandSource::kStop;
  dwell_until_ms_ = now_ms + config_.zero_dwell_ms;
  output_ = {};
}

void CommandMuxCore::receive_heartbeat(int64_t now_ms) {heartbeat_time_ms_ = now_ms;}

void CommandMuxCore::receive_deadman(bool active, int64_t now_ms)
{
  deadman_active_ = active;
  deadman_time_ms_ = now_ms;
}

void CommandMuxCore::receive_manual_command(
  const VelocityCommand & command, int64_t now_ms)
{
  manual_command_ = limited(command);
  manual_command_time_ms_ = now_ms;
}

void CommandMuxCore::receive_auto_command(
  const VelocityCommand & command, int64_t now_ms)
{
  auto_command_ = limited(command);
  auto_command_time_ms_ = now_ms;
}

void CommandMuxCore::latch_estop()
{
  estop_latched_ = true;
  active_source_ = CommandSource::kStop;
  pending_source_ = CommandSource::kStop;
  output_ = {};
}

bool CommandMuxCore::clear_estop(int64_t now_ms)
{
  const bool heartbeat_ok = fresh(
    heartbeat_time_ms_, now_ms, config_.heartbeat_timeout_ms);
  const bool deadman_released = !deadman_active_ || !fresh(
    deadman_time_ms_, now_ms, config_.heartbeat_timeout_ms);
  if (!heartbeat_ok || !deadman_released || !is_zero(output_)) {
    return false;
  }
  estop_latched_ = false;
  dwell_until_ms_ = now_ms + config_.zero_dwell_ms;
  return true;
}

MuxDecision CommandMuxCore::step(int64_t now_ms, double dt_sec)
{
  MuxDecision decision;
  decision.estop_latched = estop_latched_;
  if (estop_latched_) {
    output_ = {};
    decision.reason = "estop_latched";
    return decision;
  }

  CommandSource desired = CommandSource::kStop;
  VelocityCommand target;
  std::string reason = "safe_stop";
  const bool heartbeat_ok = fresh(
    heartbeat_time_ms_, now_ms, config_.heartbeat_timeout_ms);
  const bool deadman_ok = deadman_active_ && fresh(
    deadman_time_ms_, now_ms, config_.heartbeat_timeout_ms);
  const bool manual_command_ok = fresh(
    manual_command_time_ms_, now_ms, config_.command_timeout_ms);
  const bool manual_ok = heartbeat_ok && deadman_ok && manual_command_ok;
  if (mode_ == ControlMode::kManual) {
    if (manual_ok) {
      desired = CommandSource::kManual;
      target = manual_command_;
      reason = "manual_active";
    } else if (!heartbeat_ok) {
      reason = "manual_heartbeat_timeout";
    } else if (!deadman_ok) {
      reason = "manual_deadman_released";
    } else {
      reason = "manual_command_timeout";
    }
  } else if (mode_ == ControlMode::kAuto) {
    if (manual_ok) {
      desired = CommandSource::kManual;
      target = manual_command_;
      reason = "manual_override";
    } else if (fresh(auto_command_time_ms_, now_ms, config_.command_timeout_ms)) {
      desired = CommandSource::kAuto;
      target = auto_command_;
      reason = "auto_active";
    } else {
      reason = "auto_command_timeout";
    }
  } else {
    reason = "mode_stop";
  }

  if (desired != pending_source_) {
    pending_source_ = desired;
    active_source_ = CommandSource::kStop;
    output_ = {};
    if (desired == CommandSource::kStop) {
      dwell_until_ms_ = now_ms;
    } else if (now_ms >= dwell_until_ms_) {
      // Starting or resuming the same source is already coming from zero and
      // is protected by the slew limiter. Only a real manual<->auto handover
      // needs the full zero dwell. This avoids adding 200 ms to every keyboard
      // lease renewal.
      const bool changing_live_source =
        last_non_stop_source_ != CommandSource::kStop &&
        last_non_stop_source_ != desired;
      dwell_until_ms_ = now_ms +
        (changing_live_source ? config_.zero_dwell_ms : 0);
    }
  }
  if (now_ms < dwell_until_ms_) {
    decision.reason = "source_switch_zero_dwell";
    return decision;
  }
  active_source_ = pending_source_;
  if (active_source_ == CommandSource::kStop) {
    output_ = {};
    decision.reason = reason;
    return decision;
  }
  last_non_stop_source_ = active_source_;

  const double dt = std::max(0.0, std::min(dt_sec, 0.25));
  output_.x = approach(output_.x, target.x, config_.max_accel_x * dt);
  output_.y = approach(output_.y, target.y, config_.max_accel_y * dt);
  output_.yaw = approach(output_.yaw, target.yaw, config_.max_accel_yaw * dt);
  decision.velocity = output_;
  decision.source = active_source_;
  decision.reason = reason;
  return decision;
}

double CommandMuxCore::clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

double CommandMuxCore::approach(double current, double target, double max_delta)
{
  return current + clamp(target - current, -max_delta, max_delta);
}

VelocityCommand CommandMuxCore::limited(const VelocityCommand & command) const
{
  return {
    clamp(command.x, -config_.max_linear_x, config_.max_linear_x),
    clamp(command.y, -config_.max_linear_y, config_.max_linear_y),
    clamp(command.yaw, -config_.max_angular_z, config_.max_angular_z)};
}

bool CommandMuxCore::fresh(
  int64_t timestamp_ms, int64_t now_ms, int64_t timeout_ms) const
{
  return timestamp_ms >= 0 && now_ms >= timestamp_ms &&
         now_ms - timestamp_ms <= timeout_ms;
}

int64_t CommandMuxCore::age_ms(int64_t timestamp_ms, int64_t now_ms)
{
  if (timestamp_ms < 0 || now_ms < timestamp_ms) {return -1;}
  return now_ms - timestamp_ms;
}

int64_t CommandMuxCore::heartbeat_age_ms(int64_t now_ms) const
{
  return age_ms(heartbeat_time_ms_, now_ms);
}

int64_t CommandMuxCore::manual_command_age_ms(int64_t now_ms) const
{
  return age_ms(manual_command_time_ms_, now_ms);
}

int64_t CommandMuxCore::auto_command_age_ms(int64_t now_ms) const
{
  return age_ms(auto_command_time_ms_, now_ms);
}

bool CommandMuxCore::is_zero(const VelocityCommand & command) const
{
  constexpr double epsilon = 1e-9;
  return std::abs(command.x) < epsilon && std::abs(command.y) < epsilon &&
         std::abs(command.yaw) < epsilon;
}

const char * to_string(ControlMode mode)
{
  switch (mode) {
    case ControlMode::kManual: return "manual";
    case ControlMode::kAuto: return "auto";
    default: return "stop";
  }
}

const char * to_string(CommandSource source)
{
  switch (source) {
    case CommandSource::kManual: return "manual";
    case CommandSource::kAuto: return "auto";
    default: return "stop";
  }
}

}  // namespace teleop_control_pkg
