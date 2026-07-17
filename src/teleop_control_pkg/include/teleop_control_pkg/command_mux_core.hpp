#pragma once

#include <cstdint>
#include <string>

namespace teleop_control_pkg
{

enum class ControlMode { kStop, kManual, kAuto };
enum class CommandSource { kStop, kManual, kAuto };

struct VelocityCommand
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct CommandMuxConfig
{
  int64_t heartbeat_timeout_ms{250};
  int64_t command_timeout_ms{200};
  int64_t zero_dwell_ms{200};
  double max_linear_x{0.05};
  double max_linear_y{0.05};
  double max_angular_z{0.25};
  double max_accel_x{0.15};
  double max_accel_y{0.15};
  double max_accel_yaw{0.50};
};

struct MuxDecision
{
  VelocityCommand velocity;
  CommandSource source{CommandSource::kStop};
  std::string reason{"safe_stop"};
  bool estop_latched{false};
};

class CommandMuxCore
{
public:
  explicit CommandMuxCore(CommandMuxConfig config);

  void set_mode(ControlMode mode, int64_t now_ms);
  void receive_heartbeat(int64_t now_ms);
  void receive_deadman(bool active, int64_t now_ms);
  void receive_manual_command(const VelocityCommand & command, int64_t now_ms);
  void receive_auto_command(const VelocityCommand & command, int64_t now_ms);
  void latch_estop();
  bool clear_estop(int64_t now_ms);
  MuxDecision step(int64_t now_ms, double dt_sec);

  ControlMode mode() const {return mode_;}
  bool estop_latched() const {return estop_latched_;}
  bool deadman_active() const {return deadman_active_;}
  int64_t heartbeat_age_ms(int64_t now_ms) const;
  int64_t manual_command_age_ms(int64_t now_ms) const;
  int64_t auto_command_age_ms(int64_t now_ms) const;

private:
  static double clamp(double value, double lower, double upper);
  static double approach(double current, double target, double max_delta);
  VelocityCommand limited(const VelocityCommand & command) const;
  bool fresh(int64_t timestamp_ms, int64_t now_ms, int64_t timeout_ms) const;
  static int64_t age_ms(int64_t timestamp_ms, int64_t now_ms);
  bool is_zero(const VelocityCommand & command) const;

  CommandMuxConfig config_;
  ControlMode mode_{ControlMode::kManual};
  CommandSource active_source_{CommandSource::kStop};
  CommandSource pending_source_{CommandSource::kStop};
  int64_t dwell_until_ms_{0};
  int64_t heartbeat_time_ms_{-1};
  int64_t deadman_time_ms_{-1};
  int64_t manual_command_time_ms_{-1};
  int64_t auto_command_time_ms_{-1};
  bool deadman_active_{false};
  bool estop_latched_{false};
  VelocityCommand manual_command_;
  VelocityCommand auto_command_;
  VelocityCommand output_;
};

const char * to_string(ControlMode mode);
const char * to_string(CommandSource source);

}  // namespace teleop_control_pkg
