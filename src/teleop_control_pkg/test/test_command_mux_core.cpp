#include "teleop_control_pkg/command_mux_core.hpp"

#include <gtest/gtest.h>

using teleop_control_pkg::CommandMuxConfig;
using teleop_control_pkg::CommandMuxCore;
using teleop_control_pkg::CommandSource;
using teleop_control_pkg::ControlMode;
using teleop_control_pkg::VelocityCommand;

TEST(CommandMuxCore, ManualRequiresHeartbeatDeadmanAndFreshCommand)
{
  CommandMuxConfig config;
  config.zero_dwell_ms = 0;
  CommandMuxCore core(config);
  core.receive_manual_command({0.03, 0.0, 0.0}, 0);
  EXPECT_EQ(core.step(0, 0.05).source, CommandSource::kStop);
  core.receive_heartbeat(10);
  core.receive_deadman(true, 10);
  core.receive_manual_command({0.03, 0.0, 0.0}, 10);
  EXPECT_EQ(core.step(10, 0.05).source, CommandSource::kManual);
  EXPECT_EQ(core.step(220, 0.05).source, CommandSource::kStop);
}

TEST(CommandMuxCore, EstopLatchesAndNeedsSafeClear)
{
  CommandMuxConfig config;
  config.zero_dwell_ms = 0;
  CommandMuxCore core(config);
  core.receive_heartbeat(0);
  core.receive_deadman(true, 0);
  core.receive_manual_command({0.03, 0.0, 0.0}, 0);
  core.step(0, 0.05);
  core.latch_estop();
  EXPECT_TRUE(core.step(1, 0.05).estop_latched);
  EXPECT_FALSE(core.clear_estop(1));
  core.receive_deadman(false, 2);
  EXPECT_TRUE(core.clear_estop(2));
  EXPECT_FALSE(core.step(2, 0.05).estop_latched);
}

TEST(CommandMuxCore, ModeSwitchEnforcesZeroDwell)
{
  CommandMuxConfig config;
  config.zero_dwell_ms = 200;
  CommandMuxCore core(config);
  core.receive_auto_command({0.03, 0.0, 0.0}, 0);
  core.set_mode(ControlMode::kAuto, 0);
  EXPECT_EQ(core.step(100, 0.05).source, CommandSource::kStop);
  core.receive_auto_command({0.03, 0.0, 0.0}, 200);
  EXPECT_EQ(core.step(200, 0.05).source, CommandSource::kAuto);
}

TEST(CommandMuxCore, ManualDeadmanOverridesAutoMode)
{
  CommandMuxConfig config;
  config.zero_dwell_ms = 0;
  CommandMuxCore core(config);
  core.set_mode(ControlMode::kAuto, 0);
  core.receive_auto_command({0.02, 0.0, 0.0}, 0);
  EXPECT_EQ(core.step(0, 0.05).source, CommandSource::kAuto);
  core.receive_heartbeat(1);
  core.receive_deadman(true, 1);
  core.receive_manual_command({-0.02, 0.0, 0.0}, 1);
  EXPECT_EQ(core.step(1, 0.05).source, CommandSource::kManual);
}

TEST(CommandMuxCore, LimitsVelocityAndAcceleration)
{
  CommandMuxConfig config;
  config.zero_dwell_ms = 0;
  CommandMuxCore core(config);
  core.receive_heartbeat(0);
  core.receive_deadman(true, 0);
  core.receive_manual_command({5.0, -5.0, 5.0}, 0);
  const auto decision = core.step(0, 0.1);
  EXPECT_NEAR(decision.velocity.x, 0.015, 1e-9);
  EXPECT_NEAR(decision.velocity.y, -0.015, 1e-9);
  EXPECT_NEAR(decision.velocity.yaw, 0.05, 1e-9);
}
