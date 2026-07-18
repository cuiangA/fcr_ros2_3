#include "servo_control_pkg/target_input.hpp"

#include <gtest/gtest.h>

#include <cstdint>

using vision_servo_msgs::msg::Target;
using vision_servo_msgs::msg::TargetArray;

namespace {

Target make_target(int id, uint8_t state, bool visible)
{
  Target target;
  target.id = id;
  target.tracking_state = state;
  target.visible = visible;
  return target;
}

TEST(TargetInput, SelectsVisibleConfirmedTrackingId)
{
  TargetArray input;
  input.tracking_id = 7;
  input.targets = {
    make_target(3, Target::TRACKING_STATE_CONFIRMED, true),
    make_target(7, Target::TRACKING_STATE_CONFIRMED, true)};
  const auto selected = servo_control_pkg::select_actionable_target(input);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->id, 7);
}

TEST(TargetInput, RejectsLostOrInvisibleLockedTrack)
{
  TargetArray input;
  input.tracking_id = 7;
  input.targets = {
    make_target(7, Target::TRACKING_STATE_LOST, false),
    make_target(8, Target::TRACKING_STATE_CONFIRMED, true)};
  EXPECT_FALSE(servo_control_pkg::select_actionable_target(input).has_value());
}

TEST(TargetInput, AllowsVisibleUntrackedMeasurements)
{
  TargetArray input;
  input.tracking_id = -1;
  input.targets = {make_target(-1, Target::TRACKING_STATE_UNTRACKED, true)};
  EXPECT_TRUE(servo_control_pkg::select_actionable_target(input).has_value());
}

}  // namespace
