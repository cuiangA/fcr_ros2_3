#include "servo_control_pkg/mvp_safety.hpp"

#include <gtest/gtest.h>

#include <limits>

using vision_servo_msgs::msg::Target;

namespace {

Target confirmed_target()
{
  Target target;
  target.visible = true;
  target.tracking_state = Target::TRACKING_STATE_CONFIRMED;
  target.position[2] = 0.0F;
  target.depth_confidence = 0.0F;
  return target;
}

TEST(MvpSafety, Pure2dConfirmedTargetDoesNotRequireDepth)
{
  EXPECT_TRUE(servo_control_pkg::mvp_safety::target_visible_for_control(
    confirmed_target()));
}

TEST(MvpSafety, LostTentativeAndInvisibleTargetsAreRejected)
{
  auto target = confirmed_target();
  target.tracking_state = Target::TRACKING_STATE_LOST;
  EXPECT_FALSE(servo_control_pkg::mvp_safety::target_visible_for_control(target));

  target.tracking_state = Target::TRACKING_STATE_TENTATIVE;
  EXPECT_FALSE(servo_control_pkg::mvp_safety::target_visible_for_control(target));

  target.tracking_state = Target::TRACKING_STATE_CONFIRMED;
  target.visible = false;
  EXPECT_FALSE(servo_control_pkg::mvp_safety::target_visible_for_control(target));
}

TEST(MvpSafety, TranslationRequiresConfidenceAndBoundedMetricDepth)
{
  auto target = confirmed_target();
  target.position[2] = 2.0F;
  target.depth_confidence = 0.8F;
  EXPECT_TRUE(servo_control_pkg::mvp_safety::metric_depth_valid(
    target, 0.6, 0.3, 10.0));

  target.depth_confidence = 0.2F;
  EXPECT_FALSE(servo_control_pkg::mvp_safety::metric_depth_valid(
    target, 0.6, 0.3, 10.0));

  target.depth_confidence = 0.8F;
  target.position[2] = 0.0F;
  EXPECT_FALSE(servo_control_pkg::mvp_safety::metric_depth_valid(
    target, 0.6, 0.3, 10.0));

  target.position[2] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(servo_control_pkg::mvp_safety::metric_depth_valid(
    target, 0.6, 0.3, 10.0));
}

}  // namespace
