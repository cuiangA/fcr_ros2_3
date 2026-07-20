#include "servo_control_pkg/mvp_yaw_control.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace yaw_control = servo_control_pkg::mvp_yaw_control;

TEST(MvpYawControl, StartupReferenceRemovesAbsoluteEncoderOffset)
{
  EXPECT_NEAR(yaw_control::wrapped_angle_error(3.077, 3.077), 0.0, 1e-9);
}

TEST(MvpYawControl, DifferenceIsContinuousAcrossPiWrap)
{
  const double reference = M_PI - 0.02;
  const double current = -M_PI + 0.03;
  EXPECT_NEAR(yaw_control::wrapped_angle_error(current, reference), 0.05, 1e-9);
}

TEST(MvpYawControl, RightGimbalOffsetCommandsRightBaseTurn)
{
  EXPECT_NEAR(yaw_control::base_yaw_command(0.5, 0.2, -1.0, 0.12, 0.12), -0.1, 1e-9);
}

TEST(MvpYawControl, DeadbandAndRateLimitAreApplied)
{
  EXPECT_DOUBLE_EQ(yaw_control::base_yaw_command(0.1, 0.2, -1.0, 0.12, 0.12), 0.0);
  EXPECT_NEAR(yaw_control::base_yaw_command(-2.0, 0.2, -1.0, 0.12, 0.12), 0.12, 1e-9);
}
