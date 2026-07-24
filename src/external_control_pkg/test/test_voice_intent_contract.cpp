#include "external_control_pkg/voice_intent_contract.hpp"

#include <gtest/gtest.h>

namespace external_control_pkg {

TEST(VoiceIntentContract, SeparatesGimbalAndChassisDirections)
{
  EXPECT_EQ(
    intentTarget("gimbal_nudge_right"), VoiceTarget::Gimbal);
  EXPECT_EQ(
    intentTarget("chassis_move_right"), VoiceTarget::Chassis);
  EXPECT_NE(
    intentTarget("chassis_move_right"), VoiceTarget::Gimbal);
}

TEST(VoiceIntentContract, RoutesCameraAndAutonomyIntents)
{
  EXPECT_EQ(
    intentTarget("camera_take_photo"), VoiceTarget::Camera);
  EXPECT_EQ(
    intentTarget("camera_start_recording"), VoiceTarget::Camera);
  EXPECT_EQ(
    intentTarget("start_following"), VoiceTarget::Autonomy);
  EXPECT_EQ(
    intentTarget("switch_target"), VoiceTarget::Autonomy);
}

TEST(VoiceIntentContract, RejectsUnknownAndAmbiguousStops)
{
  EXPECT_EQ(intentTarget("not_a_real_intent"), VoiceTarget::Unknown);
  EXPECT_TRUE(isAmbiguousStopIntent("stop"));
  EXPECT_TRUE(isAmbiguousStopIntent("stop_current_action"));
  EXPECT_FALSE(isAmbiguousStopIntent("gimbal_stop"));
  EXPECT_FALSE(isAmbiguousStopIntent("camera_stop_recording"));
}

}  // namespace external_control_pkg
