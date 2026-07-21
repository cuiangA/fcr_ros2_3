#include <gtest/gtest.h>

#include <string>

#include "perception_pkg/target_selector.hpp"

namespace {

vision_servo_msgs::msg::Target target(
    int id, const std::string& class_name, bool visible, float width, float confidence)
{
  vision_servo_msgs::msg::Target result;
  result.id = id;
  result.class_name = class_name;
  result.visible = visible;
  result.width = width;
  result.height = width;
  result.confidence = confidence;
  result.tracking_state = visible
      ? vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED
      : vision_servo_msgs::msg::Target::TRACKING_STATE_LOST;
  return result;
}

vision_servo_msgs::msg::TargetArray make_tracks(
    const std::vector<vision_servo_msgs::msg::Target>& targets)
{
  vision_servo_msgs::msg::TargetArray result;
  result.targets = targets;
  result.header.stamp.sec = 1000;
  return result;
}

TEST(TargetSelector, RejectsUnknownManualId)
{
  perception_pkg::TargetSelector selector;
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  selector.update(tracks);
  std::string message;
  EXPECT_FALSE(selector.request(99, "person", true, message));
  EXPECT_EQ(selector.active_id(), 7);
}

TEST(TargetSelector, DisableOnlyClearsTargetLock)
{
  perception_pkg::TargetSelector selector;
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  EXPECT_EQ(selector.update(tracks), 7);
  std::string message;
  EXPECT_TRUE(selector.request(-1, "", false, message));
  EXPECT_FALSE(selector.enabled());
  EXPECT_EQ(selector.update(tracks), -1);
}

TEST(TargetSelector, AutoSelectUsesVisibleConfiguredClassOnly)
{
  perception_pkg::TargetSelector selector(true, "person");
  auto tracks = make_tracks({
    target(1, "person", false, 500.0F, 0.99F),
    target(2, "dog", true, 400.0F, 0.99F),
    target(3, "person", true, 100.0F, 0.8F),
  });
  EXPECT_EQ(selector.update(tracks), 3);
}

TEST(TargetSelector, DefaultAllowAutoSwitchIsFalse)
{
  perception_pkg::TargetSelector selector;
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  EXPECT_EQ(selector.update(tracks), 7);
  tracks.targets.front().visible = false;
  tracks.targets.front().tracking_state =
      vision_servo_msgs::msg::Target::TRACKING_STATE_LOST;
  EXPECT_EQ(selector.update(tracks), 7);
  EXPECT_EQ(selector.state(), perception_pkg::TargetSelector::State::LOCKED_PERSON_LOST);
}

TEST(TargetSelector, KeepsLockedIdDuringLostTimeout)
{
  perception_pkg::TargetSelector selector(true, "person", 5.0, false);
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  EXPECT_EQ(selector.update(tracks), 7);
  tracks.targets.front().visible = false;
  tracks.targets.front().tracking_state =
      vision_servo_msgs::msg::Target::TRACKING_STATE_LOST;
  EXPECT_EQ(selector.update(tracks), 7);
  EXPECT_EQ(selector.state(), perception_pkg::TargetSelector::State::LOCKED_PERSON_LOST);
}

TEST(TargetSelector, DoesNotAutoSelectTentativeTrack)
{
  perception_pkg::TargetSelector selector(true, "person");
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  tracks.targets.front().tracking_state =
      vision_servo_msgs::msg::Target::TRACKING_STATE_TENTATIVE;
  EXPECT_EQ(selector.update(tracks), -1);
}

TEST(TargetSelector, ManualModeDoesNotClaimAutoSelectIsEnabled)
{
  perception_pkg::TargetSelector selector(false, "person");
  std::string message;
  EXPECT_TRUE(selector.request(-1, "", true, message));
  EXPECT_EQ(selector.active_id(), -1);
  EXPECT_NE(message.find("manual target_id"), std::string::npos);
}

TEST(TargetSelector, ManualLockPreventsAutoSwitch)
{
  perception_pkg::TargetSelector selector(false, "person", 0.5, true);
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  selector.update(tracks);
  std::string message;
  EXPECT_TRUE(selector.request(7, "person", true, message));
  EXPECT_TRUE(selector.manual_lock());
  tracks.targets.front().visible = false;
  tracks.targets.front().tracking_state =
      vision_servo_msgs::msg::Target::TRACKING_STATE_LOST;
  EXPECT_EQ(selector.update(tracks), 7);
}

TEST(TargetSelector, WaitsForReacquireAfterIdDeleted)
{
  perception_pkg::TargetSelector selector(true, "person", 0.5, false);
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  EXPECT_EQ(selector.update(tracks), 7);
  tracks.targets.clear();
  EXPECT_EQ(selector.update(tracks), -1);
  EXPECT_EQ(selector.state(), perception_pkg::TargetSelector::State::NO_TARGET);
}

TEST(TargetSelector, ManualLockedIdDeletedEntersWaitReacquire)
{
  perception_pkg::TargetSelector selector(true, "person", 0.5, false);
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  selector.update(tracks);
  std::string message;
  selector.request(7, "person", true, message);
  tracks.targets.clear();
  EXPECT_EQ(selector.update(tracks), -1);
  EXPECT_EQ(selector.state(), perception_pkg::TargetSelector::State::WAIT_REACQUIRE);
}

TEST(TargetSelector, AllowAutoSwitchChangesTargetAfterTimeout)
{
  perception_pkg::TargetSelector selector(true, "person", 0.001, true);
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  EXPECT_EQ(selector.update(tracks), 7);
  tracks.targets.front().visible = false;
  tracks.targets.front().tracking_state =
      vision_servo_msgs::msg::Target::TRACKING_STATE_LOST;
  tracks.targets.push_back(target(8, "person", true, 200.0F, 0.9F));
  tracks.header.stamp.sec = 1001;
  int new_id = selector.update(tracks);
  EXPECT_EQ(new_id, 8);
  EXPECT_EQ(selector.state(), perception_pkg::TargetSelector::State::SWITCH_ALLOWED);
}

TEST(TargetSelector, StateNameReturnsCorrectString)
{
  perception_pkg::TargetSelector selector;
  EXPECT_EQ(selector.state_name(), "NO_TARGET");
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  selector.update(tracks);
  EXPECT_EQ(selector.state_name(), "LOCKED_VISIBLE");
}

TEST(TargetSelector, SwitchDelayExtendsHoldTime)
{
  perception_pkg::TargetSelector selector(true, "person", 0.001, true, 5.0);
  auto tracks = make_tracks({target(7, "person", true, 100.0F, 0.9F)});
  EXPECT_EQ(selector.update(tracks), 7);
  tracks.targets.front().visible = false;
  tracks.targets.front().tracking_state =
      vision_servo_msgs::msg::Target::TRACKING_STATE_LOST;
  tracks.header.stamp.sec = 1001;
  EXPECT_EQ(selector.update(tracks), 7);
  EXPECT_EQ(selector.state(), perception_pkg::TargetSelector::State::WAIT_REACQUIRE);
}

}  // namespace
