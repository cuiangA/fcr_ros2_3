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

TEST(TargetSelector, RejectsUnknownManualId)
{
  perception_pkg::TargetSelector selector;
  vision_servo_msgs::msg::TargetArray tracks;
  tracks.targets.push_back(target(7, "person", true, 100.0F, 0.9F));
  selector.update(tracks);
  std::string message;
  EXPECT_FALSE(selector.request(99, "person", true, message));
  EXPECT_EQ(selector.active_id(), 7);
}

TEST(TargetSelector, DisableOnlyClearsTargetLock)
{
  perception_pkg::TargetSelector selector;
  vision_servo_msgs::msg::TargetArray tracks;
  tracks.targets.push_back(target(7, "person", true, 100.0F, 0.9F));
  EXPECT_EQ(selector.update(tracks), 7);
  std::string message;
  EXPECT_TRUE(selector.request(-1, "", false, message));
  EXPECT_FALSE(selector.enabled());
  EXPECT_EQ(selector.update(tracks), -1);
}

TEST(TargetSelector, AutoSelectUsesVisibleConfiguredClassOnly)
{
  perception_pkg::TargetSelector selector(true, "person");
  vision_servo_msgs::msg::TargetArray tracks;
  tracks.targets.push_back(target(1, "person", false, 500.0F, 0.99F));
  tracks.targets.push_back(target(2, "dog", true, 400.0F, 0.99F));
  tracks.targets.push_back(target(3, "person", true, 100.0F, 0.8F));
  EXPECT_EQ(selector.update(tracks), 3);
}

TEST(TargetSelector, KeepsLockedIdDuringShortLostWindow)
{
  perception_pkg::TargetSelector selector;
  vision_servo_msgs::msg::TargetArray tracks;
  tracks.targets.push_back(target(7, "person", true, 100.0F, 0.9F));
  EXPECT_EQ(selector.update(tracks), 7);
  tracks.targets.front().visible = false;
  tracks.targets.front().tracking_state =
      vision_servo_msgs::msg::Target::TRACKING_STATE_LOST;
  EXPECT_EQ(selector.update(tracks), 7);
}

TEST(TargetSelector, ManualModeDoesNotClaimAutoSelectIsEnabled)
{
  perception_pkg::TargetSelector selector(false, "person");
  std::string message;
  EXPECT_TRUE(selector.request(-1, "", true, message));
  EXPECT_EQ(selector.active_id(), -1);
  EXPECT_NE(message.find("manual target_id"), std::string::npos);
  EXPECT_EQ(message.find("Auto-select enabled"), std::string::npos);
}

}  // namespace
