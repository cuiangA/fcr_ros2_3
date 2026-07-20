#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "perception_pkg/tracking_node.hpp"

namespace {

vision_servo_msgs::msg::Target make_detection(
    float center_x, float center_y, float width, float height,
    const std::string& class_name = "person")
{
  vision_servo_msgs::msg::Target target;
  target.id = -1;
  target.class_name = class_name;
  target.confidence = 0.9F;
  target.center = {center_x, center_y};
  target.width = width;
  target.height = height;
  target.bbox = {
    center_x - width * 0.5F, center_y - height * 0.5F,
    center_x + width * 0.5F, center_y + height * 0.5F};
  return target;
}

vision_servo_msgs::msg::TargetArray make_frame(uint32_t nanosec)
{
  vision_servo_msgs::msg::TargetArray frame;
  frame.header.stamp.nanosec = nanosec;
  frame.header.frame_id = "sony_camera_optical_frame";
  return frame;
}

TEST(MultiObjectTracker, KeepsIdAcrossNearbyDetections)
{
  perception_pkg::MultiObjectTracker tracker(3, 1, 0.2F);
  auto first = make_frame(10'000'000U);
  first.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F));
  tracker.update(first);
  const auto first_tracks = tracker.get_tracks(10'000'000ULL, first.header.frame_id);
  ASSERT_EQ(first_tracks.targets.size(), 1U);
  const int first_id = first_tracks.targets.front().id;

  auto second = make_frame(43'000'000U);
  second.targets.push_back(make_detection(104.0F, 121.0F, 50.0F, 100.0F));
  tracker.update(second);
  const auto second_tracks = tracker.get_tracks(43'000'000ULL, second.header.frame_id);
  ASSERT_EQ(second_tracks.targets.size(), 1U);
  EXPECT_EQ(second_tracks.targets.front().id, first_id);
  EXPECT_EQ(second_tracks.header.frame_id, "sony_camera_optical_frame");
  EXPECT_TRUE(second_tracks.targets.front().visible);
  EXPECT_EQ(
      second_tracks.targets.front().tracking_state,
      vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED);
}

TEST(MultiObjectTracker, RequiresConsecutiveHitsToConfirm)
{
  perception_pkg::MultiObjectTracker tracker(3, 3, 0.2F);
  auto frame = make_frame(10'000'000U);
  frame.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F));
  tracker.update(frame);
  EXPECT_TRUE(tracker.get_tracks(10'000'000ULL, frame.header.frame_id).targets.empty());

  frame.targets.clear();
  frame.header.stamp.nanosec = 43'000'000U;
  tracker.update(frame);

  frame.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F));
  frame.header.stamp.nanosec = 76'000'000U;
  tracker.update(frame);
  EXPECT_TRUE(tracker.get_tracks(76'000'000ULL, frame.header.frame_id).targets.empty());

  frame.header.stamp.nanosec = 109'000'000U;
  tracker.update(frame);
  EXPECT_TRUE(tracker.get_tracks(109'000'000ULL, frame.header.frame_id).targets.empty());

  frame.header.stamp.nanosec = 142'000'000U;
  tracker.update(frame);
  ASSERT_EQ(tracker.get_tracks(142'000'000ULL, frame.header.frame_id).targets.size(), 1U);
}

TEST(MultiObjectTracker, DoesNotAssociateAcrossClasses)
{
  perception_pkg::MultiObjectTracker tracker(3, 1, 0.2F);
  auto first = make_frame(10'000'000U);
  first.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F, "person"));
  tracker.update(first);
  const int person_id = tracker.get_tracks(10'000'000ULL, first.header.frame_id).targets.front().id;

  auto second = make_frame(43'000'000U);
  second.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F, "dog"));
  tracker.update(second);
  const auto tracks = tracker.get_tracks(43'000'000ULL, second.header.frame_id);
  ASSERT_EQ(tracks.targets.size(), 2U);
  const auto dog = std::find_if(
      tracks.targets.begin(), tracks.targets.end(),
      [](const auto& target) { return target.class_name == "dog"; });
  ASSERT_NE(dog, tracks.targets.end());
  EXPECT_NE(dog->id, person_id);
}

TEST(MultiObjectTracker, PublishesLostStateUntilTrackExpires)
{
  perception_pkg::MultiObjectTracker tracker(2, 1, 0.2F);
  auto detected = make_frame(10'000'000U);
  detected.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F));
  tracker.update(detected);

  auto empty = make_frame(43'000'000U);
  tracker.update(empty);
  const auto lost = tracker.get_tracks(43'000'000ULL, empty.header.frame_id);
  ASSERT_EQ(lost.targets.size(), 1U);
  EXPECT_FALSE(lost.targets.front().visible);
  EXPECT_EQ(
      lost.targets.front().tracking_state,
      vision_servo_msgs::msg::Target::TRACKING_STATE_LOST);
}

TEST(MultiObjectTracker, UsesGlobalAssignmentForCompetingTracks)
{
  perception_pkg::MultiObjectTracker tracker(2, 1, 0.2F);
  auto first = make_frame(10'000'000U);
  first.targets.push_back(make_detection(50.0F, 100.0F, 100.0F, 100.0F));
  first.targets.push_back(make_detection(100.0F, 100.0F, 100.0F, 100.0F));
  tracker.update(first);
  ASSERT_EQ(tracker.get_tracks(10'000'000ULL, first.header.frame_id).targets.size(), 2U);

  auto second = make_frame(43'000'000U);
  // Detection 0 can match both tracks; detection 1 can only match track 0.
  // A per-track greedy pass consumes detection 0 too early, while global
  // assignment keeps both existing tracks visible.
  second.targets.push_back(make_detection(75.0F, 100.0F, 100.0F, 100.0F));
  second.targets.push_back(make_detection(25.0F, 100.0F, 100.0F, 100.0F));
  tracker.update(second);
  const auto tracks = tracker.get_tracks(43'000'000ULL, second.header.frame_id);
  ASSERT_EQ(tracks.targets.size(), 2U);
  EXPECT_TRUE(std::all_of(
      tracks.targets.begin(), tracks.targets.end(),
      [](const auto& target) { return target.visible; }));
}

TEST(MultiObjectTracker, HandlesTimestampRollbackWithoutInvalidState)
{
  perception_pkg::MultiObjectTracker tracker(2, 1, 0.2F);
  auto first = make_frame(100'000'000U);
  first.targets.push_back(make_detection(50.0F, 100.0F, 100.0F, 100.0F));
  tracker.update(first);

  auto rollback = make_frame(50'000'000U);
  rollback.targets.push_back(make_detection(52.0F, 100.0F, 100.0F, 100.0F));
  tracker.update(rollback);
  const auto tracks = tracker.get_tracks(50'000'000ULL, rollback.header.frame_id);
  ASSERT_EQ(tracks.targets.size(), 1U);
  EXPECT_TRUE(std::isfinite(tracks.targets.front().center[0]));
  EXPECT_TRUE(std::isfinite(tracks.targets.front().center[1]));
}

TEST(MultiObjectTracker, RejectsInvalidConfiguration)
{
  EXPECT_THROW(perception_pkg::MultiObjectTracker(-1, 1, 0.2F), std::invalid_argument);
  EXPECT_THROW(perception_pkg::MultiObjectTracker(1, 0, 0.2F), std::invalid_argument);
  EXPECT_THROW(perception_pkg::MultiObjectTracker(1, 1, 1.1F), std::invalid_argument);
}

TEST(MultiObjectTracker, RemovesExpiredTrack)
{
  perception_pkg::MultiObjectTracker tracker(1, 1, 0.2F);
  auto detected = make_frame(10'000'000U);
  detected.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F));
  tracker.update(detected);

  auto empty = make_frame(43'000'000U);
  tracker.update(empty);
  empty.header.stamp.nanosec = 76'000'000U;
  tracker.update(empty);
  const auto tracks = tracker.get_tracks(76'000'000ULL, empty.header.frame_id);
  EXPECT_TRUE(tracks.targets.empty());
}

}  // namespace
