#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

#include "perception_pkg/byte_tracker.hpp"

namespace {

vision_servo_msgs::msg::Target make_detection(
    float center_x, float center_y, float width, float height, float confidence,
    const std::string& class_name = "person")
{
  vision_servo_msgs::msg::Target target;
  target.id = -1;
  target.class_name = class_name;
  target.confidence = confidence;
  target.center = {center_x, center_y};
  target.width = width;
  target.height = height;
  target.bbox = {
    center_x - width * 0.5F, center_y - height * 0.5F,
    center_x + width * 0.5F, center_y + height * 0.5F};
  target.visible = true;
  target.tracking_state =
      vision_servo_msgs::msg::Target::TRACKING_STATE_UNTRACKED;
  return target;
}

vision_servo_msgs::msg::TargetArray make_frame(uint64_t timestamp_ns)
{
  vision_servo_msgs::msg::TargetArray frame;
  frame.header.stamp.sec = static_cast<int32_t>(timestamp_ns / 1000000000ULL);
  frame.header.stamp.nanosec =
      static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
  frame.header.frame_id = "sony_camera_optical_frame";
  frame.tracking_id = -1;
  return frame;
}

perception_pkg::ByteTrackerConfig test_config()
{
  perception_pkg::ByteTrackerConfig config;
  config.track_high_threshold = 0.50F;
  config.track_low_threshold = 0.10F;
  config.new_track_threshold = 0.60F;
  config.first_match_min_iou = 0.20F;
  config.second_match_min_iou = 0.20F;
  config.unconfirmed_match_min_iou = 0.20F;
  config.duplicate_iou_threshold = 0.85F;
  config.lost_timeout_seconds = 1.0;
  config.min_confirm_hits = 1;
  config.fuse_detection_score = true;
  config.publish_tentative_tracks = true;
  return config;
}

TEST(ByteTracker, LowConfidenceDetectionRecoversExistingId)
{
  perception_pkg::ByteTracker tracker(test_config());
  auto first = make_frame(1'000'000'000ULL);
  first.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F, 0.90F));
  tracker.update(first);
  const auto initial = tracker.get_tracks(1'000'000'000ULL, first.header.frame_id);
  ASSERT_EQ(initial.targets.size(), 1U);
  const int id = initial.targets.front().id;

  auto second = make_frame(1'033'000'000ULL);
  second.targets.push_back(make_detection(103.0F, 121.0F, 50.0F, 100.0F, 0.20F));
  tracker.update(second);
  const auto recovered = tracker.get_tracks(1'033'000'000ULL, second.header.frame_id);
  ASSERT_EQ(recovered.targets.size(), 1U);
  EXPECT_EQ(recovered.targets.front().id, id);
  EXPECT_TRUE(recovered.targets.front().visible);
  EXPECT_EQ(
      recovered.targets.front().tracking_state,
      vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED);
}

TEST(ByteTracker, LowConfidenceDetectionCannotCreateTrack)
{
  perception_pkg::ByteTracker tracker(test_config());
  auto frame = make_frame(1'000'000'000ULL);
  frame.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F, 0.20F));
  tracker.update(frame);
  EXPECT_TRUE(
      tracker.get_tracks(1'000'000'000ULL, frame.header.frame_id).targets.empty());
}

TEST(ByteTracker, NewTrackUsesStricterCreationThreshold)
{
  perception_pkg::ByteTracker tracker(test_config());
  auto below_creation = make_frame(1'000'000'000ULL);
  below_creation.targets.push_back(
      make_detection(100.0F, 120.0F, 50.0F, 100.0F, 0.55F));
  tracker.update(below_creation);
  EXPECT_TRUE(tracker.get_tracks(
      1'000'000'000ULL, below_creation.header.frame_id).targets.empty());

  auto above_creation = make_frame(1'033'000'000ULL);
  above_creation.targets.push_back(
      make_detection(100.0F, 120.0F, 50.0F, 100.0F, 0.65F));
  tracker.update(above_creation);
  EXPECT_EQ(tracker.get_tracks(
      1'033'000'000ULL, above_creation.header.frame_id).targets.size(), 1U);
}

TEST(ByteTracker, RequiresConsecutiveHitsToConfirmTentativeTrack)
{
  auto config = test_config();
  config.min_confirm_hits = 3;
  perception_pkg::ByteTracker tracker(config);

  for (int index = 0; index < 3; ++index) {
    const uint64_t timestamp = 1'000'000'000ULL +
        static_cast<uint64_t>(index) * 33'000'000ULL;
    auto frame = make_frame(timestamp);
    frame.targets.push_back(make_detection(
        100.0F + static_cast<float>(index), 120.0F,
        50.0F, 100.0F, 0.90F));
    tracker.update(frame);
    const auto tracks = tracker.get_tracks(timestamp, frame.header.frame_id);
    ASSERT_EQ(tracks.targets.size(), 1U);
    EXPECT_EQ(
        tracks.targets.front().tracking_state,
        index < 2
            ? vision_servo_msgs::msg::Target::TRACKING_STATE_TENTATIVE
            : vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED);
  }
}

TEST(ByteTracker, ReacquiresLostTrackBeforeTimeout)
{
  auto config = test_config();
  config.lost_timeout_seconds = 0.5;
  perception_pkg::ByteTracker tracker(config);
  auto first = make_frame(1'000'000'000ULL);
  first.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F, 0.90F));
  tracker.update(first);
  const int id = tracker.get_tracks(
      1'000'000'000ULL, first.header.frame_id).targets.front().id;

  auto empty = make_frame(1'100'000'000ULL);
  tracker.update(empty);
  const auto lost = tracker.get_tracks(1'100'000'000ULL, empty.header.frame_id);
  ASSERT_EQ(lost.targets.size(), 1U);
  EXPECT_FALSE(lost.targets.front().visible);
  EXPECT_EQ(
      lost.targets.front().tracking_state,
      vision_servo_msgs::msg::Target::TRACKING_STATE_LOST);

  auto recovered = make_frame(1'200'000'000ULL);
  recovered.targets.push_back(
      make_detection(102.0F, 120.0F, 50.0F, 100.0F, 0.85F));
  tracker.update(recovered);
  const auto tracks = tracker.get_tracks(
      1'200'000'000ULL, recovered.header.frame_id);
  ASSERT_EQ(tracks.targets.size(), 1U);
  EXPECT_EQ(tracks.targets.front().id, id);
  EXPECT_TRUE(tracks.targets.front().visible);
}

TEST(ByteTracker, ExpiresLostTrackByElapsedTime)
{
  auto config = test_config();
  config.lost_timeout_seconds = 0.10;
  perception_pkg::ByteTracker tracker(config);
  auto first = make_frame(1'000'000'000ULL);
  first.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F, 0.90F));
  tracker.update(first);

  auto empty = make_frame(1'050'000'000ULL);
  tracker.update(empty);
  EXPECT_EQ(
      tracker.get_tracks(1'050'000'000ULL, empty.header.frame_id).targets.size(), 1U);
  empty.header.stamp.nanosec = 200'000'000U;
  tracker.update(empty);
  EXPECT_TRUE(
      tracker.get_tracks(1'200'000'000ULL, empty.header.frame_id).targets.empty());
}

TEST(ByteTracker, DoesNotAssociateDifferentClasses)
{
  perception_pkg::ByteTracker tracker(test_config());
  auto first = make_frame(1'000'000'000ULL);
  first.targets.push_back(make_detection(
      100.0F, 120.0F, 50.0F, 100.0F, 0.90F, "person"));
  tracker.update(first);
  const int person_id = tracker.get_tracks(
      1'000'000'000ULL, first.header.frame_id).targets.front().id;

  auto second = make_frame(1'033'000'000ULL);
  second.targets.push_back(make_detection(
      100.0F, 120.0F, 50.0F, 100.0F, 0.90F, "dog"));
  tracker.update(second);
  const auto tracks = tracker.get_tracks(1'033'000'000ULL, second.header.frame_id);
  ASSERT_EQ(tracks.targets.size(), 2U);
  EXPECT_NE(tracks.targets.back().id, person_id);
}

TEST(ByteTracker, UsesGlobalAssignmentForCompetingTracks)
{
  perception_pkg::ByteTracker tracker(test_config());
  auto first = make_frame(1'000'000'000ULL);
  first.targets.push_back(make_detection(50.0F, 100.0F, 100.0F, 100.0F, 0.90F));
  first.targets.push_back(make_detection(100.0F, 100.0F, 100.0F, 100.0F, 0.90F));
  tracker.update(first);
  ASSERT_EQ(tracker.get_tracks(
      1'000'000'000ULL, first.header.frame_id).targets.size(), 2U);

  auto second = make_frame(1'033'000'000ULL);
  second.targets.push_back(make_detection(75.0F, 100.0F, 100.0F, 100.0F, 0.90F));
  second.targets.push_back(make_detection(25.0F, 100.0F, 100.0F, 100.0F, 0.90F));
  tracker.update(second);
  const auto tracks = tracker.get_tracks(
      1'033'000'000ULL, second.header.frame_id);
  ASSERT_EQ(tracks.targets.size(), 2U);
  EXPECT_TRUE(tracks.targets[0].visible);
  EXPECT_TRUE(tracks.targets[1].visible);
}

TEST(ByteTracker, RejectsInvalidConfiguration)
{
  auto config = test_config();
  config.track_low_threshold = 0.8F;
  config.track_high_threshold = 0.5F;
  EXPECT_THROW(
      static_cast<void>(perception_pkg::ByteTracker{config}),
      std::invalid_argument);

  config = test_config();
  config.new_track_threshold = 0.4F;
  EXPECT_THROW(
      static_cast<void>(perception_pkg::ByteTracker{config}),
      std::invalid_argument);

  config = test_config();
  config.min_confirm_hits = 0;
  EXPECT_THROW(
      static_cast<void>(perception_pkg::ByteTracker{config}),
      std::invalid_argument);
}

TEST(ByteTracker, TimestampRollbackKeepsFinitePrediction)
{
  perception_pkg::ByteTracker tracker(test_config());
  auto first = make_frame(2'000'000'000ULL);
  first.targets.push_back(make_detection(100.0F, 120.0F, 50.0F, 100.0F, 0.90F));
  tracker.update(first);
  auto rollback = make_frame(1'000'000'000ULL);
  rollback.targets.push_back(make_detection(102.0F, 120.0F, 50.0F, 100.0F, 0.90F));
  tracker.update(rollback);
  const auto tracks = tracker.get_tracks(
      1'000'000'000ULL, rollback.header.frame_id);
  ASSERT_EQ(tracks.targets.size(), 1U);
  EXPECT_TRUE(std::isfinite(tracks.targets.front().center[0]));
  EXPECT_TRUE(std::isfinite(tracks.targets.front().center[1]));
}

}  // namespace
