#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/video/tracking.hpp>

#include "perception_pkg/tracker_interface.hpp"

namespace perception_pkg {

struct ByteTrackerConfig {
  float track_high_threshold = 0.50F;
  float track_low_threshold = 0.10F;
  float new_track_threshold = 0.60F;
  float first_match_min_iou = 0.30F;
  float second_match_min_iou = 0.50F;
  float unconfirmed_match_min_iou = 0.30F;
  float duplicate_iou_threshold = 0.85F;
  double lost_timeout_seconds = 2.5;
  int min_confirm_hits = 3;
  bool fuse_detection_score = true;
  bool publish_tentative_tracks = true;
};

/**
 * Lightweight native C++ ByteTrack implementation for TargetArray messages.
 *
 * High-confidence detections perform the primary association. Remaining active
 * tracks get a second chance against low-confidence detections, while low-score
 * detections are never allowed to create a new identity.
 */
class ByteTracker final : public TrackerInterface {
public:
  explicit ByteTracker(ByteTrackerConfig config = {});

  void update(const vision_servo_msgs::msg::TargetArray& detections) override;
  vision_servo_msgs::msg::TargetArray get_tracks(
      uint64_t timestamp, const std::string& frame_id) override;
  const char* name() const noexcept override { return "bytetrack"; }

private:
  enum class TrackState { kTentative, kConfirmed, kLost, kRemoved };

  struct Track {
    int id = -1;
    std::string class_name;
    cv::KalmanFilter kf;
    TrackState state = TrackState::kTentative;
    int consecutive_hits = 0;
    int total_hits = 0;
    float confidence = 0.0F;
    bool visible_in_current_frame = false;
    double created_time_seconds = 0.0;
    double last_seen_time_seconds = 0.0;
  };

  struct AssociationResult {
    std::vector<std::pair<int, int>> matches;
    std::set<int> unmatched_tracks;
    std::set<int> unmatched_detections;
  };

  void validate_config() const;
  double advance_timeline(const vision_servo_msgs::msg::TargetArray& detections);
  void predict(double dt_seconds);
  AssociationResult associate(
      const std::vector<int>& track_ids,
      const std::vector<int>& detection_indices,
      const vision_servo_msgs::msg::TargetArray& detections,
      float minimum_iou,
      bool fuse_detection_score) const;
  void update_track(Track& track, const vision_servo_msgs::msg::Target& detection);
  void create_track(const vision_servo_msgs::msg::Target& detection);
  void remove_duplicate_tracks();
  int allocate_track_id();

  static void initialize_filter(
      Track& track, const vision_servo_msgs::msg::Target& detection);
  static float compute_iou(
      const vision_servo_msgs::msg::Target& left,
      const vision_servo_msgs::msg::Target& right);
  static vision_servo_msgs::msg::Target target_from_track(const Track& track);

  ByteTrackerConfig config_;
  std::map<int, Track> tracks_;
  int next_id_ = 0;
  uint64_t last_input_timestamp_ns_ = 0;
  double timeline_seconds_ = 0.0;
};

}  // namespace perception_pkg
