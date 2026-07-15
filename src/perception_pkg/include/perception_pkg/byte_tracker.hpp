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
  float new_track_threshold = 0.65F;
  float confirmed_match_min_iou = 0.25F;
  float lost_match_min_iou = 0.10F;
  float second_match_min_iou = 0.20F;
  float unconfirmed_match_min_iou = 0.30F;
  float confirmed_center_gate = 0.70F;
  float lost_center_gate = 1.50F;
  float minimum_size_ratio = 0.50F;
  float maximum_size_ratio = 2.00F;
  float iou_cost_weight = 0.55F;
  float center_cost_weight = 0.30F;
  float size_cost_weight = 0.15F;
  float confirmed_mahalanobis_gate = 16.0F;
  float lost_mahalanobis_gate = 36.0F;
  float duplicate_iou_threshold = 0.85F;
  double lost_timeout_seconds = 2.5;
  int min_confirm_hits = 3;
  int new_track_delay_frames = 2;
  int lost_new_track_suppression_frames = 10;
  bool fuse_detection_score = true;
  bool publish_tentative_tracks = false;
  bool enable_mahalanobis_gating = true;
  bool lost_recovery_suppression = true;
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
  void set_camera_motion(const CameraMotion& motion) override;
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

  struct PendingCandidate {
    vision_servo_msgs::msg::Target detection;
    int consecutive_hits = 1;
    int required_hits = 1;
  };

  struct AssociationOptions {
    float minimum_iou = 0.0F;
    float center_gate = 1.0F;
    float mahalanobis_gate = 16.0F;
    bool fuse_detection_score = false;
    bool use_mahalanobis_gate = true;
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
      const AssociationOptions& options) const;
  void update_track(Track& track, const vision_servo_msgs::msg::Target& detection);
  void create_track(
      const vision_servo_msgs::msg::Target& detection, int initial_hits = 1);
  void update_pending_candidates(
      const std::vector<int>& detection_indices,
      const vision_servo_msgs::msg::TargetArray& detections);
  bool detection_near_lost_track(
      const vision_servo_msgs::msg::Target& detection) const;
  void remove_duplicate_tracks();
  int allocate_track_id();
  void apply_camera_motion(Track& track) const;
  void apply_camera_motion(vision_servo_msgs::msg::Target& target) const;

  void initialize_filter(
      Track& track, const vision_servo_msgs::msg::Target& detection);
  static float compute_iou(
      const vision_servo_msgs::msg::Target& left,
      const vision_servo_msgs::msg::Target& right);
  static vision_servo_msgs::msg::Target target_from_track(const Track& track);
  static float normalized_center_distance(
      const vision_servo_msgs::msg::Target& left,
      const vision_servo_msgs::msg::Target& right);
  static float size_ratio(
      const vision_servo_msgs::msg::Target& left,
      const vision_servo_msgs::msg::Target& right);
  float mahalanobis_distance_squared(
      const Track& track,
      const vision_servo_msgs::msg::Target& detection) const;
  cv::Mat measurement_for(const vision_servo_msgs::msg::Target& detection) const;
  cv::Mat measurement_noise_for(
      const Track& track, float detection_confidence) const;

  ByteTrackerConfig config_;
  std::map<int, Track> tracks_;
  std::vector<PendingCandidate> pending_candidates_;
  CameraMotion pending_camera_motion_;
  int next_id_ = 0;
  uint64_t last_input_timestamp_ns_ = 0;
  double timeline_seconds_ = 0.0;
};

}  // namespace perception_pkg
