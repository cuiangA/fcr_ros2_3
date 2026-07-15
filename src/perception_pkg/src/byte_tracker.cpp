#include "perception_pkg/byte_tracker.hpp"

#include "perception_pkg/assignment_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace perception_pkg {
namespace {

constexpr uint64_t kNanosecondsPerSecond = 1000000000ULL;
constexpr double kDefaultFramePeriodSeconds = 1.0 / 30.0;
constexpr double kInvalidAssociationCost = 1000000.0;

bool valid_unit_interval(float value)
{
  return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
}

}  // namespace

ByteTracker::ByteTracker(ByteTrackerConfig config)
  : config_(std::move(config))
{
  validate_config();
}

void ByteTracker::validate_config() const
{
  if (!valid_unit_interval(config_.track_low_threshold) ||
      !valid_unit_interval(config_.track_high_threshold) ||
      config_.track_low_threshold > config_.track_high_threshold) {
    throw std::invalid_argument(
        "ByteTrack thresholds require 0 <= track_low_threshold <= "
        "track_high_threshold <= 1");
  }
  if (!valid_unit_interval(config_.new_track_threshold) ||
      config_.new_track_threshold < config_.track_high_threshold) {
    throw std::invalid_argument(
        "new_track_threshold must be in [track_high_threshold, 1]");
  }
  if (!valid_unit_interval(config_.first_match_min_iou) ||
      !valid_unit_interval(config_.second_match_min_iou) ||
      !valid_unit_interval(config_.unconfirmed_match_min_iou) ||
      !valid_unit_interval(config_.duplicate_iou_threshold)) {
    throw std::invalid_argument("ByteTrack IoU thresholds must be in [0, 1]");
  }
  if (!std::isfinite(config_.lost_timeout_seconds) ||
      config_.lost_timeout_seconds < 0.0) {
    throw std::invalid_argument("lost_timeout_seconds must be finite and non-negative");
  }
  if (config_.min_confirm_hits < 1) {
    throw std::invalid_argument("min_confirm_hits must be at least one");
  }
}

double ByteTracker::advance_timeline(
    const vision_servo_msgs::msg::TargetArray& detections)
{
  const uint64_t timestamp_ns = detections.header.stamp.sec >= 0
      ? static_cast<uint64_t>(detections.header.stamp.sec) * kNanosecondsPerSecond +
        static_cast<uint64_t>(detections.header.stamp.nanosec)
      : 0;
  double elapsed_seconds = kDefaultFramePeriodSeconds;
  if (last_input_timestamp_ns_ > 0 && timestamp_ns > last_input_timestamp_ns_) {
    elapsed_seconds =
        static_cast<double>(timestamp_ns - last_input_timestamp_ns_) * 1.0e-9;
  }
  if (timestamp_ns > last_input_timestamp_ns_) {
    last_input_timestamp_ns_ = timestamp_ns;
  }
  // Track retention follows actual elapsed message time. Kalman prediction is
  // clamped separately so a long input gap cannot explode the motion state.
  timeline_seconds_ += elapsed_seconds;
  return std::clamp(elapsed_seconds, 1.0 / 120.0, 0.2);
}

void ByteTracker::predict(double dt_seconds)
{
  for (auto& [id, track] : tracks_) {
    (void)id;
    if (track.state == TrackState::kRemoved) {
      continue;
    }
    const float dt = static_cast<float>(dt_seconds);
    for (int index = 0; index < 4; ++index) {
      track.kf.transitionMatrix.at<float>(index, index + 4) = dt;
    }
    // Lost tracks should not extrapolate scale indefinitely.
    if (track.state == TrackState::kLost) {
      track.kf.statePost.at<float>(6) = 0.0F;
      track.kf.statePost.at<float>(7) = 0.0F;
    }
    track.kf.predict();
    track.visible_in_current_frame = false;
  }
}

void ByteTracker::update(
    const vision_servo_msgs::msg::TargetArray& detections)
{
  const double dt_seconds = advance_timeline(detections);
  predict(dt_seconds);

  std::vector<int> high_confidence;
  std::vector<int> low_confidence;
  for (size_t index = 0; index < detections.targets.size(); ++index) {
    const float score = detections.targets[index].confidence;
    if (!std::isfinite(score) || score < config_.track_low_threshold) {
      continue;
    }
    if (score >= config_.track_high_threshold) {
      high_confidence.push_back(static_cast<int>(index));
    } else {
      low_confidence.push_back(static_cast<int>(index));
    }
  }

  std::vector<int> confirmed_and_lost;
  std::vector<int> tentative;
  for (const auto& [id, track] : tracks_) {
    if (track.state == TrackState::kConfirmed || track.state == TrackState::kLost) {
      confirmed_and_lost.push_back(id);
    } else if (track.state == TrackState::kTentative) {
      tentative.push_back(id);
    }
  }

  // Stage 1: associate established and recently lost tracks with reliable boxes.
  const auto primary = associate(
      confirmed_and_lost, high_confidence, detections,
      config_.first_match_min_iou, config_.fuse_detection_score);
  for (const auto& [track_id, detection_index] : primary.matches) {
    update_track(tracks_.at(track_id), detections.targets.at(detection_index));
  }

  // Stage 2: only previously active tracks may be recovered by low-score boxes.
  // A low-score box can therefore preserve an ID but can never create one.
  std::vector<int> second_stage_tracks;
  for (const int track_id : primary.unmatched_tracks) {
    if (tracks_.at(track_id).state == TrackState::kConfirmed) {
      second_stage_tracks.push_back(track_id);
    }
  }
  const auto secondary = associate(
      second_stage_tracks, low_confidence, detections,
      config_.second_match_min_iou, false);
  for (const auto& [track_id, detection_index] : secondary.matches) {
    update_track(tracks_.at(track_id), detections.targets.at(detection_index));
  }
  for (const int track_id : secondary.unmatched_tracks) {
    auto& track = tracks_.at(track_id);
    track.state = TrackState::kLost;
    track.consecutive_hits = 0;
  }
  // Tracks which were already LOST before this frame stay LOST when stage 1 misses.

  // Unconfirmed tracks are matched only with high-confidence detections left by
  // stage 1. This keeps tentative identities from consuming low-score noise.
  const auto tentative_matches = associate(
      tentative,
      std::vector<int>(
          primary.unmatched_detections.begin(), primary.unmatched_detections.end()),
      detections, config_.unconfirmed_match_min_iou,
      config_.fuse_detection_score);
  for (const auto& [track_id, detection_index] : tentative_matches.matches) {
    update_track(tracks_.at(track_id), detections.targets.at(detection_index));
  }
  for (const int track_id : tentative_matches.unmatched_tracks) {
    tracks_.at(track_id).state = TrackState::kRemoved;
  }

  // Only an unmatched high-confidence detection above the stricter creation
  // threshold may start a new identity.
  for (const int detection_index : tentative_matches.unmatched_detections) {
    const auto& detection = detections.targets.at(detection_index);
    if (detection.confidence >= config_.new_track_threshold) {
      create_track(detection);
    }
  }

  for (auto& [id, track] : tracks_) {
    (void)id;
    if (track.state == TrackState::kLost &&
        timeline_seconds_ - track.last_seen_time_seconds >
            config_.lost_timeout_seconds) {
      track.state = TrackState::kRemoved;
    }
  }

  remove_duplicate_tracks();
  for (auto iterator = tracks_.begin(); iterator != tracks_.end();) {
    if (iterator->second.state == TrackState::kRemoved) {
      iterator = tracks_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

ByteTracker::AssociationResult ByteTracker::associate(
    const std::vector<int>& track_ids,
    const std::vector<int>& detection_indices,
    const vision_servo_msgs::msg::TargetArray& detections,
    float minimum_iou,
    bool fuse_detection_score) const
{
  AssociationResult result;
  result.unmatched_tracks.insert(track_ids.begin(), track_ids.end());
  result.unmatched_detections.insert(
      detection_indices.begin(), detection_indices.end());
  if (track_ids.empty() || detection_indices.empty()) {
    return result;
  }

  std::vector<std::vector<double>> costs(
      track_ids.size(),
      std::vector<double>(detection_indices.size(), kInvalidAssociationCost));
  for (size_t row = 0; row < track_ids.size(); ++row) {
    const auto predicted = target_from_track(tracks_.at(track_ids[row]));
    for (size_t column = 0; column < detection_indices.size(); ++column) {
      const auto& detection = detections.targets.at(detection_indices[column]);
      if (detection.class_name != predicted.class_name) {
        continue;
      }
      const float iou = compute_iou(predicted, detection);
      if (iou < minimum_iou) {
        continue;
      }
      const double similarity = fuse_detection_score
          ? static_cast<double>(iou) * std::clamp(
                static_cast<double>(detection.confidence), 0.0, 1.0)
          : static_cast<double>(iou);
      costs[row][column] = 1.0 - similarity;
    }
  }

  const auto assignment = solve_assignment(costs);
  for (size_t row = 0; row < assignment.size(); ++row) {
    const int column = assignment[row];
    if (column < 0 ||
        costs[row][static_cast<size_t>(column)] >= kInvalidAssociationCost) {
      continue;
    }
    const int track_id = track_ids[row];
    const int detection_index = detection_indices[static_cast<size_t>(column)];
    result.matches.emplace_back(track_id, detection_index);
    result.unmatched_tracks.erase(track_id);
    result.unmatched_detections.erase(detection_index);
  }
  return result;
}

void ByteTracker::update_track(
    Track& track, const vision_servo_msgs::msg::Target& detection)
{
  const float width = std::max(1.0F, detection.bbox[2] - detection.bbox[0]);
  const float height = std::max(1.0F, detection.bbox[3] - detection.bbox[1]);
  const cv::Mat measurement =
      (cv::Mat_<float>(4, 1) << detection.center[0], detection.center[1], width, height);
  track.kf.correct(measurement);
  track.class_name = detection.class_name;
  track.confidence = detection.confidence;
  track.visible_in_current_frame = true;
  track.last_seen_time_seconds = timeline_seconds_;
  ++track.total_hits;

  if (track.state == TrackState::kLost || track.state == TrackState::kConfirmed) {
    track.state = TrackState::kConfirmed;
    track.consecutive_hits = std::max(track.consecutive_hits + 1, 1);
    return;
  }
  ++track.consecutive_hits;
  if (track.consecutive_hits >= config_.min_confirm_hits) {
    track.state = TrackState::kConfirmed;
  }
}

void ByteTracker::create_track(
    const vision_servo_msgs::msg::Target& detection)
{
  Track track;
  track.id = allocate_track_id();
  track.class_name = detection.class_name;
  track.confidence = detection.confidence;
  track.consecutive_hits = 1;
  track.total_hits = 1;
  track.visible_in_current_frame = true;
  track.created_time_seconds = timeline_seconds_;
  track.last_seen_time_seconds = timeline_seconds_;
  track.state = config_.min_confirm_hits <= 1
      ? TrackState::kConfirmed : TrackState::kTentative;
  initialize_filter(track, detection);
  tracks_.emplace(track.id, std::move(track));
}

int ByteTracker::allocate_track_id()
{
  if (next_id_ == std::numeric_limits<int>::max()) {
    next_id_ = 0;
  }
  while (tracks_.count(next_id_) > 0) {
    if (next_id_ == std::numeric_limits<int>::max()) {
      next_id_ = 0;
    } else {
      ++next_id_;
    }
  }
  return next_id_++;
}

void ByteTracker::remove_duplicate_tracks()
{
  std::vector<int> confirmed;
  std::vector<int> lost;
  for (const auto& [id, track] : tracks_) {
    if (track.state == TrackState::kConfirmed) {
      confirmed.push_back(id);
    } else if (track.state == TrackState::kLost) {
      lost.push_back(id);
    }
  }

  std::set<int> remove_ids;
  for (const int confirmed_id : confirmed) {
    for (const int lost_id : lost) {
      auto& confirmed_track = tracks_.at(confirmed_id);
      auto& lost_track = tracks_.at(lost_id);
      if (confirmed_track.class_name != lost_track.class_name ||
          compute_iou(
              target_from_track(confirmed_track), target_from_track(lost_track)) <
              config_.duplicate_iou_threshold) {
        continue;
      }
      if (confirmed_track.total_hits >= lost_track.total_hits) {
        remove_ids.insert(lost_id);
      } else {
        remove_ids.insert(confirmed_id);
      }
    }
  }
  for (const int id : remove_ids) {
    tracks_.at(id).state = TrackState::kRemoved;
  }
}

void ByteTracker::initialize_filter(
    Track& track, const vision_servo_msgs::msg::Target& detection)
{
  const float width = std::max(1.0F, detection.bbox[2] - detection.bbox[0]);
  const float height = std::max(1.0F, detection.bbox[3] - detection.bbox[1]);
  track.kf.init(8, 4, 0);
  track.kf.transitionMatrix = cv::Mat::eye(8, 8, CV_32F);
  for (int index = 0; index < 4; ++index) {
    track.kf.transitionMatrix.at<float>(index, index + 4) = 1.0F;
  }
  track.kf.measurementMatrix = cv::Mat::zeros(4, 8, CV_32F);
  for (int index = 0; index < 4; ++index) {
    track.kf.measurementMatrix.at<float>(index, index) = 1.0F;
  }
  cv::setIdentity(track.kf.processNoiseCov, cv::Scalar::all(1e-2));
  cv::setIdentity(track.kf.measurementNoiseCov, cv::Scalar::all(1e-1));
  cv::setIdentity(track.kf.errorCovPost, cv::Scalar::all(1.0));
  track.kf.statePost = cv::Mat::zeros(8, 1, CV_32F);
  track.kf.statePost.at<float>(0) = detection.center[0];
  track.kf.statePost.at<float>(1) = detection.center[1];
  track.kf.statePost.at<float>(2) = width;
  track.kf.statePost.at<float>(3) = height;
  track.kf.statePre = track.kf.statePost.clone();
}

float ByteTracker::compute_iou(
    const vision_servo_msgs::msg::Target& left,
    const vision_servo_msgs::msg::Target& right)
{
  const float x1 = std::max(left.bbox[0], right.bbox[0]);
  const float y1 = std::max(left.bbox[1], right.bbox[1]);
  const float x2 = std::min(left.bbox[2], right.bbox[2]);
  const float y2 = std::min(left.bbox[3], right.bbox[3]);
  const float intersection =
      std::max(0.0F, x2 - x1) * std::max(0.0F, y2 - y1);
  const float left_area =
      std::max(0.0F, left.bbox[2] - left.bbox[0]) *
      std::max(0.0F, left.bbox[3] - left.bbox[1]);
  const float right_area =
      std::max(0.0F, right.bbox[2] - right.bbox[0]) *
      std::max(0.0F, right.bbox[3] - right.bbox[1]);
  const float union_area = left_area + right_area - intersection;
  return union_area > 1e-6F ? intersection / union_area : 0.0F;
}

vision_servo_msgs::msg::Target ByteTracker::target_from_track(const Track& track)
{
  const cv::Mat& state = track.visible_in_current_frame
      ? track.kf.statePost : track.kf.statePre;
  const float center_x = state.at<float>(0);
  const float center_y = state.at<float>(1);
  const float width = std::max(1.0F, state.at<float>(2));
  const float height = std::max(1.0F, state.at<float>(3));
  vision_servo_msgs::msg::Target target;
  target.id = track.id;
  target.class_name = track.class_name;
  target.confidence = track.confidence;
  target.center = {center_x, center_y};
  target.bbox = {
    center_x - width * 0.5F, center_y - height * 0.5F,
    center_x + width * 0.5F, center_y + height * 0.5F};
  target.width = width;
  target.height = height;
  target.visible = track.visible_in_current_frame;
  if (track.state == TrackState::kTentative) {
    target.tracking_state =
        vision_servo_msgs::msg::Target::TRACKING_STATE_TENTATIVE;
  } else if (track.state == TrackState::kLost) {
    target.tracking_state = vision_servo_msgs::msg::Target::TRACKING_STATE_LOST;
  } else {
    target.tracking_state =
        vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED;
  }
  return target;
}

vision_servo_msgs::msg::TargetArray ByteTracker::get_tracks(
    uint64_t timestamp, const std::string& frame_id)
{
  vision_servo_msgs::msg::TargetArray result;
  result.header.stamp.sec = static_cast<int32_t>(timestamp / kNanosecondsPerSecond);
  result.header.stamp.nanosec =
      static_cast<uint32_t>(timestamp % kNanosecondsPerSecond);
  result.header.frame_id = frame_id;
  result.tracking_id = -1;
  for (const auto& [id, track] : tracks_) {
    (void)id;
    if (track.state == TrackState::kRemoved ||
        (track.state == TrackState::kTentative &&
         !config_.publish_tentative_tracks)) {
      continue;
    }
    auto target = target_from_track(track);
    target.header = result.header;
    result.targets.push_back(std::move(target));
  }
  return result;
}

}  // namespace perception_pkg
