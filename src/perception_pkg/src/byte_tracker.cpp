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

float target_area(const vision_servo_msgs::msg::Target& target)
{
  return std::max(1.0F, target.width) * std::max(1.0F, target.height);
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
  if (!valid_unit_interval(config_.confirmed_match_min_iou) ||
      !valid_unit_interval(config_.lost_match_min_iou) ||
      !valid_unit_interval(config_.second_match_min_iou) ||
      !valid_unit_interval(config_.unconfirmed_match_min_iou) ||
      !valid_unit_interval(config_.duplicate_iou_threshold)) {
    throw std::invalid_argument("ByteTrack IoU thresholds must be in [0, 1]");
  }
  if (!std::isfinite(config_.confirmed_center_gate) ||
      !std::isfinite(config_.lost_center_gate) ||
      config_.confirmed_center_gate <= 0.0F || config_.lost_center_gate <= 0.0F) {
    throw std::invalid_argument("center gates must be finite and positive");
  }
  if (!std::isfinite(config_.minimum_size_ratio) ||
      !std::isfinite(config_.maximum_size_ratio) ||
      config_.minimum_size_ratio <= 0.0F ||
      config_.minimum_size_ratio > 1.0F ||
      config_.maximum_size_ratio < 1.0F ||
      config_.minimum_size_ratio > config_.maximum_size_ratio) {
    throw std::invalid_argument(
        "size ratios require 0 < minimum_size_ratio <= 1 <= maximum_size_ratio");
  }
  const float weight_sum = config_.iou_cost_weight + config_.center_cost_weight +
      config_.size_cost_weight;
  if (!std::isfinite(weight_sum) || weight_sum <= 0.0F ||
      config_.iou_cost_weight < 0.0F || config_.center_cost_weight < 0.0F ||
      config_.size_cost_weight < 0.0F) {
    throw std::invalid_argument("association cost weights must be non-negative and non-zero");
  }
  if (!std::isfinite(config_.confirmed_mahalanobis_gate) ||
      !std::isfinite(config_.lost_mahalanobis_gate) ||
      config_.confirmed_mahalanobis_gate <= 0.0F ||
      config_.lost_mahalanobis_gate <= 0.0F) {
    throw std::invalid_argument("Mahalanobis gates must be finite and positive");
  }
  if (!std::isfinite(config_.lost_timeout_seconds) ||
      config_.lost_timeout_seconds < 0.0) {
    throw std::invalid_argument("lost_timeout_seconds must be finite and non-negative");
  }
  if (config_.min_confirm_hits < 1 || config_.new_track_delay_frames < 1 ||
      config_.lost_new_track_suppression_frames < 1) {
    throw std::invalid_argument("track confirmation/delay frames must be at least one");
  }
}

void ByteTracker::set_camera_motion(const CameraMotion& motion)
{
  pending_camera_motion_ = motion;
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

    const float height = std::max(1.0F, track.kf.statePost.at<float>(3));
    const float position_std = std::max(1.0F, height / 20.0F);
    const float velocity_std = std::max(0.1F, height / 160.0F);
    track.kf.processNoiseCov = cv::Mat::zeros(8, 8, CV_32F);
    track.kf.processNoiseCov.at<float>(0, 0) = position_std * position_std * dt;
    track.kf.processNoiseCov.at<float>(1, 1) = position_std * position_std * dt;
    track.kf.processNoiseCov.at<float>(2, 2) = 1.0e-4F;
    track.kf.processNoiseCov.at<float>(3, 3) = position_std * position_std * dt;
    track.kf.processNoiseCov.at<float>(4, 4) = velocity_std * velocity_std * dt;
    track.kf.processNoiseCov.at<float>(5, 5) = velocity_std * velocity_std * dt;
    track.kf.processNoiseCov.at<float>(6, 6) = 1.0e-5F;
    track.kf.processNoiseCov.at<float>(7, 7) = velocity_std * velocity_std * dt;

    if (track.state == TrackState::kLost) {
      track.kf.statePost.at<float>(6) = 0.0F;
      track.kf.statePost.at<float>(7) = 0.0F;
    }
    track.kf.predict();
    apply_camera_motion(track);
    track.visible_in_current_frame = false;
  }
  for (auto& candidate : pending_candidates_) {
    apply_camera_motion(candidate.detection);
  }
  pending_camera_motion_ = CameraMotion{};
}

void ByteTracker::apply_camera_motion(Track& track) const
{
  if (!pending_camera_motion_.valid) {
    return;
  }
  const auto& a = pending_camera_motion_.affine;
  const float scale = std::clamp(std::hypot(a[0], a[3]), 0.8F, 1.2F);
  cv::Mat covariance_transform = cv::Mat::eye(8, 8, CV_32F);
  covariance_transform.at<float>(0, 0) = a[0];
  covariance_transform.at<float>(0, 1) = a[1];
  covariance_transform.at<float>(1, 0) = a[3];
  covariance_transform.at<float>(1, 1) = a[4];
  covariance_transform.at<float>(3, 3) = scale;
  covariance_transform.at<float>(4, 4) = a[0];
  covariance_transform.at<float>(4, 5) = a[1];
  covariance_transform.at<float>(5, 4) = a[3];
  covariance_transform.at<float>(5, 5) = a[4];
  covariance_transform.at<float>(7, 7) = scale;
  const auto transform_state = [&](cv::Mat& state) {
    const float x = state.at<float>(0);
    const float y = state.at<float>(1);
    const float vx = state.at<float>(4);
    const float vy = state.at<float>(5);
    state.at<float>(0) = a[0] * x + a[1] * y + a[2];
    state.at<float>(1) = a[3] * x + a[4] * y + a[5];
    state.at<float>(3) = std::max(1.0F, state.at<float>(3) * scale);
    state.at<float>(4) = a[0] * vx + a[1] * vy;
    state.at<float>(5) = a[3] * vx + a[4] * vy;
    state.at<float>(7) *= scale;
  };
  transform_state(track.kf.statePre);
  transform_state(track.kf.statePost);
  track.kf.errorCovPre = covariance_transform * track.kf.errorCovPre *
      covariance_transform.t();
  track.kf.errorCovPost = covariance_transform * track.kf.errorCovPost *
      covariance_transform.t();
}

void ByteTracker::apply_camera_motion(
    vision_servo_msgs::msg::Target& target) const
{
  if (!pending_camera_motion_.valid) {
    return;
  }
  const auto& a = pending_camera_motion_.affine;
  const float center_x = target.center[0];
  const float center_y = target.center[1];
  const float scale = std::clamp(std::hypot(a[0], a[3]), 0.8F, 1.2F);
  target.center[0] = a[0] * center_x + a[1] * center_y + a[2];
  target.center[1] = a[3] * center_x + a[4] * center_y + a[5];
  target.width = std::max(1.0F, target.width * scale);
  target.height = std::max(1.0F, target.height * scale);
  target.bbox = {
      target.center[0] - target.width * 0.5F,
      target.center[1] - target.height * 0.5F,
      target.center[0] + target.width * 0.5F,
      target.center[1] + target.height * 0.5F};
}

void ByteTracker::update(
    const vision_servo_msgs::msg::TargetArray& detections)
{
  predict(advance_timeline(detections));

  std::vector<int> high_confidence;
  std::vector<int> low_confidence;
  for (size_t index = 0; index < detections.targets.size(); ++index) {
    const float score = detections.targets[index].confidence;
    if (!std::isfinite(score) || score < config_.track_low_threshold) {
      continue;
    }
    (score >= config_.track_high_threshold ? high_confidence : low_confidence)
        .push_back(static_cast<int>(index));
  }

  std::vector<int> confirmed;
  std::vector<int> lost;
  std::vector<int> tentative;
  for (const auto& [id, track] : tracks_) {
    if (track.state == TrackState::kConfirmed) {
      confirmed.push_back(id);
    } else if (track.state == TrackState::kLost) {
      lost.push_back(id);
    } else if (track.state == TrackState::kTentative) {
      tentative.push_back(id);
    }
  }

  const AssociationOptions confirmed_options{
    config_.confirmed_match_min_iou, config_.confirmed_center_gate,
    config_.confirmed_mahalanobis_gate,
    config_.fuse_detection_score, config_.enable_mahalanobis_gating};
  const auto primary = associate(confirmed, high_confidence, detections, confirmed_options);
  for (const auto& [track_id, detection_index] : primary.matches) {
    update_track(tracks_.at(track_id), detections.targets.at(detection_index));
  }

  const AssociationOptions lost_options{
    config_.lost_match_min_iou, config_.lost_center_gate,
    config_.lost_mahalanobis_gate,
    config_.fuse_detection_score, config_.enable_mahalanobis_gating};
  const auto recovered = associate(
      lost,
      std::vector<int>(primary.unmatched_detections.begin(),
                       primary.unmatched_detections.end()),
      detections, lost_options);
  for (const auto& [track_id, detection_index] : recovered.matches) {
    update_track(tracks_.at(track_id), detections.targets.at(detection_index));
  }

  const AssociationOptions low_score_options{
    config_.second_match_min_iou, config_.confirmed_center_gate,
    config_.confirmed_mahalanobis_gate,
    false, config_.enable_mahalanobis_gating};
  const auto secondary = associate(
      std::vector<int>(primary.unmatched_tracks.begin(), primary.unmatched_tracks.end()),
      low_confidence, detections, low_score_options);
  for (const auto& [track_id, detection_index] : secondary.matches) {
    update_track(tracks_.at(track_id), detections.targets.at(detection_index));
  }
  for (const int track_id : secondary.unmatched_tracks) {
    auto& track = tracks_.at(track_id);
    track.state = TrackState::kLost;
    track.consecutive_hits = 0;
  }

  const AssociationOptions tentative_options{
    config_.unconfirmed_match_min_iou, config_.confirmed_center_gate,
    config_.confirmed_mahalanobis_gate,
    config_.fuse_detection_score, config_.enable_mahalanobis_gating};
  const auto tentative_matches = associate(
      tentative,
      std::vector<int>(recovered.unmatched_detections.begin(),
                       recovered.unmatched_detections.end()),
      detections, tentative_options);
  for (const auto& [track_id, detection_index] : tentative_matches.matches) {
    update_track(tracks_.at(track_id), detections.targets.at(detection_index));
  }
  for (const int track_id : tentative_matches.unmatched_tracks) {
    tracks_.at(track_id).state = TrackState::kRemoved;
  }

  update_pending_candidates(
      std::vector<int>(tentative_matches.unmatched_detections.begin(),
                       tentative_matches.unmatched_detections.end()),
      detections);

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
    const AssociationOptions& options) const
{
  AssociationResult result;
  result.unmatched_tracks.insert(track_ids.begin(), track_ids.end());
  result.unmatched_detections.insert(
      detection_indices.begin(), detection_indices.end());
  if (track_ids.empty() || detection_indices.empty()) {
    return result;
  }

  const float weight_sum = config_.iou_cost_weight + config_.center_cost_weight +
      config_.size_cost_weight;
  const float maximum_log_size = std::max(
      std::abs(std::log(config_.minimum_size_ratio)),
      std::abs(std::log(config_.maximum_size_ratio)));
  std::vector<std::vector<double>> costs(
      track_ids.size(),
      std::vector<double>(detection_indices.size(), kInvalidAssociationCost));
  for (size_t row = 0; row < track_ids.size(); ++row) {
    const auto& track = tracks_.at(track_ids[row]);
    const auto predicted = target_from_track(track);
    for (size_t column = 0; column < detection_indices.size(); ++column) {
      const auto& detection = detections.targets.at(detection_indices[column]);
      if (detection.class_name != predicted.class_name) {
        continue;
      }
      const float iou = compute_iou(predicted, detection);
      const float center_distance = normalized_center_distance(predicted, detection);
      const float relative_size = size_ratio(predicted, detection);
      if ((iou < options.minimum_iou && center_distance > options.center_gate) ||
          relative_size < config_.minimum_size_ratio ||
          relative_size > config_.maximum_size_ratio) {
        continue;
      }
      if (options.use_mahalanobis_gate &&
          mahalanobis_distance_squared(track, detection) > options.mahalanobis_gate) {
        continue;
      }

      const float size_cost = maximum_log_size > 1.0e-6F
          ? std::clamp(std::abs(std::log(relative_size)) / maximum_log_size, 0.0F, 1.0F)
          : 0.0F;
      double cost = (
          config_.iou_cost_weight * (1.0F - iou) +
          config_.center_cost_weight *
              std::clamp(center_distance / options.center_gate, 0.0F, 1.0F) +
          config_.size_cost_weight * size_cost) / weight_sum;
      if (options.fuse_detection_score) {
        cost += 0.05 * (1.0 - std::clamp(
            static_cast<double>(detection.confidence), 0.0, 1.0));
      }
      costs[row][column] = std::min(cost, 0.999);
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
  track.kf.measurementNoiseCov = measurement_noise_for(track, detection.confidence);
  track.kf.correct(measurement_for(detection));
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
    const vision_servo_msgs::msg::Target& detection, int initial_hits)
{
  Track track;
  track.id = allocate_track_id();
  track.class_name = detection.class_name;
  track.confidence = detection.confidence;
  track.consecutive_hits = std::max(1, initial_hits);
  track.total_hits = track.consecutive_hits;
  track.visible_in_current_frame = true;
  track.created_time_seconds = timeline_seconds_;
  track.last_seen_time_seconds = timeline_seconds_;
  track.state = track.consecutive_hits >= config_.min_confirm_hits
      ? TrackState::kConfirmed : TrackState::kTentative;
  initialize_filter(track, detection);
  tracks_.emplace(track.id, std::move(track));
}

void ByteTracker::update_pending_candidates(
    const std::vector<int>& detection_indices,
    const vision_servo_msgs::msg::TargetArray& detections)
{
  const int base_required_hits = config_.publish_tentative_tracks
      ? config_.new_track_delay_frames
      : std::max(config_.new_track_delay_frames, config_.min_confirm_hits);
  const auto required_candidate_hits = [&](bool near_lost) {
    return near_lost && config_.lost_recovery_suppression
        ? std::max(base_required_hits, config_.lost_new_track_suppression_frames)
        : base_required_hits;
  };
  std::set<int> unused_detections;
  for (const int index : detection_indices) {
    unused_detections.insert(index);
  }

  std::vector<PendingCandidate> next_candidates;
  for (auto candidate : pending_candidates_) {
    int best_index = -1;
    float best_cost = std::numeric_limits<float>::infinity();
    for (const int index : unused_detections) {
      const auto& detection = detections.targets.at(index);
      if (detection.class_name != candidate.detection.class_name) {
        continue;
      }
      const float iou = compute_iou(candidate.detection, detection);
      const float center = normalized_center_distance(candidate.detection, detection);
      const float ratio = size_ratio(candidate.detection, detection);
      if ((iou < config_.unconfirmed_match_min_iou &&
           center > config_.confirmed_center_gate) ||
          ratio < config_.minimum_size_ratio || ratio > config_.maximum_size_ratio) {
        continue;
      }
      const float cost = (1.0F - iou) + center;
      if (cost < best_cost) {
        best_cost = cost;
        best_index = index;
      }
    }
    if (best_index < 0) {
      continue;
    }
    candidate.detection = detections.targets.at(best_index);
    ++candidate.consecutive_hits;
    unused_detections.erase(best_index);
    const bool near_lost = detection_near_lost_track(candidate.detection);
    candidate.required_hits = std::max(
        candidate.required_hits, required_candidate_hits(near_lost));
    if (candidate.consecutive_hits >= candidate.required_hits) {
      create_track(candidate.detection, candidate.consecutive_hits);
    } else {
      next_candidates.push_back(std::move(candidate));
    }
  }

  for (const int index : unused_detections) {
    if (detections.targets.at(index).confidence < config_.new_track_threshold) {
      continue;
    }
    PendingCandidate candidate;
    candidate.detection = detections.targets.at(index);
    const bool near_lost = detection_near_lost_track(candidate.detection);
    candidate.required_hits = required_candidate_hits(near_lost);
    if (candidate.required_hits <= 1) {
      create_track(candidate.detection, 1);
    } else {
      next_candidates.push_back(std::move(candidate));
    }
  }
  pending_candidates_ = std::move(next_candidates);
}

bool ByteTracker::detection_near_lost_track(
    const vision_servo_msgs::msg::Target& detection) const
{
  for (const auto& [id, track] : tracks_) {
    (void)id;
    if (track.state != TrackState::kLost || track.class_name != detection.class_name) {
      continue;
    }
    const auto predicted = target_from_track(track);
    const float ratio = size_ratio(predicted, detection);
    if (normalized_center_distance(predicted, detection) <= config_.lost_center_gate &&
        ratio >= config_.minimum_size_ratio && ratio <= config_.maximum_size_ratio) {
      return true;
    }
  }
  return false;
}

int ByteTracker::allocate_track_id()
{
  if (next_id_ == std::numeric_limits<int>::max()) {
    next_id_ = 0;
  }
  while (tracks_.count(next_id_) > 0) {
    next_id_ = next_id_ == std::numeric_limits<int>::max() ? 0 : next_id_ + 1;
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
      const auto& confirmed_track = tracks_.at(confirmed_id);
      const auto& lost_track = tracks_.at(lost_id);
      if (confirmed_track.class_name != lost_track.class_name ||
          compute_iou(target_from_track(confirmed_track), target_from_track(lost_track)) <
              config_.duplicate_iou_threshold) {
        continue;
      }
      remove_ids.insert(
          confirmed_track.total_hits >= lost_track.total_hits ? lost_id : confirmed_id);
    }
  }
  for (const int id : remove_ids) {
    tracks_.at(id).state = TrackState::kRemoved;
  }
}

void ByteTracker::initialize_filter(
    Track& track, const vision_servo_msgs::msg::Target& detection)
{
  const float height = std::max(1.0F, detection.height);
  track.kf.init(8, 4, 0);
  track.kf.transitionMatrix = cv::Mat::eye(8, 8, CV_32F);
  for (int index = 0; index < 4; ++index) {
    track.kf.transitionMatrix.at<float>(index, index + 4) = 1.0F;
  }
  track.kf.measurementMatrix = cv::Mat::zeros(4, 8, CV_32F);
  for (int index = 0; index < 4; ++index) {
    track.kf.measurementMatrix.at<float>(index, index) = 1.0F;
  }
  track.kf.statePost = cv::Mat::zeros(8, 1, CV_32F);
  const cv::Mat measurement = (cv::Mat_<float>(4, 1) <<
      detection.center[0], detection.center[1],
      std::max(1.0F, detection.width) / height, height);
  measurement.copyTo(track.kf.statePost.rowRange(0, 4));
  track.kf.statePre = track.kf.statePost.clone();

  track.kf.errorCovPost = cv::Mat::zeros(8, 8, CV_32F);
  const float position_std = std::max(1.0F, height / 10.0F);
  const float velocity_std = std::max(1.0F, height / 4.0F);
  track.kf.errorCovPost.at<float>(0, 0) = position_std * position_std;
  track.kf.errorCovPost.at<float>(1, 1) = position_std * position_std;
  track.kf.errorCovPost.at<float>(2, 2) = 1.0e-2F;
  track.kf.errorCovPost.at<float>(3, 3) = position_std * position_std;
  track.kf.errorCovPost.at<float>(4, 4) = velocity_std * velocity_std;
  track.kf.errorCovPost.at<float>(5, 5) = velocity_std * velocity_std;
  track.kf.errorCovPost.at<float>(6, 6) = 1.0e-4F;
  track.kf.errorCovPost.at<float>(7, 7) = velocity_std * velocity_std;
  track.kf.errorCovPre = track.kf.errorCovPost.clone();
  track.kf.measurementNoiseCov = measurement_noise_for(track, detection.confidence);
}

cv::Mat ByteTracker::measurement_for(
    const vision_servo_msgs::msg::Target& detection) const
{
  const float height = std::max(1.0F, detection.height);
  return (cv::Mat_<float>(4, 1) <<
      detection.center[0], detection.center[1],
      std::max(1.0F, detection.width) / height, height);
}

cv::Mat ByteTracker::measurement_noise_for(
    const Track& track, float detection_confidence) const
{
  const float height = std::max(1.0F, track.kf.statePre.at<float>(3));
  const float confidence_scale = 1.0F + 2.0F *
      (1.0F - std::clamp(detection_confidence, 0.0F, 1.0F));
  const float position_std = std::max(1.0F, height / 20.0F) * confidence_scale;
  cv::Mat noise = cv::Mat::zeros(4, 4, CV_32F);
  noise.at<float>(0, 0) = position_std * position_std;
  noise.at<float>(1, 1) = position_std * position_std;
  noise.at<float>(2, 2) = 1.0e-2F * confidence_scale;
  noise.at<float>(3, 3) = position_std * position_std;
  return noise;
}

float ByteTracker::mahalanobis_distance_squared(
    const Track& track,
    const vision_servo_msgs::msg::Target& detection) const
{
  const cv::Mat residual = measurement_for(detection) -
      track.kf.measurementMatrix * track.kf.statePre;
  const cv::Mat innovation_covariance =
      track.kf.measurementMatrix * track.kf.errorCovPre *
      track.kf.measurementMatrix.t() +
      measurement_noise_for(track, detection.confidence);
  cv::Mat solved;
  if (!cv::solve(innovation_covariance, residual, solved, cv::DECOMP_CHOLESKY)) {
    return std::numeric_limits<float>::infinity();
  }
  return static_cast<float>(residual.dot(solved));
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
  const float union_area = target_area(left) + target_area(right) - intersection;
  return union_area > 1.0e-6F ? intersection / union_area : 0.0F;
}

float ByteTracker::normalized_center_distance(
    const vision_servo_msgs::msg::Target& left,
    const vision_servo_msgs::msg::Target& right)
{
  const float dx = left.center[0] - right.center[0];
  const float dy = left.center[1] - right.center[1];
  const float left_diagonal = std::hypot(left.width, left.height);
  const float right_diagonal = std::hypot(right.width, right.height);
  const float scale = std::max(1.0F, 0.5F * (left_diagonal + right_diagonal));
  return std::hypot(dx, dy) / scale;
}

float ByteTracker::size_ratio(
    const vision_servo_msgs::msg::Target& left,
    const vision_servo_msgs::msg::Target& right)
{
  return target_area(right) / target_area(left);
}

vision_servo_msgs::msg::Target ByteTracker::target_from_track(const Track& track)
{
  const cv::Mat& state = track.visible_in_current_frame
      ? track.kf.statePost : track.kf.statePre;
  const float center_x = state.at<float>(0);
  const float center_y = state.at<float>(1);
  const float aspect = std::clamp(state.at<float>(2), 0.05F, 20.0F);
  const float height = std::max(1.0F, state.at<float>(3));
  const float width = std::max(1.0F, aspect * height);
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
