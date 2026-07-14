/**
 * @file multi_object_tracker.cpp
 * @brief MultiObjectTracker core implementation shared by standalone and composable perception paths.
 */

#include "perception_pkg/tracking_node.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace perception_pkg {
namespace {

constexpr uint64_t kNanosecondsPerSecond = 1000000000ULL;
constexpr double kInvalidAssociationCost = 1000000.0;

std::vector<int> solve_assignment(const std::vector<std::vector<double>>& costs)
{
  const size_t row_count = costs.size();
  const size_t column_count = row_count == 0 ? 0 : costs.front().size();
  const size_t size = std::max(row_count, column_count);
  if (size == 0) {
    return {};
  }

  std::vector<std::vector<double>> square(size, std::vector<double>(size, 0.0));
  for (size_t row = 0; row < size; ++row) {
    for (size_t column = 0; column < size; ++column) {
      if (row < row_count && column < column_count) {
        square[row][column] = costs[row][column];
      } else if (row < row_count) {
        square[row][column] = 1.0;  // unmatched real track
      }
    }
  }

  // Hungarian algorithm for a square minimum-cost assignment, using 1-based
  // indexing internally. Input order is stable, so ties are deterministic.
  std::vector<double> row_potential(size + 1, 0.0);
  std::vector<double> column_potential(size + 1, 0.0);
  std::vector<size_t> column_match(size + 1, 0);
  std::vector<size_t> previous_column(size + 1, 0);
  for (size_t row = 1; row <= size; ++row) {
    column_match[0] = row;
    size_t current_column = 0;
    std::vector<double> minimum(
        size + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(size + 1, false);
    do {
      used[current_column] = true;
      const size_t current_row = column_match[current_column];
      double delta = std::numeric_limits<double>::infinity();
      size_t next_column = 0;
      for (size_t column = 1; column <= size; ++column) {
        if (used[column]) {
          continue;
        }
        const double reduced_cost = square[current_row - 1][column - 1] -
            row_potential[current_row] - column_potential[column];
        if (reduced_cost < minimum[column]) {
          minimum[column] = reduced_cost;
          previous_column[column] = current_column;
        }
        if (minimum[column] < delta) {
          delta = minimum[column];
          next_column = column;
        }
      }
      for (size_t column = 0; column <= size; ++column) {
        if (used[column]) {
          row_potential[column_match[column]] += delta;
          column_potential[column] -= delta;
        } else {
          minimum[column] -= delta;
        }
      }
      current_column = next_column;
    } while (column_match[current_column] != 0);

    do {
      const size_t previous = previous_column[current_column];
      column_match[current_column] = column_match[previous];
      current_column = previous;
    } while (current_column != 0);
  }

  std::vector<int> assignment(row_count, -1);
  for (size_t column = 1; column <= size; ++column) {
    const size_t row = column_match[column];
    if (row > 0 && row <= row_count && column <= column_count) {
      assignment[row - 1] = static_cast<int>(column - 1);
    }
  }
  return assignment;
}

void initialize_track_filter(
    MultiObjectTracker::Track& track,
    const vision_servo_msgs::msg::Target& detection)
{
  const float width = std::max(1.0f, detection.bbox[2] - detection.bbox[0]);
  const float height = std::max(1.0f, detection.bbox[3] - detection.bbox[1]);

  track.kf.init(8, 4, 0);
  track.kf.transitionMatrix = cv::Mat::eye(8, 8, CV_32F);
  for (int i = 0; i < 4; ++i) {
    track.kf.transitionMatrix.at<float>(i, i + 4) = 1.0f;
  }

  track.kf.measurementMatrix = cv::Mat::zeros(4, 8, CV_32F);
  for (int i = 0; i < 4; ++i) {
    track.kf.measurementMatrix.at<float>(i, i) = 1.0f;
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

vision_servo_msgs::msg::Target target_from_track(const MultiObjectTracker::Track& track)
{
  const auto& state = track.visible_in_current_frame
      ? track.kf.statePost
      : track.kf.statePre;
  const float cx = state.at<float>(0);
  const float cy = state.at<float>(1);
  const float width = std::max(1.0f, state.at<float>(2));
  const float height = std::max(1.0f, state.at<float>(3));
  const float half_w = width * 0.5f;
  const float half_h = height * 0.5f;

  vision_servo_msgs::msg::Target target;
  target.id = track.id;
  target.class_name = track.class_name;
  target.confidence = track.confidence;
  target.center = {cx, cy};
  target.bbox = {cx - half_w, cy - half_h, cx + half_w, cy + half_h};
  target.width = width;
  target.height = height;
  target.visible = track.visible_in_current_frame;
  target.tracking_state = track.visible_in_current_frame
      ? vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED
      : vision_servo_msgs::msg::Target::TRACKING_STATE_LOST;
  return target;
}

}  // namespace

MultiObjectTracker::MultiObjectTracker(int max_age, int min_hits, float iou_threshold)
  : next_id_(0), max_age_(max_age), min_hits_(min_hits), iou_threshold_(iou_threshold)
{
  if (max_age < 0) {
    throw std::invalid_argument("max_age must be non-negative");
  }
  if (min_hits < 1) {
    throw std::invalid_argument("min_hits must be at least one");
  }
  if (iou_threshold < 0.0F || iou_threshold > 1.0F) {
    throw std::invalid_argument("iou_threshold must be in [0, 1]");
  }
}

void MultiObjectTracker::predict(float dt_seconds)
{
  for (auto& [id, track] : tracks_) {
    (void)id;
    for (int i = 0; i < 4; ++i) {
      track.kf.transitionMatrix.at<float>(i, i + 4) = dt_seconds;
    }
    track.kf.predict();
    track.age++;
    track.consecutive_invisible_count++;
    track.visible_in_current_frame = false;
  }
}

void MultiObjectTracker::update(const vision_servo_msgs::msg::TargetArray& detections)
{
  const uint64_t timestamp_ns = detections.header.stamp.sec >= 0
      ? static_cast<uint64_t>(detections.header.stamp.sec) * kNanosecondsPerSecond +
        static_cast<uint64_t>(detections.header.stamp.nanosec)
      : 0;
  float dt_seconds = 1.0F / 30.0F;
  if (last_timestamp_ns_ > 0 && timestamp_ns > last_timestamp_ns_) {
    dt_seconds = static_cast<float>(timestamp_ns - last_timestamp_ns_) * 1.0e-9F;
    dt_seconds = std::clamp(dt_seconds, 1.0F / 120.0F, 0.2F);
  }
  if (timestamp_ns > last_timestamp_ns_) {
    last_timestamp_ns_ = timestamp_ns;
  }
  predict(dt_seconds);

  std::map<int, int> matches;
  std::set<int> unmatched_detections;
  std::set<int> unmatched_tracks;
  associate_detections_to_tracks(detections, matches, unmatched_detections, unmatched_tracks);

  for (const auto& [track_id, det_idx] : matches) {
    const auto& det = detections.targets[det_idx];
    auto& track = tracks_.at(track_id);
    const float width = std::max(1.0f, det.bbox[2] - det.bbox[0]);
    const float height = std::max(1.0f, det.bbox[3] - det.bbox[1]);
    cv::Mat measurement = (cv::Mat_<float>(4, 1) << det.center[0], det.center[1], width, height);

    track.kf.correct(measurement);
    track.age = 0;
    track.consecutive_invisible_count = 0;
    track.consecutive_visible_count++;
    track.visible_in_current_frame = true;
    if (track.consecutive_visible_count >= min_hits_) {
      track.confirmed = true;
    }

    track.confidence = det.confidence;
    track.class_name = det.class_name;
  }

  for (int det_idx : unmatched_detections) {
    const auto& det = detections.targets[det_idx];
    if (next_id_ == std::numeric_limits<int>::max()) {
      next_id_ = 0;
      while (tracks_.count(next_id_) > 0) {
        ++next_id_;
      }
    }

    Track track;
    track.id = next_id_++;
    track.class_name = det.class_name;
    track.age = 0;
    track.consecutive_visible_count = 1;
    track.consecutive_invisible_count = 0;
    track.confidence = det.confidence;
    track.confirmed = min_hits_ <= 1;
    track.visible_in_current_frame = true;
    initialize_track_filter(track, det);
    tracks_[track.id] = track;
  }

  for (const int track_id : unmatched_tracks) {
    auto track = tracks_.find(track_id);
    if (track != tracks_.end()) {
      track->second.consecutive_visible_count = 0;
    }
  }

  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (it->second.consecutive_invisible_count > max_age_) {
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
}

vision_servo_msgs::msg::TargetArray MultiObjectTracker::get_tracks(
    uint64_t timestamp, const std::string& frame_id)
{
  vision_servo_msgs::msg::TargetArray result;
  result.header.stamp.sec = static_cast<int32_t>(timestamp / kNanosecondsPerSecond);
  result.header.stamp.nanosec = static_cast<uint32_t>(timestamp % kNanosecondsPerSecond);
  result.header.frame_id = frame_id;

  for (const auto& [id, track] : tracks_) {
    (void)id;
    if (track.confirmed) {
      auto target = target_from_track(track);
      target.header = result.header;
      result.targets.push_back(target);
    }
  }
  return result;
}

float MultiObjectTracker::compute_iou(
    const vision_servo_msgs::msg::Target& a,
    const vision_servo_msgs::msg::Target& b)
{
  const float x1 = std::max(a.bbox[0], b.bbox[0]);
  const float y1 = std::max(a.bbox[1], b.bbox[1]);
  const float x2 = std::min(a.bbox[2], b.bbox[2]);
  const float y2 = std::min(a.bbox[3], b.bbox[3]);
  const float intersection = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);

  const float area_a =
      std::max(0.0f, a.bbox[2] - a.bbox[0]) * std::max(0.0f, a.bbox[3] - a.bbox[1]);
  const float area_b =
      std::max(0.0f, b.bbox[2] - b.bbox[0]) * std::max(0.0f, b.bbox[3] - b.bbox[1]);
  const float union_area = area_a + area_b - intersection;
  if (union_area <= 1e-6f) {
    return 0.0f;
  }
  return intersection / union_area;
}

void MultiObjectTracker::associate_detections_to_tracks(
    const vision_servo_msgs::msg::TargetArray& detections,
    std::map<int, int>& matches,
    std::set<int>& unmatched_detections,
    std::set<int>& unmatched_tracks)
{
  for (size_t d = 0; d < detections.targets.size(); ++d) {
    unmatched_detections.insert(static_cast<int>(d));
  }
  for (const auto& [track_id, track] : tracks_) {
    (void)track;
    unmatched_tracks.insert(track_id);
  }

  std::vector<int> track_ids;
  std::vector<std::vector<double>> costs;
  track_ids.reserve(tracks_.size());
  costs.reserve(tracks_.size());
  for (const auto& [track_id, track] : tracks_) {
    track_ids.push_back(track_id);
    const auto predicted = target_from_track(track);
    std::vector<double> row(detections.targets.size(), kInvalidAssociationCost);
    for (size_t detection_index = 0;
         detection_index < detections.targets.size(); ++detection_index) {
      const auto& detection = detections.targets[detection_index];
      if (detection.class_name != track.class_name) {
        continue;
      }
      const float iou = compute_iou(detection, predicted);
      if (iou >= iou_threshold_) {
        row[detection_index] = 1.0 - static_cast<double>(iou);
      }
    }
    costs.push_back(std::move(row));
  }

  const std::vector<int> assignment = solve_assignment(costs);
  for (size_t row = 0; row < assignment.size(); ++row) {
    const int detection_index = assignment[row];
    if (detection_index < 0 ||
        costs[row][static_cast<size_t>(detection_index)] >= kInvalidAssociationCost) {
      continue;
    }
    const int track_id = track_ids[row];
    matches[track_id] = detection_index;
    unmatched_tracks.erase(track_id);
    unmatched_detections.erase(detection_index);
  }
}

}  // namespace perception_pkg
