/**
 * @file multi_object_tracker.cpp
 * @brief MultiObjectTracker core implementation shared by standalone and composable perception paths.
 */

#include "perception_pkg/tracking_node.hpp"

#include <algorithm>
#include <cstdint>

namespace perception_pkg {
namespace {

constexpr uint64_t kNanosecondsPerSecond = 1000000000ULL;

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
  const auto& state = track.kf.statePost;
  const float cx = state.at<float>(0);
  const float cy = state.at<float>(1);
  const float width = std::max(1.0f, state.at<float>(2));
  const float height = std::max(1.0f, state.at<float>(3));
  const float half_w = width * 0.5f;
  const float half_h = height * 0.5f;

  vision_servo_msgs::msg::Target target;
  target.id = track.id;
  target.class_name = track.class_name;
  target.confidence = track.max_confidence;
  target.center = {cx, cy};
  target.bbox = {cx - half_w, cy - half_h, cx + half_w, cy + half_h};
  target.width = width;
  target.height = height;
  return target;
}

}  // namespace

MultiObjectTracker::MultiObjectTracker(int max_age, int min_hits, float iou_threshold)
  : next_id_(0), max_age_(max_age), min_hits_(min_hits), iou_threshold_(iou_threshold)
{}

void MultiObjectTracker::predict()
{
  for (auto& [id, track] : tracks_) {
    (void)id;
    track.kf.predict();
    track.age++;
    track.consecutive_invisible_count++;
  }
}

void MultiObjectTracker::update(const vision_servo_msgs::msg::TargetArray& detections)
{
  predict();

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
    track.total_visible_count++;

    if (det.confidence >= track.max_confidence) {
      track.max_confidence = det.confidence;
      track.class_name = det.class_name;
    }
  }

  for (int det_idx : unmatched_detections) {
    const auto& det = detections.targets[det_idx];
    Track track;
    track.id = next_id_++;
    track.class_name = det.class_name;
    track.age = 0;
    track.total_visible_count = 1;
    track.consecutive_invisible_count = 0;
    track.max_confidence = det.confidence;
    initialize_track_filter(track, det);
    tracks_[track.id] = track;
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
    if (track.total_visible_count >= min_hits_) {
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

  for (const auto& [track_id, track] : tracks_) {
    float best_iou = iou_threshold_;
    int best_det = -1;
    const auto predicted = target_from_track(track);

    for (int det_idx : unmatched_detections) {
      const float iou = compute_iou(detections.targets[det_idx], predicted);
      if (iou > best_iou) {
        best_iou = iou;
        best_det = det_idx;
      }
    }

    if (best_det >= 0) {
      matches[track_id] = best_det;
      unmatched_tracks.erase(track_id);
      unmatched_detections.erase(best_det);
    }
  }
}

}  // namespace perception_pkg
