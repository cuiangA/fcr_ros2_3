#include "perception_pkg/camera_motion_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace perception_pkg {
namespace {

constexpr uint64_t kTimestampToleranceNs = 5'000'000ULL;

cv::Matx33f affine_to_homogeneous(const cv::Mat& affine, float inverse_scale)
{
  return cv::Matx33f(
      static_cast<float>(affine.at<double>(0, 0)),
      static_cast<float>(affine.at<double>(0, 1)),
      static_cast<float>(affine.at<double>(0, 2)) * inverse_scale,
      static_cast<float>(affine.at<double>(1, 0)),
      static_cast<float>(affine.at<double>(1, 1)),
      static_cast<float>(affine.at<double>(1, 2)) * inverse_scale,
      0.0F, 0.0F, 1.0F);
}

}  // namespace

CameraMotionEstimator::CameraMotionEstimator(CameraMotionEstimatorConfig config)
  : config_(std::move(config))
{
  if (config_.max_width < 32 || config_.max_features < 4 ||
      config_.min_inliers < 3 || config_.min_inliers > config_.max_features ||
      !std::isfinite(config_.quality_level) || config_.quality_level <= 0.0 ||
      config_.quality_level > 1.0 ||
      !std::isfinite(config_.minimum_feature_distance) ||
      config_.minimum_feature_distance <= 0.0 ||
      !std::isfinite(config_.ransac_reprojection_threshold) ||
      config_.ransac_reprojection_threshold <= 0.0 || config_.history_size < 2) {
    throw std::invalid_argument("invalid camera motion estimator configuration");
  }
}

void CameraMotionEstimator::add_frame(
    const cv::Mat& bgr_or_gray, uint64_t timestamp_ns)
{
  if (bgr_or_gray.empty() || timestamp_ns == 0) {
    return;
  }

  cv::Mat gray;
  if (bgr_or_gray.channels() == 1) {
    gray = bgr_or_gray;
  } else if (bgr_or_gray.channels() == 3) {
    cv::cvtColor(bgr_or_gray, gray, cv::COLOR_BGR2GRAY);
  } else if (bgr_or_gray.channels() == 4) {
    cv::cvtColor(bgr_or_gray, gray, cv::COLOR_BGRA2GRAY);
  } else {
    return;
  }

  const double resize_scale = gray.cols > config_.max_width
      ? static_cast<double>(config_.max_width) / static_cast<double>(gray.cols)
      : 1.0;
  if (resize_scale < 1.0) {
    cv::resize(gray, gray, cv::Size(), resize_scale, resize_scale, cv::INTER_AREA);
  } else if (!gray.isContinuous()) {
    gray = gray.clone();
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (previous_gray_.empty() || previous_gray_.size() != gray.size() ||
      timestamp_ns <= previous_timestamp_ns_) {
    previous_gray_ = gray.clone();
    previous_timestamp_ns_ = timestamp_ns;
    cumulative_motion_ = cv::Matx33f::eye();
    ++continuity_segment_;
    history_.clear();
    history_.push_back({timestamp_ns, cumulative_motion_, continuity_segment_});
    return;
  }

  cv::Matx33f frame_motion = cv::Matx33f::eye();
  bool frame_motion_valid = false;
  std::vector<cv::Point2f> previous_points;
  cv::goodFeaturesToTrack(
      previous_gray_, previous_points, config_.max_features,
      config_.quality_level, config_.minimum_feature_distance);
  if (previous_points.size() >= static_cast<std::size_t>(config_.min_inliers)) {
    std::vector<cv::Point2f> current_points;
    std::vector<unsigned char> status;
    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(
        previous_gray_, gray, previous_points, current_points, status, errors);

    std::vector<cv::Point2f> valid_previous;
    std::vector<cv::Point2f> valid_current;
    valid_previous.reserve(previous_points.size());
    valid_current.reserve(previous_points.size());
    for (std::size_t index = 0; index < status.size(); ++index) {
      if (status[index] != 0 && std::isfinite(current_points[index].x) &&
          std::isfinite(current_points[index].y)) {
        valid_previous.push_back(previous_points[index]);
        valid_current.push_back(current_points[index]);
      }
    }

    if (valid_previous.size() >= static_cast<std::size_t>(config_.min_inliers)) {
      cv::Mat inlier_mask;
      const cv::Mat affine = cv::estimateAffinePartial2D(
          valid_previous, valid_current, inlier_mask, cv::RANSAC,
          config_.ransac_reprojection_threshold);
      const int inlier_count = inlier_mask.empty() ? 0 : cv::countNonZero(inlier_mask);
      if (inlier_count >= config_.min_inliers && valid_affine(affine)) {
        frame_motion = affine_to_homogeneous(
            affine, static_cast<float>(1.0 / resize_scale));
        frame_motion_valid = true;
      }
    }
  }

  if (frame_motion_valid) {
    cumulative_motion_ = frame_motion * cumulative_motion_;
  } else {
    ++continuity_segment_;
  }
  previous_gray_ = gray.clone();
  previous_timestamp_ns_ = timestamp_ns;
  history_.push_back({timestamp_ns, cumulative_motion_, continuity_segment_});
  while (history_.size() > config_.history_size) {
    history_.pop_front();
  }
}

CameraMotion CameraMotionEstimator::motion_between(
    uint64_t from_timestamp_ns, uint64_t to_timestamp_ns) const
{
  CameraMotion result;
  if (from_timestamp_ns == 0 || to_timestamp_ns <= from_timestamp_ns) {
    return result;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const MotionSample* from = find_sample(from_timestamp_ns);
  const MotionSample* to = find_sample(to_timestamp_ns);
  if (from == nullptr || to == nullptr ||
      from->continuity_segment != to->continuity_segment) {
    return result;
  }
  const cv::Matx33f relative = to->cumulative * from->cumulative.inv();
  for (float value : relative.val) {
    if (!std::isfinite(value)) {
      return result;
    }
  }
  result.valid = true;
  result.affine = {
      relative(0, 0), relative(0, 1), relative(0, 2),
      relative(1, 0), relative(1, 1), relative(1, 2)};
  return result;
}

bool CameraMotionEstimator::valid_affine(const cv::Mat& affine)
{
  if (affine.rows != 2 || affine.cols != 3 || affine.type() != CV_64F) {
    return false;
  }
  for (int row = 0; row < affine.rows; ++row) {
    for (int column = 0; column < affine.cols; ++column) {
      if (!std::isfinite(affine.at<double>(row, column))) {
        return false;
      }
    }
  }
  const double scale = std::hypot(
      affine.at<double>(0, 0), affine.at<double>(1, 0));
  const double rotation = std::atan2(
      affine.at<double>(1, 0), affine.at<double>(0, 0));
  return scale >= 0.8 && scale <= 1.2 && std::abs(rotation) <= 0.52;
}

const CameraMotionEstimator::MotionSample* CameraMotionEstimator::find_sample(
    uint64_t timestamp_ns) const
{
  const MotionSample* nearest = nullptr;
  uint64_t nearest_delta = std::numeric_limits<uint64_t>::max();
  for (const auto& sample : history_) {
    const uint64_t delta = sample.timestamp_ns > timestamp_ns
        ? sample.timestamp_ns - timestamp_ns : timestamp_ns - sample.timestamp_ns;
    if (delta < nearest_delta) {
      nearest = &sample;
      nearest_delta = delta;
    }
  }
  return nearest_delta <= kTimestampToleranceNs ? nearest : nullptr;
}

}  // namespace perception_pkg
