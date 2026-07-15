#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

#include <opencv2/core.hpp>

#include "perception_pkg/tracker_interface.hpp"

namespace perception_pkg {

struct CameraMotionEstimatorConfig {
  int max_width = 320;
  int max_features = 200;
  int min_inliers = 20;
  double quality_level = 0.01;
  double minimum_feature_distance = 8.0;
  double ransac_reprojection_threshold = 2.0;
  std::size_t history_size = 120;
};

/**
 * Estimates frame-to-frame camera motion from image background features.
 *
 * The returned affine transform maps pixel coordinates from the older image
 * into the newer image.  It is an observation-only helper: failures simply
 * produce an invalid motion so tracking can continue with its Kalman model.
 */
class CameraMotionEstimator {
public:
  explicit CameraMotionEstimator(CameraMotionEstimatorConfig config = {});

  void add_frame(const cv::Mat& bgr_or_gray, uint64_t timestamp_ns);
  CameraMotion motion_between(uint64_t from_timestamp_ns, uint64_t to_timestamp_ns) const;

private:
  struct MotionSample {
    uint64_t timestamp_ns = 0;
    cv::Matx33f cumulative = cv::Matx33f::eye();
    uint64_t continuity_segment = 0;
  };

  static bool valid_affine(const cv::Mat& affine);
  const MotionSample* find_sample(uint64_t timestamp_ns) const;

  CameraMotionEstimatorConfig config_;
  mutable std::mutex mutex_;
  cv::Mat previous_gray_;
  uint64_t previous_timestamp_ns_ = 0;
  cv::Matx33f cumulative_motion_ = cv::Matx33f::eye();
  uint64_t continuity_segment_ = 0;
  std::deque<MotionSample> history_;
};

}  // namespace perception_pkg
