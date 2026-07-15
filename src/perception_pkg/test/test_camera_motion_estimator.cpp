#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "perception_pkg/camera_motion_estimator.hpp"

TEST(CameraMotionEstimator, EstimatesFrameTranslationAtOriginalResolution)
{
  perception_pkg::CameraMotionEstimatorConfig config;
  config.max_width = 320;
  config.max_features = 300;
  config.min_inliers = 10;
  perception_pkg::CameraMotionEstimator estimator(config);

  cv::Mat first = cv::Mat::zeros(480, 640, CV_8UC1);
  for (int y = 40; y < first.rows - 40; y += 40) {
    for (int x = 40; x < first.cols - 40; x += 40) {
      cv::circle(first, cv::Point(x, y), 4, cv::Scalar(255), cv::FILLED);
    }
  }
  const cv::Mat transform = (cv::Mat_<double>(2, 3) <<
      1.0, 0.0, 12.0,
      0.0, 1.0, 8.0);
  cv::Mat second;
  cv::warpAffine(first, second, transform, first.size());

  estimator.add_frame(first, 1'000'000'000ULL);
  estimator.add_frame(second, 1'033'000'000ULL);
  const auto motion = estimator.motion_between(
      1'000'000'000ULL, 1'033'000'000ULL);

  ASSERT_TRUE(motion.valid);
  EXPECT_NEAR(motion.affine[0], 1.0F, 0.02F);
  EXPECT_NEAR(motion.affine[4], 1.0F, 0.02F);
  EXPECT_NEAR(motion.affine[2], 12.0F, 1.0F);
  EXPECT_NEAR(motion.affine[5], 8.0F, 1.0F);
}

TEST(CameraMotionEstimator, MissingTimestampReturnsInvalidMotion)
{
  perception_pkg::CameraMotionEstimator estimator;
  cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC1);
  estimator.add_frame(image, 1'000'000'000ULL);
  EXPECT_FALSE(estimator.motion_between(
      1'000'000'000ULL, 2'000'000'000ULL).valid);
}
