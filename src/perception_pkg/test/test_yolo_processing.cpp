#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "perception_pkg/yolo_processing.hpp"

namespace {

using perception_pkg::LetterboxTransform;
using perception_pkg::YoloOutputParser;
using perception_pkg::YoloParserConfig;
using perception_pkg::YoloPreprocessor;

YoloOutputParser make_parser(
    std::vector<std::string> class_filter = {}, int max_detections = 100)
{
  return YoloOutputParser(YoloParserConfig{
      {"person", "dog"}, std::move(class_filter), "yolov8", 0.5F, 0.45F,
      max_detections});
}

LetterboxTransform landscape_transform()
{
  LetterboxTransform transform;
  transform.scale = 0.5F;
  transform.pad_y = 140;
  transform.source_width = 1280;
  transform.source_height = 720;
  transform.input_width = 640;
  transform.input_height = 640;
  return transform;
}

cv::Mat output_with_candidates(int count)
{
  const int dimensions[] = {1, 6, count};
  return cv::Mat(3, dimensions, CV_32F, cv::Scalar(0.0F));
}

void set_candidate(
    cv::Mat& output, int candidate, float cx, float cy, float width, float height,
    float person_score, float dog_score)
{
  output.at<float>(0, 0, candidate) = cx;
  output.at<float>(0, 1, candidate) = cy;
  output.at<float>(0, 2, candidate) = width;
  output.at<float>(0, 3, candidate) = height;
  output.at<float>(0, 4, candidate) = person_score;
  output.at<float>(0, 5, candidate) = dog_score;
}

TEST(YoloPreprocessor, LandscapeLetterboxTransform)
{
  const cv::Mat image(720, 1280, CV_8UC3, cv::Scalar(1, 2, 3));
  LetterboxTransform transform;
  const cv::Mat blob = YoloPreprocessor::make_blob(image, 640, 640, true, transform);
  EXPECT_EQ(blob.dims, 4);
  EXPECT_EQ(blob.size[2], 640);
  EXPECT_EQ(blob.size[3], 640);
  EXPECT_FLOAT_EQ(transform.scale, 0.5F);
  EXPECT_EQ(transform.pad_x, 0);
  EXPECT_EQ(transform.pad_y, 140);
}

TEST(YoloPreprocessor, PortraitLetterboxTransform)
{
  const cv::Mat image(1280, 720, CV_8UC3, cv::Scalar(1, 2, 3));
  LetterboxTransform transform;
  YoloPreprocessor::make_blob(image, 640, 640, true, transform);
  EXPECT_FLOAT_EQ(transform.scale, 0.5F);
  EXPECT_EQ(transform.pad_x, 140);
  EXPECT_EQ(transform.pad_y, 0);
}

TEST(YoloOutputParser, RestoresAndClipsCoordinates)
{
  auto output = output_with_candidates(1);
  set_candidate(output, 0, 10.0F, 320.0F, 100.0F, 200.0F, 0.9F, 0.1F);
  const auto targets = make_parser().parse(output, landscape_transform());
  ASSERT_EQ(targets.targets.size(), 1U);
  EXPECT_FLOAT_EQ(targets.targets.front().bbox[0], 0.0F);
  EXPECT_FLOAT_EQ(targets.targets.front().bbox[1], 160.0F);
  EXPECT_FLOAT_EQ(targets.targets.front().bbox[2], 120.0F);
  EXPECT_FLOAT_EQ(targets.targets.front().bbox[3], 560.0F);
}

TEST(YoloOutputParser, SuppressesOverlappingBoxesOfSameClass)
{
  auto output = output_with_candidates(2);
  set_candidate(output, 0, 320.0F, 320.0F, 100.0F, 200.0F, 0.9F, 0.1F);
  set_candidate(output, 1, 322.0F, 320.0F, 100.0F, 200.0F, 0.8F, 0.1F);
  EXPECT_EQ(make_parser().parse(output, landscape_transform()).targets.size(), 1U);
}

TEST(YoloOutputParser, KeepsOverlappingBoxesOfDifferentClasses)
{
  auto output = output_with_candidates(2);
  set_candidate(output, 0, 320.0F, 320.0F, 100.0F, 200.0F, 0.9F, 0.1F);
  set_candidate(output, 1, 320.0F, 320.0F, 100.0F, 200.0F, 0.1F, 0.9F);
  EXPECT_EQ(make_parser().parse(output, landscape_transform()).targets.size(), 2U);
}

TEST(YoloOutputParser, EmptyOutputProducesNoTargets)
{
  EXPECT_TRUE(make_parser().parse(cv::Mat(), landscape_transform()).targets.empty());
}

TEST(YoloOutputParser, RejectsNonFiniteCandidate)
{
  auto output = output_with_candidates(1);
  set_candidate(
      output, 0, std::numeric_limits<float>::quiet_NaN(), 320.0F,
      100.0F, 200.0F, 0.9F, 0.1F);
  EXPECT_TRUE(make_parser().parse(output, landscape_transform()).targets.empty());
}

TEST(YoloOutputParser, AppliesClassFilterAndMaximumCount)
{
  auto output = output_with_candidates(3);
  set_candidate(output, 0, 100.0F, 320.0F, 50.0F, 100.0F, 0.9F, 0.1F);
  set_candidate(output, 1, 300.0F, 320.0F, 50.0F, 100.0F, 0.8F, 0.1F);
  set_candidate(output, 2, 500.0F, 320.0F, 50.0F, 100.0F, 0.1F, 0.99F);
  const auto targets = make_parser({"person"}, 1).parse(output, landscape_transform());
  ASSERT_EQ(targets.targets.size(), 1U);
  EXPECT_EQ(targets.targets.front().class_name, "person");
  EXPECT_FLOAT_EQ(targets.targets.front().confidence, 0.9F);
}

TEST(YoloOutputParser, ReshapesYoloV8ChannelFirstOutput)
{
  std::vector<std::string> labels(80);
  for (size_t index = 0; index < labels.size(); ++index) {
    labels[index] = "class_" + std::to_string(index);
  }
  YoloOutputParser parser(YoloParserConfig{
      labels, {}, "yolov8", 0.5F, 0.45F, 100});
  const int dimensions[] = {1, 84, 8400};
  const cv::Mat output(3, dimensions, CV_32F, cv::Scalar(0.0F));
  const cv::Mat rows = parser.reshape_predictions(output);
  EXPECT_EQ(rows.rows, 8400);
  EXPECT_EQ(rows.cols, 84);
  EXPECT_NO_THROW(parser.validate_output(output));
}

}  // namespace
