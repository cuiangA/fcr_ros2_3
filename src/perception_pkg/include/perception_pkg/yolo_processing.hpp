/**
 * @file yolo_processing.hpp
 * @brief ROS-independent YOLO image preprocessing and output parsing.
 */

#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

namespace perception_pkg {

struct LetterboxTransform {
  float scale = 1.0F;
  int pad_x = 0;
  int pad_y = 0;
  int source_width = 0;
  int source_height = 0;
  int input_width = 0;
  int input_height = 0;
};

class YoloPreprocessor {
public:
  static cv::Mat make_blob(
      const cv::Mat& source,
      int input_width,
      int input_height,
      bool swap_rb,
      LetterboxTransform& transform);
};

struct YoloParserConfig {
  std::vector<std::string> labels;
  std::vector<std::string> class_filter;
  std::string model_type = "yolov8";
  float confidence_threshold = 0.5F;
  float nms_threshold = 0.45F;
  int max_detections = 100;
};

class YoloOutputParser {
public:
  explicit YoloOutputParser(YoloParserConfig config);

  cv::Mat reshape_predictions(const cv::Mat& raw_output) const;
  void validate_output(const cv::Mat& raw_output) const;
  vision_servo_msgs::msg::TargetArray parse(
      const cv::Mat& raw_output,
      const LetterboxTransform& transform) const;

private:
  bool has_objectness(int prediction_columns) const;
  bool class_is_enabled(const std::string& class_name) const;
  std::vector<int> class_aware_nms(
      const std::vector<cv::Rect>& boxes,
      const std::vector<float>& scores,
      const std::vector<int>& class_ids) const;

  YoloParserConfig config_;
  std::unordered_set<std::string> enabled_classes_;
};

}  // namespace perception_pkg
