#include "perception_pkg/yolo_processing.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace perception_pkg {
namespace {

std::string tensor_shape(const cv::Mat& tensor)
{
  std::ostringstream stream;
  stream << '[';
  for (int dimension = 0; dimension < tensor.dims; ++dimension) {
    if (dimension > 0) {
      stream << ',';
    }
    stream << tensor.size[dimension];
  }
  stream << ']';
  return stream.str();
}

}  // namespace

cv::Mat YoloPreprocessor::make_blob(
    const cv::Mat& source,
    int input_width,
    int input_height,
    bool swap_rb,
    LetterboxTransform& transform)
{
  if (source.empty() || source.cols <= 0 || source.rows <= 0) {
    throw std::invalid_argument("YOLO source image must be non-empty");
  }
  if (source.type() != CV_8UC3) {
    throw std::invalid_argument("YOLO source image must use 8-bit BGR encoding");
  }
  if (input_width <= 0 || input_height <= 0) {
    throw std::invalid_argument("YOLO input dimensions must be positive");
  }

  transform.scale = std::min(
      static_cast<float>(input_width) / static_cast<float>(source.cols),
      static_cast<float>(input_height) / static_cast<float>(source.rows));
  const int resized_width = std::max(
      1, static_cast<int>(std::round(static_cast<float>(source.cols) * transform.scale)));
  const int resized_height = std::max(
      1, static_cast<int>(std::round(static_cast<float>(source.rows) * transform.scale)));
  transform.pad_x = (input_width - resized_width) / 2;
  transform.pad_y = (input_height - resized_height) / 2;
  transform.source_width = source.cols;
  transform.source_height = source.rows;
  transform.input_width = input_width;
  transform.input_height = input_height;

  cv::Mat resized;
  cv::resize(source, resized, cv::Size(resized_width, resized_height));
  cv::Mat letterboxed(input_height, input_width, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(letterboxed(cv::Rect(
      transform.pad_x, transform.pad_y, resized_width, resized_height)));

  cv::Mat blob;
  cv::dnn::blobFromImage(
      letterboxed, blob, 1.0 / 255.0, cv::Size(input_width, input_height),
      cv::Scalar(), swap_rb, false, CV_32F);
  return blob;
}

YoloOutputParser::YoloOutputParser(YoloParserConfig config)
  : config_(std::move(config)),
    enabled_classes_(config_.class_filter.begin(), config_.class_filter.end())
{
  if (config_.labels.empty()) {
    throw std::invalid_argument("YOLO labels must not be empty");
  }
  if (config_.model_type != "yolov8" && config_.model_type != "yolov5" &&
      config_.model_type != "auto") {
    throw std::invalid_argument("YOLO model_type must be yolov8, yolov5, or auto");
  }
  if (config_.confidence_threshold < 0.0F || config_.confidence_threshold > 1.0F ||
      config_.nms_threshold < 0.0F || config_.nms_threshold > 1.0F) {
    throw std::invalid_argument("YOLO thresholds must be in [0, 1]");
  }
  if (config_.max_detections < 1) {
    throw std::invalid_argument("YOLO max_detections must be positive");
  }
}

bool YoloOutputParser::has_objectness(int prediction_columns) const
{
  return config_.model_type == "yolov5" ||
      (config_.model_type == "auto" &&
       prediction_columns == static_cast<int>(config_.labels.size()) + 5);
}

cv::Mat YoloOutputParser::reshape_predictions(const cv::Mat& raw_output) const
{
  if (raw_output.empty()) {
    throw std::runtime_error("YOLO output tensor is empty");
  }

  cv::Mat output;
  if (raw_output.type() == CV_32F) {
    output = raw_output;
  } else {
    raw_output.convertTo(output, CV_32F);
  }
  if (!output.isContinuous()) {
    output = output.clone();
  }

  cv::Mat rows;
  const int v8_columns = static_cast<int>(config_.labels.size()) + 4;
  const int v5_columns = static_cast<int>(config_.labels.size()) + 5;
  if (output.dims == 3) {
    if (output.size[0] != 1) {
      throw std::runtime_error(
          "Only batch size 1 is supported, received " + tensor_shape(output));
    }
    const int first = output.size[1];
    const int second = output.size[2];
    const bool first_is_columns = first == v8_columns || first == v5_columns;
    const bool second_is_columns = second == v8_columns || second == v5_columns;
    cv::Mat matrix = output.reshape(1, first);
    if ((first_is_columns && !second_is_columns) ||
        (!first_is_columns && !second_is_columns && first < second)) {
      cv::transpose(matrix, rows);
    } else {
      rows = matrix;
    }
  } else if (output.dims == 2) {
    const bool rows_are_columns = output.rows == v8_columns || output.rows == v5_columns;
    const bool cols_are_columns = output.cols == v8_columns || output.cols == v5_columns;
    if ((rows_are_columns && !cols_are_columns) ||
        (!rows_are_columns && !cols_are_columns && output.rows < output.cols)) {
      cv::transpose(output, rows);
    } else {
      rows = output;
    }
  } else {
    throw std::runtime_error(
        "Unsupported YOLO output rank/shape: " + tensor_shape(output));
  }

  if (rows.rows <= 0 || rows.cols < 5) {
    throw std::runtime_error(
        "Invalid YOLO prediction matrix: " +
        std::to_string(rows.rows) + "x" + std::to_string(rows.cols));
  }
  return rows;
}

void YoloOutputParser::validate_output(const cv::Mat& raw_output) const
{
  const cv::Mat predictions = reshape_predictions(raw_output);
  const int class_count = predictions.cols - (has_objectness(predictions.cols) ? 5 : 4);
  if (class_count != static_cast<int>(config_.labels.size())) {
    throw std::runtime_error(
        "YOLO output contains " + std::to_string(class_count) +
        " classes but labels contain " + std::to_string(config_.labels.size()));
  }
}

vision_servo_msgs::msg::TargetArray YoloOutputParser::parse(
    const cv::Mat& raw_output,
    const LetterboxTransform& transform) const
{
  vision_servo_msgs::msg::TargetArray result;
  result.tracking_id = -1;
  if (raw_output.empty()) {
    return result;
  }
  if (transform.scale <= 0.0F || transform.source_width <= 0 ||
      transform.source_height <= 0) {
    throw std::invalid_argument("Invalid letterbox transform");
  }

  const cv::Mat rows = reshape_predictions(raw_output);
  const bool objectness_present = has_objectness(rows.cols);
  const int class_offset = objectness_present ? 5 : 4;
  const int class_count = rows.cols - class_offset;
  if (class_count != static_cast<int>(config_.labels.size())) {
    throw std::runtime_error("YOLO output class count does not match configured labels");
  }

  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;
  boxes.reserve(static_cast<size_t>(rows.rows));
  scores.reserve(static_cast<size_t>(rows.rows));
  class_ids.reserve(static_cast<size_t>(rows.rows));

  for (int row_index = 0; row_index < rows.rows; ++row_index) {
    const float* data = rows.ptr<float>(row_index);
    if (!std::isfinite(data[0]) || !std::isfinite(data[1]) ||
        !std::isfinite(data[2]) || !std::isfinite(data[3])) {
      continue;
    }
    const float objectness = objectness_present ? data[4] : 1.0F;
    if (!std::isfinite(objectness) || objectness < config_.confidence_threshold) {
      continue;
    }

    cv::Mat class_scores(1, class_count, CV_32F, const_cast<float*>(data + class_offset));
    cv::Point best_class;
    double best_score = 0.0;
    cv::minMaxLoc(class_scores, nullptr, &best_score, nullptr, &best_class);
    const float confidence = objectness * static_cast<float>(best_score);
    if (!std::isfinite(confidence) || confidence < config_.confidence_threshold) {
      continue;
    }
    const int class_id = best_class.x;
    const std::string& class_name = config_.labels.at(static_cast<size_t>(class_id));
    if (!class_is_enabled(class_name)) {
      continue;
    }

    const float center_x = (data[0] - static_cast<float>(transform.pad_x)) / transform.scale;
    const float center_y = (data[1] - static_cast<float>(transform.pad_y)) / transform.scale;
    const float width = data[2] / transform.scale;
    const float height = data[3] / transform.scale;
    if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
        !std::isfinite(width) || !std::isfinite(height) || width <= 0.0F || height <= 0.0F) {
      continue;
    }

    cv::Rect box(
        static_cast<int>(std::round(center_x - width * 0.5F)),
        static_cast<int>(std::round(center_y - height * 0.5F)),
        static_cast<int>(std::round(width)),
        static_cast<int>(std::round(height)));
    box &= cv::Rect(0, 0, transform.source_width, transform.source_height);
    if (box.width <= 1 || box.height <= 1) {
      continue;
    }
    boxes.push_back(box);
    scores.push_back(confidence);
    class_ids.push_back(class_id);
  }

  std::vector<int> kept_indices = class_aware_nms(boxes, scores, class_ids);
  if (static_cast<int>(kept_indices.size()) > config_.max_detections) {
    kept_indices.resize(static_cast<size_t>(config_.max_detections));
  }
  result.targets.reserve(kept_indices.size());
  for (const int index : kept_indices) {
    const cv::Rect& box = boxes.at(static_cast<size_t>(index));
    vision_servo_msgs::msg::Target target;
    target.id = -1;
    target.tracking_state = vision_servo_msgs::msg::Target::TRACKING_STATE_UNTRACKED;
    target.visible = true;
    target.class_name = config_.labels.at(static_cast<size_t>(class_ids.at(index)));
    target.confidence = scores.at(static_cast<size_t>(index));
    target.bbox = {
      static_cast<float>(box.x), static_cast<float>(box.y),
      static_cast<float>(box.x + box.width), static_cast<float>(box.y + box.height)};
    target.center = {
      static_cast<float>(box.x) + static_cast<float>(box.width) * 0.5F,
      static_cast<float>(box.y) + static_cast<float>(box.height) * 0.5F};
    target.width = static_cast<float>(box.width);
    target.height = static_cast<float>(box.height);
    target.depth_confidence = 0.0F;
    result.targets.push_back(std::move(target));
  }
  return result;
}

bool YoloOutputParser::class_is_enabled(const std::string& class_name) const
{
  return enabled_classes_.empty() || enabled_classes_.count(class_name) > 0;
}

std::vector<int> YoloOutputParser::class_aware_nms(
    const std::vector<cv::Rect>& boxes,
    const std::vector<float>& scores,
    const std::vector<int>& class_ids) const
{
  if (boxes.size() != scores.size() || boxes.size() != class_ids.size()) {
    throw std::invalid_argument("NMS input arrays must have equal lengths");
  }
  std::unordered_map<int, std::vector<int>> indices_by_class;
  for (size_t index = 0; index < class_ids.size(); ++index) {
    indices_by_class[class_ids[index]].push_back(static_cast<int>(index));
  }

  std::vector<int> kept_indices;
  for (const auto& [class_id, global_indices] : indices_by_class) {
    (void)class_id;
    std::vector<cv::Rect> class_boxes;
    std::vector<float> class_scores;
    for (const int global_index : global_indices) {
      class_boxes.push_back(boxes.at(static_cast<size_t>(global_index)));
      class_scores.push_back(scores.at(static_cast<size_t>(global_index)));
    }
    std::vector<int> local_kept;
    cv::dnn::NMSBoxes(
        class_boxes, class_scores, config_.confidence_threshold,
        config_.nms_threshold, local_kept);
    for (const int local_index : local_kept) {
      kept_indices.push_back(global_indices.at(static_cast<size_t>(local_index)));
    }
  }
  std::sort(
      kept_indices.begin(), kept_indices.end(),
      [&scores](int left, int right) {
        const float left_score = scores.at(left);
        const float right_score = scores.at(right);
        return left_score == right_score ? left < right : left_score > right_score;
      });
  return kept_indices;
}

}  // namespace perception_pkg
