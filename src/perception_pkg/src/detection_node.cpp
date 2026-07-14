/**
 * @file detection_node.cpp
 * @brief OpenCV-DNN implementation for YOLOv5/YOLOv8 ONNX exports.
 */

#include "perception_pkg/detection_node.hpp"

#include "perception_pkg/qos.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace perception_pkg {
namespace {

const std::vector<std::string>& coco_labels()
{
  static const std::vector<std::string> labels = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
    "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};
  return labels;
}

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string lowercase(std::string value)
{
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
  return value;
}

rcl_interfaces::msg::ParameterDescriptor parameter_descriptor(
    const std::string& description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor integer_descriptor(
    const std::string& description, int64_t minimum, int64_t maximum)
{
  auto descriptor = parameter_descriptor(description);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = minimum;
  range.to_value = maximum;
  range.step = 1;
  descriptor.integer_range.push_back(range);
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor floating_descriptor(
    const std::string& description, double minimum, double maximum)
{
  auto descriptor = parameter_descriptor(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = minimum;
  range.to_value = maximum;
  range.step = 0.0;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

double percentile_95(std::vector<double> samples)
{
  if (samples.empty()) {
    return -1.0;
  }
  std::sort(samples.begin(), samples.end());
  const size_t index = static_cast<size_t>(
      std::ceil(0.95 * static_cast<double>(samples.size()))) - 1;
  return samples[std::min(index, samples.size() - 1)];
}

}  // namespace

DetectionNode::DetectionNode(const rclcpp::NodeOptions& options)
  : Node("detection_node", options), diagnostics_(this)
{
  declare_parameters();
  validate_parameters();
  load_labels();
  output_parser_ = std::make_unique<YoloOutputParser>(YoloParserConfig{
      labels_, class_filter_, model_type_, confidence_threshold_, nms_threshold_, max_detections_});
  load_model();

  detection_pub_ = create_publisher<vision_servo_msgs::msg::TargetArray>(
      "detections", qos::perception());
  debug_pub_ = image_transport::create_publisher(this, "debug_image");
  image_sub_ = image_transport::create_subscription(
      this, "image",
      std::bind(&DetectionNode::image_callback, this, std::placeholders::_1),
      "raw", qos::image().get_rmw_qos_profile());

  diagnostics_.setHardwareID("yolo_detector");
  diagnostics_.add("image_and_inference", this, &DetectionNode::diagnostic_callback);

  stats_window_start_ = std::chrono::steady_clock::now();
  worker_ = std::thread(&DetectionNode::inference_loop, this);
  RCLCPP_INFO(
      get_logger(),
      "Detector ready: model=%s model_type=%s device=%s input=%dx%d classes=%zu loaded=%s",
      model_path_.c_str(), model_type_.c_str(), device_.c_str(),
      input_width_, input_height_, labels_.size(), model_ready_ ? "true" : "false");
}

DetectionNode::~DetectionNode()
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_worker_ = true;
    pending_image_.reset();
  }
  queue_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void DetectionNode::declare_parameters()
{
  model_path_ = declare_parameter<std::string>(
      "model_path", "",
      parameter_descriptor("Absolute path to a YOLO ONNX model or local TensorRT engine."));
  model_type_ = declare_parameter<std::string>(
      "model_type", "yolov8",
      parameter_descriptor("YOLO output convention: yolov8, yolov5, or auto."));
  labels_path_ = declare_parameter<std::string>(
      "labels_path", "",
      parameter_descriptor("Optional newline-separated class labels; empty uses COCO labels."));
  device_ = declare_parameter<std::string>(
      "device", "cpu",
      parameter_descriptor("Inference backend: cpu, cuda_fp16, cuda_fp32, or tensorrt."));
  input_width_ = declare_parameter<int>(
      "input_width", 640,
      integer_descriptor("Model input width in pixels.", 32, 4096));
  input_height_ = declare_parameter<int>(
      "input_height", 640,
      integer_descriptor("Model input height in pixels.", 32, 4096));
  max_detections_ = declare_parameter<int>(
      "max_detections", 100,
      integer_descriptor("Maximum number of detections published per frame.", 1, 10000));
  warmup_runs_ = declare_parameter<int>(
      "warmup_runs", 1,
      integer_descriptor("Startup forward passes used to validate and warm the model.", 1, 20));
  confidence_threshold_ = static_cast<float>(
      declare_parameter<double>(
          "confidence_threshold", 0.5,
          floating_descriptor("Minimum final detection confidence.", 0.0, 1.0)));
  nms_threshold_ = static_cast<float>(
      declare_parameter<double>(
          "nms_threshold", 0.45,
          floating_descriptor("IoU threshold used by class-aware NMS.", 0.0, 1.0)));
  performance_log_period_ = declare_parameter<double>(
      "performance_log_period", 5.0,
      floating_descriptor("Seconds between inference statistics; 0 disables logging.", 0.0, 3600.0));
  input_timeout_seconds_ = declare_parameter<double>(
      "input_timeout_seconds", 2.0,
      floating_descriptor("Warn when no input image arrives for this many seconds.", 0.1, 3600.0));
  swap_rb_ = declare_parameter<bool>(
      "swap_rb", true,
      parameter_descriptor("Convert OpenCV BGR input to RGB before inference."));
  publish_debug_ = declare_parameter<bool>(
      "publish_debug_image", true,
      parameter_descriptor("Publish an annotated debug image."));
  fail_on_model_error_ = declare_parameter<bool>(
      "fail_on_model_error", true,
      parameter_descriptor("Fail node construction when the model cannot be loaded."));
  class_filter_ = declare_parameter<std::vector<std::string>>(
      "class_names", std::vector<std::string>{"person"},
      parameter_descriptor("Class names to publish; an empty list enables all classes."));
  camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "",
      parameter_descriptor("Optional frame_id override; empty preserves the source header."));
}

void DetectionNode::validate_parameters()
{
  model_type_ = lowercase(trim(model_type_));
  device_ = lowercase(trim(device_));

  if (model_type_ != "yolov8" && model_type_ != "yolov5" && model_type_ != "auto") {
    throw std::invalid_argument("model_type must be yolov8, yolov5, or auto");
  }
  static const std::unordered_set<std::string> supported_devices = {
    "cpu", "cuda", "cuda:0", "cuda_fp16", "cuda_fp32", "tensorrt"};
  if (supported_devices.count(device_) == 0) {
    throw std::invalid_argument(
        "device must be cpu, cuda, cuda:0, cuda_fp16, cuda_fp32, or tensorrt");
  }
  if (input_width_ % 32 != 0 || input_height_ % 32 != 0) {
    throw std::invalid_argument("input_width and input_height must be multiples of 32");
  }
  for (auto& class_name : class_filter_) {
    class_name = trim(class_name);
    if (class_name.empty()) {
      throw std::invalid_argument("class_names must not contain empty strings");
    }
  }
}

void DetectionNode::load_labels()
{
  labels_.clear();
  if (!labels_path_.empty()) {
    std::ifstream input(labels_path_);
    if (!input.is_open()) {
      throw std::runtime_error("Cannot open labels_path: " + labels_path_);
    }
    std::string line;
    while (std::getline(input, line)) {
      line = trim(line);
      if (!line.empty()) {
        labels_.push_back(line);
      }
    }
    if (labels_.empty()) {
      throw std::runtime_error("labels_path does not contain any class names: " + labels_path_);
    }
  } else {
    labels_ = coco_labels();
  }

  std::unordered_set<std::string> unique_labels;
  for (const auto& label : labels_) {
    if (!unique_labels.insert(label).second) {
      throw std::runtime_error("Duplicate class label: " + label);
    }
  }
  for (const auto& class_name : class_filter_) {
    if (unique_labels.count(class_name) == 0) {
      throw std::runtime_error(
          "class_names contains '" + class_name + "', which is absent from the label set");
    }
  }
}

void DetectionNode::load_model()
{
  model_ready_ = false;
  if (model_path_.empty()) {
    handle_model_error("model_path is empty");
    return;
  }

  const std::filesystem::path requested_path(model_path_);
  std::error_code path_error;
  const auto absolute_path = std::filesystem::absolute(requested_path, path_error);
  if (!path_error) {
    model_path_ = absolute_path.lexically_normal().string();
  }
  const std::filesystem::path path(model_path_);
  if (!std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
    handle_model_error("YOLO model does not exist or is not a regular file: " + model_path_);
    return;
  }
  const std::string extension = lowercase(path.extension().string());
  const bool use_tensorrt = device_ == "tensorrt";
  if ((!use_tensorrt && extension != ".onnx") ||
      (use_tensorrt && extension != ".engine" && extension != ".plan")) {
    handle_model_error(
        use_tensorrt
            ? "TensorRT backend requires a locally-built .engine or .plan file: " + model_path_
            : "OpenCV DNN backend requires a .onnx model: " + model_path_);
    return;
  }
  const auto file_size = std::filesystem::file_size(path, path_error);
  if (path_error || file_size == 0) {
    handle_model_error("YOLO model is empty or unreadable: " + model_path_);
    return;
  }

  try {
    const auto load_start = std::chrono::steady_clock::now();
    if (use_tensorrt) {
#ifdef PERCEPTION_HAS_TENSORRT
      tensorrt_backend_ = std::make_unique<TensorRtBackend>(model_path_);
      warmup_model();
      model_ready_ = true;
      const double load_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - load_start).count();
      RCLCPP_INFO(
          get_logger(), "Loaded TensorRT engine input=%s output=%s in %.1f ms",
          tensorrt_backend_->input_shape().c_str(),
          tensorrt_backend_->output_shape().c_str(), load_ms);
      return;
#else
      throw std::runtime_error(
          "This detection_node was built without TensorRT development libraries");
#endif
    }
    net_ = cv::dnn::readNetFromONNX(model_path_);
    if (net_.empty()) {
      throw std::runtime_error("OpenCV returned an empty network");
    }
    output_layer_names_ = net_.getUnconnectedOutLayersNames();
    if (output_layer_names_.empty()) {
      throw std::runtime_error("ONNX network has no output layers");
    }
    configure_backend();
    warmup_model();
    model_ready_ = true;
    const double load_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - load_start).count();
    RCLCPP_INFO(
        get_logger(), "Loaded YOLO model (%.2f MiB) in %.1f ms",
        static_cast<double>(file_size) / (1024.0 * 1024.0), load_ms);
  } catch (const cv::Exception& error) {
    handle_model_error(std::string("OpenCV failed to load/warm YOLO model: ") + error.what());
  } catch (const std::exception& error) {
    handle_model_error(std::string("Failed to load/warm YOLO model: ") + error.what());
  }
}

void DetectionNode::configure_backend()
{
  if (device_ == "cpu") {
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    return;
  }

  const int requested_target = device_ == "cuda_fp32"
      ? cv::dnn::DNN_TARGET_CUDA
      : cv::dnn::DNN_TARGET_CUDA_FP16;
  const auto available_targets = cv::dnn::getAvailableTargets(cv::dnn::DNN_BACKEND_CUDA);
  if (std::find(available_targets.begin(), available_targets.end(), requested_target) ==
      available_targets.end()) {
    throw std::runtime_error(
        "Requested device=" + device_ +
        " but this OpenCV build exposes no matching CUDA DNN target; use cpu or TensorRT");
  }
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
  net_.setPreferableTarget(requested_target);
}

void DetectionNode::warmup_model()
{
  const int dimensions[] = {1, 3, input_height_, input_width_};
  cv::Mat blob(4, dimensions, CV_32F, cv::Scalar(0.0F));
  if (device_ == "tensorrt") {
#ifdef PERCEPTION_HAS_TENSORRT
    cv::Mat output;
    for (int run = 0; run < warmup_runs_; ++run) {
      output = tensorrt_backend_->infer(blob);
    }
    output_parser_->validate_output(output);
    return;
#else
    throw std::runtime_error("TensorRT backend is unavailable in this build");
#endif
  }
  std::vector<cv::Mat> outputs;
  for (int run = 0; run < warmup_runs_; ++run) {
    outputs.clear();
    net_.setInput(blob);
    net_.forward(outputs, output_layer_names_);
    if (outputs.empty()) {
      throw std::runtime_error("ONNX network produced no outputs during warmup");
    }
  }

  output_parser_->validate_output(outputs.front());
  const cv::Mat predictions = output_parser_->reshape_predictions(outputs.front());
  RCLCPP_INFO(
      get_logger(), "Validated ONNX output -> %dx%d predictions",
      predictions.rows, predictions.cols);
}

void DetectionNode::handle_model_error(const std::string& message)
{
  model_ready_ = false;
  if (fail_on_model_error_) {
    throw std::runtime_error(message);
  }
  RCLCPP_ERROR(
      get_logger(), "%s; detector will publish empty results", message.c_str());
}

void DetectionNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  received_frames_.fetch_add(1, std::memory_order_relaxed);
  last_input_steady_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count(),
      std::memory_order_relaxed);
  if (msg->width == 0 || msg->height == 0 || msg->data.empty()) {
    inference_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Discarding empty or zero-sized input image");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (pending_image_) {
      dropped_frames_.fetch_add(1, std::memory_order_relaxed);
    }
    pending_image_ = msg;
  }
  queue_cv_.notify_one();
}

void DetectionNode::inference_loop()
{
  while (true) {
    sensor_msgs::msg::Image::ConstSharedPtr message;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return stop_worker_ || pending_image_; });
      if (stop_worker_) {
        return;
      }
      message = std::move(pending_image_);
      pending_image_.reset();
    }

    try {
      const auto cv_image = cv_bridge::toCvShare(message, "bgr8");
      vision_servo_msgs::msg::TargetArray detections;
      detections.tracking_id = -1;
      if (model_ready_) {
        const auto inference_start = std::chrono::steady_clock::now();
        detections = infer(cv_image->image);
        double end_to_end_ms = -1.0;
        const rclcpp::Time image_stamp(message->header.stamp, get_clock()->get_clock_type());
        const rclcpp::Time current_time = now();
        if (image_stamp.nanoseconds() > 0 && current_time >= image_stamp) {
          end_to_end_ms = static_cast<double>((current_time - image_stamp).nanoseconds()) * 1.0e-6;
        }
        record_inference_time(
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - inference_start).count(),
            end_to_end_ms);
      }
      detections.header = message->header;
      if (!camera_frame_.empty()) {
        detections.header.frame_id = camera_frame_;
      }
      for (auto& target : detections.targets) {
        target.header = detections.header;
      }
      detection_pub_->publish(detections);
      if (publish_debug_ && debug_pub_.getNumSubscribers() > 0) {
        publish_debug_image(cv_image->image, detections.header, detections);
      }
    } catch (const cv_bridge::Exception& error) {
      inference_errors_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "cv_bridge conversion failed: %s", error.what());
    } catch (const cv::Exception& error) {
      inference_errors_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "YOLO inference failed: %s", error.what());
    } catch (const std::exception& error) {
      inference_errors_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "Detector failed: %s", error.what());
    }
  }
}

vision_servo_msgs::msg::TargetArray DetectionNode::infer(const cv::Mat& frame)
{
  vision_servo_msgs::msg::TargetArray result;
  result.tracking_id = -1;
  if (frame.empty()) {
    return result;
  }

  LetterboxTransform transform;
  cv::Mat blob = YoloPreprocessor::make_blob(
      frame, input_width_, input_height_, swap_rb_, transform);
  if (device_ == "tensorrt") {
#ifdef PERCEPTION_HAS_TENSORRT
    return output_parser_->parse(tensorrt_backend_->infer(blob), transform);
#else
    throw std::runtime_error("TensorRT backend is unavailable in this build");
#endif
  }
  net_.setInput(blob);

  std::vector<cv::Mat> outputs;
  net_.forward(outputs, output_layer_names_);
  if (outputs.empty()) {
    throw std::runtime_error("YOLO network produced no outputs");
  }

  return output_parser_->parse(outputs.front(), transform);
}

void DetectionNode::record_inference_time(double elapsed_ms, double end_to_end_ms)
{
  if (performance_log_period_ <= 0.0) {
    return;
  }
  ++stats_frame_count_;
  stats_total_inference_ms_ += elapsed_ms;
  stats_max_inference_ms_ = std::max(stats_max_inference_ms_, elapsed_ms);
  stats_inference_samples_ms_.push_back(elapsed_ms);
  if (end_to_end_ms >= 0.0) {
    ++stats_latency_count_;
    stats_total_end_to_end_ms_ += end_to_end_ms;
    stats_max_end_to_end_ms_ = std::max(stats_max_end_to_end_ms_, end_to_end_ms);
    stats_end_to_end_samples_ms_.push_back(end_to_end_ms);
  }

  const auto now = std::chrono::steady_clock::now();
  const double window_seconds =
      std::chrono::duration<double>(now - stats_window_start_).count();
  if (window_seconds < performance_log_period_) {
    return;
  }

  const double average_ms = stats_frame_count_ > 0
      ? stats_total_inference_ms_ / static_cast<double>(stats_frame_count_)
      : 0.0;
  const double processed_fps = window_seconds > 0.0
      ? static_cast<double>(stats_frame_count_) / window_seconds
      : 0.0;
  const double average_end_to_end_ms = stats_latency_count_ > 0
      ? stats_total_end_to_end_ms_ / static_cast<double>(stats_latency_count_)
      : -1.0;
  const double inference_p95_ms = percentile_95(stats_inference_samples_ms_);
  const double end_to_end_p95_ms = percentile_95(stats_end_to_end_samples_ms_);
  RCLCPP_INFO(
      get_logger(),
      "YOLO performance: processed=%.1f FPS inference_avg=%.1f ms "
      "inference_p95=%.1f ms inference_max=%.1f ms end_to_end_avg=%.1f ms "
      "end_to_end_p95=%.1f ms end_to_end_max=%.1f ms "
      "dropped_total=%llu",
      processed_fps, average_ms, inference_p95_ms, stats_max_inference_ms_,
      average_end_to_end_ms, end_to_end_p95_ms, stats_max_end_to_end_ms_,
      static_cast<unsigned long long>(dropped_frames_.load(std::memory_order_relaxed)));

  stats_window_start_ = now;
  stats_frame_count_ = 0;
  stats_total_inference_ms_ = 0.0;
  stats_max_inference_ms_ = 0.0;
  stats_inference_samples_ms_.clear();
  stats_latency_count_ = 0;
  stats_total_end_to_end_ms_ = 0.0;
  stats_max_end_to_end_ms_ = 0.0;
  stats_end_to_end_samples_ms_.clear();
}

void DetectionNode::diagnostic_callback(
    diagnostic_updater::DiagnosticStatusWrapper& status)
{
  const int64_t last_input_ns = last_input_steady_ns_.load(std::memory_order_relaxed);
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  const double input_age = last_input_ns > 0
      ? static_cast<double>(now_ns - last_input_ns) * 1.0e-9
      : std::numeric_limits<double>::infinity();

  if (!model_ready_) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Model is not ready");
  } else if (input_age > input_timeout_seconds_) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Input image timeout");
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Detector is receiving images");
  }
  status.add("model_ready", model_ready_);
  status.add("backend", device_);
  status.add("received_frames", received_frames_.load(std::memory_order_relaxed));
  status.add("dropped_frames", dropped_frames_.load(std::memory_order_relaxed));
  status.add("inference_errors", inference_errors_.load(std::memory_order_relaxed));
  status.add("input_age_seconds", std::isfinite(input_age) ? input_age : -1.0);
  status.add("debug_subscribers", debug_pub_.getNumSubscribers());
}

void DetectionNode::publish_debug_image(
    const cv::Mat& frame,
    const std_msgs::msg::Header& header,
    const vision_servo_msgs::msg::TargetArray& detections)
{
  cv::Mat annotated = frame.clone();
  for (const auto& target : detections.targets) {
    const cv::Point top_left(
        static_cast<int>(std::round(target.bbox[0])),
        static_cast<int>(std::round(target.bbox[1])));
    const cv::Point bottom_right(
        static_cast<int>(std::round(target.bbox[2])),
        static_cast<int>(std::round(target.bbox[3])));
    cv::rectangle(annotated, top_left, bottom_right, cv::Scalar(0, 220, 0), 2);
    std::ostringstream label;
    label << target.class_name << ' ' << std::fixed << std::setprecision(2)
          << target.confidence;
    cv::putText(
        annotated, label.str(), cv::Point(top_left.x, std::max(15, top_left.y - 6)),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 220, 0), 1, cv::LINE_AA);
  }
  const auto debug_message = cv_bridge::CvImage(header, "bgr8", annotated).toImageMsg();
  debug_pub_.publish(*debug_message);
}

}  // namespace perception_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_pkg::DetectionNode>());
  rclcpp::shutdown();
  return 0;
}
