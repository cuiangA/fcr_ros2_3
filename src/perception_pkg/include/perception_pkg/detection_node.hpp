/**
 * @file detection_node.hpp
 * @brief Low-latency YOLO ONNX detector for the Sony RGB stream.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/dnn.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

#include "perception_pkg/yolo_processing.hpp"
#ifdef PERCEPTION_HAS_TENSORRT
#include "perception_pkg/tensorrt_backend.hpp"
#endif

namespace perception_pkg {

/**
 * The subscription callback only replaces a one-frame pending buffer. Inference
 * runs on a worker thread, so an overloaded detector drops stale images instead
 * of building latency in the ROS executor.
 */
class DetectionNode : public rclcpp::Node {
public:
  explicit DetectionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DetectionNode() override;

private:
  void declare_parameters();
  void validate_parameters();
  void load_labels();
  void load_model();
  void configure_backend();
  void warmup_model();
  void handle_model_error(const std::string& message);
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void inference_loop();
  vision_servo_msgs::msg::TargetArray infer(const cv::Mat& frame);
  void record_inference_time(double elapsed_ms, double end_to_end_ms);
  void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper& status);
  void publish_debug_image(
      const cv::Mat& frame,
      const std_msgs::msg::Header& header,
      const vision_servo_msgs::msg::TargetArray& detections);

  std::string model_path_;
  std::string model_type_;
  std::string labels_path_;
  std::string device_;
  std::string camera_frame_;
  int input_width_ = 640;
  int input_height_ = 640;
  int max_detections_ = 100;
  int warmup_runs_ = 1;
  float confidence_threshold_ = 0.5F;
  float nms_threshold_ = 0.45F;
  double performance_log_period_ = 5.0;
  double input_timeout_seconds_ = 2.0;
  bool swap_rb_ = true;
  bool publish_debug_ = true;
  bool fail_on_model_error_ = true;
  std::vector<std::string> labels_;
  std::vector<std::string> class_filter_;
  std::unique_ptr<YoloOutputParser> output_parser_;
#ifdef PERCEPTION_HAS_TENSORRT
  std::unique_ptr<TensorRtBackend> tensorrt_backend_;
#endif

  cv::dnn::Net net_;
  std::vector<std::string> output_layer_names_;
  bool model_ready_ = false;

  rclcpp::Publisher<vision_servo_msgs::msg::TargetArray>::SharedPtr detection_pub_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher debug_pub_;
  diagnostic_updater::Updater diagnostics_;

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_image_;
  bool stop_worker_ = false;
  std::thread worker_;
  std::atomic<uint64_t> dropped_frames_{0};
  std::atomic<uint64_t> received_frames_{0};
  std::atomic<uint64_t> inference_errors_{0};
  std::atomic<int64_t> last_input_steady_ns_{0};

  std::chrono::steady_clock::time_point stats_window_start_;
  uint64_t stats_frame_count_ = 0;
  double stats_total_inference_ms_ = 0.0;
  double stats_max_inference_ms_ = 0.0;
  std::vector<double> stats_inference_samples_ms_;
  uint64_t stats_latency_count_ = 0;
  double stats_total_end_to_end_ms_ = 0.0;
  double stats_max_end_to_end_ms_ = 0.0;
  std::vector<double> stats_end_to_end_samples_ms_;
};

}  // namespace perception_pkg
