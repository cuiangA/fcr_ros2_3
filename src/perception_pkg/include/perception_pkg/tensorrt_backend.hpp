/**
 * @file tensorrt_backend.hpp
 * @brief TensorRT 10 runtime wrapper for a trusted, locally-built static YOLO engine.
 */

#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

namespace perception_pkg {

class TensorRtBackend {
public:
  explicit TensorRtBackend(const std::string& engine_path);
  ~TensorRtBackend();
  TensorRtBackend(const TensorRtBackend&) = delete;
  TensorRtBackend& operator=(const TensorRtBackend&) = delete;

  cv::Mat infer(const cv::Mat& nchw_float_blob);
  std::string input_shape() const;
  std::string output_shape() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception_pkg
