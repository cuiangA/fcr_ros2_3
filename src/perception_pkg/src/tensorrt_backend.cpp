#include "perception_pkg/tensorrt_backend.hpp"

#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

namespace perception_pkg {
namespace {

class TensorRtLogger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char* message) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      last_message = message != nullptr ? message : "TensorRT reported an unknown error";
    }
  }
  std::string last_message;
};

template<typename T>
struct TensorRtDeleter {
  void operator()(T* object) const noexcept { delete object; }
};

template<typename T>
using TensorRtPtr = std::unique_ptr<T, TensorRtDeleter<T>>;

void check_cuda(cudaError_t result, const char* operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(
        std::string(operation) + " failed: " + cudaGetErrorString(result));
  }
}

size_t element_count(const nvinfer1::Dims& dimensions)
{
  if (dimensions.nbDims <= 0) {
    throw std::runtime_error("TensorRT tensor has no dimensions");
  }
  size_t count = 1;
  for (int index = 0; index < dimensions.nbDims; ++index) {
    const int64_t dimension = dimensions.d[index];
    if (dimension <= 0) {
      throw std::runtime_error("Only static positive TensorRT dimensions are supported");
    }
    if (static_cast<uint64_t>(dimension) >
        std::numeric_limits<size_t>::max() / count) {
      throw std::runtime_error("TensorRT tensor size overflow");
    }
    count *= static_cast<size_t>(dimension);
  }
  return count;
}

std::string dimensions_to_string(const nvinfer1::Dims& dimensions)
{
  std::ostringstream stream;
  stream << '[';
  for (int index = 0; index < dimensions.nbDims; ++index) {
    if (index > 0) {
      stream << ',';
    }
    stream << dimensions.d[index];
  }
  stream << ']';
  return stream.str();
}

std::vector<char> read_engine(const std::string& path)
{
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) {
    throw std::runtime_error("Cannot open TensorRT engine: " + path);
  }
  const std::streamsize length = input.tellg();
  if (length <= 0) {
    throw std::runtime_error("TensorRT engine is empty: " + path);
  }
  input.seekg(0, std::ios::beg);
  std::vector<char> data(static_cast<size_t>(length));
  if (!input.read(data.data(), length)) {
    throw std::runtime_error("Cannot read complete TensorRT engine: " + path);
  }
  return data;
}

}  // namespace

struct TensorRtBackend::Impl {
  explicit Impl(const std::string& engine_path)
  {
    const std::vector<char> serialized = read_engine(engine_path);
    runtime.reset(nvinfer1::createInferRuntime(logger));
    if (!runtime) {
      throw std::runtime_error("Failed to create TensorRT runtime: " + logger.last_message);
    }
    engine.reset(runtime->deserializeCudaEngine(serialized.data(), serialized.size()));
    if (!engine) {
      throw std::runtime_error(
          "Failed to deserialize TensorRT engine (version/GPU mismatch likely): " +
          logger.last_message);
    }

    for (int index = 0; index < engine->getNbIOTensors(); ++index) {
      const char* name = engine->getIOTensorName(index);
      if (name == nullptr) {
        throw std::runtime_error("TensorRT engine returned an unnamed I/O tensor");
      }
      const auto mode = engine->getTensorIOMode(name);
      if (mode == nvinfer1::TensorIOMode::kINPUT) {
        if (!input_name.empty()) {
          throw std::runtime_error("Only one TensorRT input tensor is supported");
        }
        input_name = name;
      } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
        if (!output_name.empty()) {
          throw std::runtime_error("Only one TensorRT output tensor is supported");
        }
        output_name = name;
      }
    }
    if (input_name.empty() || output_name.empty()) {
      throw std::runtime_error("TensorRT engine must contain exactly one input and one output");
    }
    if (engine->getTensorDataType(input_name.c_str()) != nvinfer1::DataType::kFLOAT ||
        engine->getTensorDataType(output_name.c_str()) != nvinfer1::DataType::kFLOAT) {
      throw std::runtime_error(
          "TensorRT engine I/O must be FP32; FP16 internal tactics remain supported");
    }

    input_dimensions = engine->getTensorShape(input_name.c_str());
    output_dimensions = engine->getTensorShape(output_name.c_str());
    input_elements = element_count(input_dimensions);
    output_elements = element_count(output_dimensions);
    context.reset(engine->createExecutionContext());
    if (!context) {
      throw std::runtime_error("Failed to create TensorRT execution context");
    }
    check_cuda(cudaStreamCreate(&stream), "cudaStreamCreate");
    try {
      check_cuda(cudaMalloc(&device_input, input_elements * sizeof(float)), "cudaMalloc(input)");
      check_cuda(cudaMalloc(&device_output, output_elements * sizeof(float)), "cudaMalloc(output)");
    } catch (...) {
      release_cuda();
      throw;
    }
    if (!context->setTensorAddress(input_name.c_str(), device_input) ||
        !context->setTensorAddress(output_name.c_str(), device_output)) {
      release_cuda();
      throw std::runtime_error("TensorRT rejected an I/O tensor address");
    }
    host_output.resize(output_elements);
  }

  ~Impl() { release_cuda(); }

  void release_cuda() noexcept
  {
    if (device_output != nullptr) {
      cudaFree(device_output);
      device_output = nullptr;
    }
    if (device_input != nullptr) {
      cudaFree(device_input);
      device_input = nullptr;
    }
    if (stream != nullptr) {
      cudaStreamDestroy(stream);
      stream = nullptr;
    }
  }

  TensorRtLogger logger;
  TensorRtPtr<nvinfer1::IRuntime> runtime;
  TensorRtPtr<nvinfer1::ICudaEngine> engine;
  TensorRtPtr<nvinfer1::IExecutionContext> context;
  std::string input_name;
  std::string output_name;
  nvinfer1::Dims input_dimensions{};
  nvinfer1::Dims output_dimensions{};
  size_t input_elements = 0;
  size_t output_elements = 0;
  cudaStream_t stream = nullptr;
  void* device_input = nullptr;
  void* device_output = nullptr;
  std::vector<float> host_output;
};

TensorRtBackend::TensorRtBackend(const std::string& engine_path)
  : impl_(std::make_unique<Impl>(engine_path))
{}

TensorRtBackend::~TensorRtBackend() = default;

cv::Mat TensorRtBackend::infer(const cv::Mat& nchw_float_blob)
{
  if (nchw_float_blob.empty() || nchw_float_blob.type() != CV_32F ||
      !nchw_float_blob.isContinuous() || nchw_float_blob.total() != impl_->input_elements) {
    throw std::invalid_argument(
        "TensorRT input blob must be contiguous FP32 with shape " + input_shape());
  }
  check_cuda(
      cudaMemcpyAsync(
          impl_->device_input, nchw_float_blob.ptr<float>(),
          impl_->input_elements * sizeof(float), cudaMemcpyHostToDevice, impl_->stream),
      "cudaMemcpyAsync(input)");
  if (!impl_->context->enqueueV3(impl_->stream)) {
    throw std::runtime_error("TensorRT enqueueV3 failed");
  }
  check_cuda(
      cudaMemcpyAsync(
          impl_->host_output.data(), impl_->device_output,
          impl_->output_elements * sizeof(float), cudaMemcpyDeviceToHost, impl_->stream),
      "cudaMemcpyAsync(output)");
  check_cuda(cudaStreamSynchronize(impl_->stream), "cudaStreamSynchronize");

  std::vector<int> dimensions(static_cast<size_t>(impl_->output_dimensions.nbDims));
  for (int index = 0; index < impl_->output_dimensions.nbDims; ++index) {
    dimensions[static_cast<size_t>(index)] =
        static_cast<int>(impl_->output_dimensions.d[index]);
  }
  return cv::Mat(
      static_cast<int>(dimensions.size()), dimensions.data(), CV_32F,
      impl_->host_output.data()).clone();
}

std::string TensorRtBackend::input_shape() const
{
  return dimensions_to_string(impl_->input_dimensions);
}

std::string TensorRtBackend::output_shape() const
{
  return dimensions_to_string(impl_->output_dimensions);
}

}  // namespace perception_pkg
