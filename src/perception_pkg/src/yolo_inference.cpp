/**
 * @file yolo_inference.cpp
 * @brief YOLOv8 ONNX 推理实现（ONNX Runtime 后端）。
 *
 * 推理流程（每帧）：
 *   1. letterbox 预处理：保持长宽比 resize + 灰边填充 → 640×640
 *   2. blob：/255 归一化 + BGR→RGB + HWC→CHW
 *   3. Ort::Session::Run → 输出 [1, 84, 8400]
 *   4. 解析 + 置信度过滤 + 类别白名单过滤 + 反 letterbox 还原像素坐标
 *   5. OpenCV NMSBoxes 后处理
 *   6. 构造 vision_servo_msgs::TargetArray
 */

#include "perception_pkg/yolo_inference.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <algorithm>
#include <stdexcept>
#include <cstring>
#include <vector>

namespace perception_pkg {

const std::vector<std::string>& YoloInference::coco_names() {
  static const std::vector<std::string> names = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
  };
  return names;
}

YoloInference::YoloInference(const std::string& model_path,
                             float conf_threshold,
                             float nms_threshold,
                             const std::vector<std::string>& class_whitelist,
                             const std::string& device)
  : conf_threshold_(conf_threshold), nms_threshold_(nms_threshold),
    class_whitelist_(class_whitelist), coco_names_(coco_names()),
    allocator_()
{
  if (model_path.empty()) {
    throw std::runtime_error("[YoloInference] model_path 为空，无法加载模型");
  }

  // ── 初始化 ORT Env + Session ────────────────────────────────────
  ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "yolo_inference");
  Ort::SessionOptions session_options;
  session_options.SetIntraOpNumThreads(1);    // 单线程内并发，避免线程开销
  session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  // 可选：CUDA Execution Provider
  if (device.find("cuda") == 0) {
    try {
      OrtCUDAProviderOptions cuda_options{};
      session_options.AppendExecutionProvider_CUDA(cuda_options);
    } catch (const Ort::Exception&) {
      // CUDA 不可用，自动回退 CPU
    }
  }

  // 创建 session（接受窄字符串或宽字符串）
  ort_session_ = std::make_unique<Ort::Session>(
    *ort_env_, model_path.c_str(), session_options);

  // 输入输出名（用 allocator 释放）
  for (size_t i = 0; i < ort_session_->GetInputCount(); ++i) {
    auto name = ort_session_->GetInputNameAllocated(i, allocator_);
    input_names_.emplace_back(name.get());
  }
  for (size_t i = 0; i < ort_session_->GetOutputCount(); ++i) {
    auto name = ort_session_->GetOutputNameAllocated(i, allocator_);
    output_names_.emplace_back(name.get());
  }

  // 输入 shape：[1, 3, 640, 640]（YOLOv8n 标准输入）
  // 注意：从 ORT 1.24 GetShape() 读出来的可能是全 0（动态维度），
  // 不能用作 Run 时的 shape 参数。直接硬编码。
  input_shape_ = {1, 3, 640, 640};
  input_h_ = 640;
  input_w_ = 640;

  if (!class_whitelist_.empty()) use_whitelist_ = true;
}

vision_servo_msgs::msg::TargetArray YoloInference::detect(const cv::Mat& frame) {
  vision_servo_msgs::msg::TargetArray result;
  if (frame.empty()) return result;
  if (!ort_session_) return result;

  // ── 1. letterbox 预处理（保持长宽比，灰边填充） ─────────────────
  const float scale = std::min(static_cast<float>(input_w_) / frame.cols,
                               static_cast<float>(input_h_) / frame.rows);
  const int new_w = static_cast<int>(frame.cols * scale);
  const int new_h = static_cast<int>(frame.rows * scale);
  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(new_w, new_h));
  cv::Mat padded(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
  const int pad_x = (input_w_ - new_w) / 2;
  const int pad_y = (input_h_ - new_h) / 2;
  resized.copyTo(padded(cv::Rect(pad_x, pad_y, new_w, new_h)));

  // ── 2. blob：/255 归一化 + BGR→RGB + HWC→CHW ──────────────────
  cv::Mat blob;
  cv::dnn::blobFromImage(padded, blob, 1.0 / 255.0,
                         cv::Size(input_w_, input_h_),
                         cv::Scalar(), true, false, CV_32F);

  // ── 3. 构造 ORT 输入 tensor ─────────────────────────────────────
  Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(
    OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    mem_info, blob.ptr<float>(), blob.total(),
    input_shape_.data(), input_shape_.size());

  // ── 4. Run（ORT 1.24 API: RunOptions, input_names, input_values, input_count,
  //                   output_names, output_values, output_count） ──
  // ORT 1.24 要求 const char* const* 形式的名字指针数组
  std::vector<const char*> input_name_ptrs;
  std::vector<const char*> output_name_ptrs;
  for (const auto& n : input_names_)  input_name_ptrs.push_back(n.c_str());
  for (const auto& n : output_names_) output_name_ptrs.push_back(n.c_str());
  std::vector<Ort::Value> outputs(1);
  ort_session_->Run(
    Ort::RunOptions{nullptr},
    input_name_ptrs.data(), &input_tensor, 1,
    output_name_ptrs.data(), outputs.data(), 1);

  // ── 5. 解析输出 [1, 84, 8400] ─────────────────────────────────
  // ORT 输出维度：[1, 84, 8400]
  const float* data = outputs[0].GetTensorData<float>();
  auto out_info = outputs[0].GetTensorTypeAndShapeInfo();
  auto out_shape = out_info.GetShape();
  const int C = (out_shape.size() >= 2) ? static_cast<int>(out_shape[1]) : 84;
  const int num_anchors = static_cast<int>(out_info.GetElementCount() / C);
  const int num_classes = C - 4;

  std::vector<cv::Rect> rects;
  std::vector<float> scores;
  std::vector<int> cls_ids;

  for (int a = 0; a < num_anchors; ++a) {
    float best = 0.f;
    int best_c = -1;
    for (int c = 0; c < num_classes; ++c) {
      const float s = data[(4 + c) * num_anchors + a];  // [1,84,8400] → channel-major
      if (s > best) { best = s; best_c = c; }
    }
    if (best < conf_threshold_) continue;

    const float xc = data[0 * num_anchors + a];
    const float yc = data[1 * num_anchors + a];
    const float w  = data[2 * num_anchors + a];
    const float h  = data[3 * num_anchors + a];

    float x1 = (xc - w / 2.f - pad_x) / scale;
    float y1 = (yc - h / 2.f - pad_y) / scale;
    float x2 = (xc + w / 2.f - pad_x) / scale;
    float y2 = (yc + h / 2.f - pad_y) / scale;

    x1 = std::max(0.f, x1); y1 = std::max(0.f, y1);
    x2 = std::min(static_cast<float>(frame.cols), x2);
    y2 = std::min(static_cast<float>(frame.rows), y2);
    if (x2 <= x1 || y2 <= y1) continue;

    const std::string name = (best_c < static_cast<int>(coco_names_.size()))
        ? coco_names_[best_c] : ("class" + std::to_string(best_c));

    if (use_whitelist_) {
      bool keep = false;
      for (const auto& wl : class_whitelist_) {
        if (wl == name) { keep = true; break; }
      }
      if (!keep) continue;
    }

    rects.emplace_back(static_cast<int>(x1), static_cast<int>(y1),
                       static_cast<int>(x2 - x1), static_cast<int>(y2 - y1));
    scores.push_back(best);
    cls_ids.push_back(best_c);
  }

  // ── 6. NMS 后处理 ──────────────────────────────────────────────
  std::vector<int> indices;
  cv::dnn::NMSBoxes(rects, scores, conf_threshold_, nms_threshold_, indices);

  for (const int i : indices) {
    vision_servo_msgs::msg::Target t;
    t.id = -1;
    const std::string name = (cls_ids[i] < static_cast<int>(coco_names_.size()))
        ? coco_names_[cls_ids[i]] : ("class" + std::to_string(cls_ids[i]));
    t.class_name = name;

    const float x1 = static_cast<float>(rects[i].x);
    const float y1 = static_cast<float>(rects[i].y);
    const float x2 = static_cast<float>(rects[i].x + rects[i].width);
    const float y2 = static_cast<float>(rects[i].y + rects[i].height);
    t.bbox = {x1, y1, x2, y2};
    t.center = {(x1 + x2) / 2.f, (y1 + y2) / 2.f};
    t.confidence = scores[i];
    t.width = static_cast<float>(rects[i].width);
    t.height = static_cast<float>(rects[i].height);
    result.targets.push_back(t);
  }

  return result;
}

}  // namespace perception_pkg
