/**
 * @file yolo_inference.cpp
 * @brief YOLOv8 ONNX 推理实现（OpenCV DNN 后端）。
 */

#include "perception_pkg/yolo_inference.hpp"
#include <algorithm>
#include <stdexcept>

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
    class_whitelist_(class_whitelist), coco_names_(coco_names())
{
  if (model_path.empty()) {
    throw std::runtime_error("[YoloInference] model_path 为空，无法加载模型");
  }
  net_ = cv::dnn::readNetFromONNX(model_path);

  // CUDA 后端：仅当 OpenCV 编译了 CUDA 且可用时启用，否则自动回退 CPU
  if (device.find("cuda") == 0) {
    bool cuda_ok = false;
    try {
      auto backends = cv::dnn::getAvailableBackends();
      for (const auto& b : backends) {
        if (b.first == cv::dnn::DNN_BACKEND_CUDA) cuda_ok = true;
      }
    } catch (...) { cuda_ok = false; }
    if (cuda_ok) {
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
  }

  if (!class_whitelist_.empty()) use_whitelist_ = true;
}

vision_servo_msgs::msg::TargetArray YoloInference::detect(const cv::Mat& frame) {
  vision_servo_msgs::msg::TargetArray result;
  if (frame.empty()) return result;

  // ── 1. letterbox 预处理（保持长宽比，灰边填充） ───────────────────
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

  // ── 2. blob：/255 归一化 + BGR->RGB ──────────────────────────────
  cv::Mat blob;
  cv::dnn::blobFromImage(padded, blob, 1.0 / 255.0,
                         cv::Size(input_w_, input_h_),
                         cv::Scalar(), true, false, CV_32F);

  // ── 3. 前向推理 ─────────────────────────────────────────────────
  net_.setInput(blob);
  cv::Mat out = net_.forward();   // 常见 [1,84,8400]；部分导出为 [1,8400,84]

  // 统一按 (channel, anchor) 读取，兼容两种输出布局
  const int C = 84;  // 4 框坐标 + 80 类分数
  const float* data = out.ptr<float>();  // 连续缓冲区首地址
  const int num_anchors = static_cast<int>(out.total() / C);

  // 判断内存排布：channels 在前 (ch*anchors+a) 还是 anchors 在前 (a*C+ch)
  bool channel_major = true;
  if (out.dims == 3 && out.size[1] == num_anchors && out.size[2] == C) {
    channel_major = false;
  }
  auto at = [&](int ch, int a) -> float {
    return channel_major ? data[ch * num_anchors + a] : data[a * C + ch];
  };

  const int num_classes = 80;

  std::vector<cv::Rect> rects;
  std::vector<float> scores;
  std::vector<int> cls_ids;

  // ── 4. 解析输出 + 置信度过滤 + 反 letterbox 还原像素坐标 ────────
  for (int a = 0; a < num_anchors; ++a) {
    float best = 0.f;
    int best_c = -1;
    for (int c = 0; c < num_classes; ++c) {
      const float s = at(4 + c, a);
      if (s > best) { best = s; best_c = c; }
    }
    if (best < conf_threshold_) continue;

    const float xc = at(0, a);
    const float yc = at(1, a);
    const float w  = at(2, a);
    const float h  = at(3, a);

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

  // ── 5. NMS 后处理 ───────────────────────────────────────────────
  std::vector<int> indices;
  cv::dnn::NMSBoxes(rects, scores, conf_threshold_, nms_threshold_, indices);

  for (const int i : indices) {
    vision_servo_msgs::msg::Target t;
    t.id = -1;  // 跟踪节点负责分配稳定 ID
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
    // 3D 位置/速度由下游 DepthEstimator / PerceptionPipeline 深度阶段填充
    result.targets.push_back(t);
  }

  return result;
}

}  // namespace perception_pkg
