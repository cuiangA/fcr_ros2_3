/**
 * @file wake_up_node.cpp
 * @brief 语音唤醒 + 录音 + HTTP 云端 ASR + 指令发布的 ROS2 节点（C++ 实现）。
 *
 * 工作流程：
 *   1. 麦克风持续监听，sherpa-onnx 做唤醒词检测（"你好西西"）
 *   2. 检测到唤醒词后进入录音模式
 *   3. 基于 RMS 能量做 VAD：有声音时录音，安静 1.5 s 后结束
 *   4. 将录音保存为临时 WAV 文件，通过 HTTP POST 发往本地/云端服务器
 *   5. 发布 ASR 文本；兼容模式下也可发布云端返回的 VoiceCommand
 *
 * 使用 reSpeaker XVF3800 等 USB 音频设备时，通过 mic_device 参数指定设备索引。
 *
 * 用法：
 *   ros2 run external_control_pkg wake_up_node
 *   ros2 launch external_control_pkg voice_control.launch.py
 */

#include "external_control_pkg/msg/voice_command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <portaudio.h>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <sherpa-onnx/c-api/c-api.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace external_control_pkg {

// ── 常量 ──────────────────────────────────────────────────────────────

/// sherpa-onnx 与 WAV 文件统一的采样率
constexpr int kSampleRate = 16000;

/// 每帧采样数（与模型 frame_length 对齐）
constexpr int kChunkSize = 512;

/// 静音超时（秒）：检测到声音后，连续静音这么久才结束录音
constexpr double kSilenceTimeoutSec = 5.0;

/// RMS 能量阈值（低于此值视为静音）
constexpr float kEnergyThreshold = 500.0f;

/// 最短录音时长（秒）：未录够这么久，不允许因静音而结束
constexpr double kMinRecordSec = 5.0;

/// 最长录音时长（秒），防止死循环
constexpr double kMaxRecordSec = 20.0;

/// 唤醒关键词 (格式: 空格分隔的 token @显示名, 与 keywords.txt 一致)
constexpr const char *kWakeKeyword = "n ǐ h ǎo x ī x ī @你好西西";

// ── libcurl 写回调 ────────────────────────────────────────────────────

/** @brief libcurl 响应数据写入回调（追加到 string）。 */
static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
  auto *output = static_cast<std::string *>(userp);
  size_t total = size * nmemb;
  output->append(static_cast<const char *>(contents), total);
  return total;
}

// ═══════════════════════════════════════════════════════════════════════
// WakeUpNode
// ═══════════════════════════════════════════════════════════════════════

/** @class WakeUpNode @brief 语音唤醒 → 录音 → ASR → 发布的 ROS2 节点。 */
class WakeUpNode : public rclcpp::Node {
public:
  /** @param options ROS2 节点选项 */
  explicit WakeUpNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("wake_up_node", options)
  {
    // ── 1. 声明参数 ──────────────────────────────────────────────
    // model_dir 默认指向安装后的 share 目录，模型随包一同 install
    const std::string default_model_dir =
      ament_index_cpp::get_package_share_directory("external_control_pkg")
      + "/models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01";

    this->declare_parameter("cloud_asr_url", "http://localhost:8080/asr");
    this->declare_parameter("cmd_topic", "/external/voice_command");
    this->declare_parameter("text_topic", "/voice/text");
    this->declare_parameter("publish_cloud_intents", true);
    this->declare_parameter("model_dir", default_model_dir);
    this->declare_parameter("energy_threshold", kEnergyThreshold);
    this->declare_parameter("silence_timeout", kSilenceTimeoutSec);
    this->declare_parameter("mic_device", -1);

    asr_url_ = this->get_parameter("cloud_asr_url").as_string();
    cmd_topic_ = this->get_parameter("cmd_topic").as_string();
    text_topic_ = this->get_parameter("text_topic").as_string();
    publish_cloud_intents_ =
      this->get_parameter("publish_cloud_intents").as_bool();
    model_dir_ = this->get_parameter("model_dir").as_string();
    if (model_dir_.empty()) {
      // launch 文件传了空字符串时，回退到编译期默认值
      model_dir_ =
        ament_index_cpp::get_package_share_directory("external_control_pkg")
        + "/models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01";
    }
    energy_thr_ = static_cast<float>(this->get_parameter("energy_threshold").as_double());
    silence_to_ = this->get_parameter("silence_timeout").as_double();
    mic_device_ = this->get_parameter("mic_device").as_int();

    // ── 2. 初始化 PortAudio ─────────────────────────────────────
    initAudio();

    // ── 3. 加载 sherpa-onnx 关键词识别器 ────────────────────────
    initSherpa();

    // ── 4. 创建发布者 ────────────────────────────────────────────
    cmd_pub_ = this->create_publisher<external_control_pkg::msg::VoiceCommand>(
      cmd_topic_, rclcpp::QoS(10).reliable());
    text_pub_ = this->create_publisher<std_msgs::msg::String>(
      text_topic_, rclcpp::QoS(10).reliable());

    // ── 5. 启动定时器（与音频帧率对齐） ──────────────────────────
    int period_ms = static_cast<int>(kChunkSize * 1000.0 / kSampleRate);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&WakeUpNode::listenLoop, this));

    RCLCPP_INFO(this->get_logger(),
                "语音唤醒节点已启动 | 模型=%s, ASR=%s, 文本=%s, 云端意图=%s",
                model_dir_.c_str(), asr_url_.c_str(), text_topic_.c_str(),
                publish_cloud_intents_ ? "enabled" : "disabled");
  }

  ~WakeUpNode() override {
    if (pa_stream_) {
      if (Pa_IsStreamActive(pa_stream_)) Pa_StopStream(pa_stream_);
      Pa_CloseStream(pa_stream_);
    }
    if (pa_initialized_) Pa_Terminate();
    if (sherpa_spotter_) {
      if (sherpa_stream_) SherpaOnnxDestroyOnlineStream(sherpa_stream_);
      SherpaOnnxDestroyKeywordSpotter(sherpa_spotter_);
    }
  }

private:
  // ═══════════════════════════════════════════════════════════════
  // 初始化
  // ═══════════════════════════════════════════════════════════════

  /** @brief 初始化 PortAudio，打开麦克风输入流。 */
  void initAudio() {
    PaError err = Pa_Initialize();
    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "PortAudio 初始化失败: %s",
                   Pa_GetErrorText(err));
      throw std::runtime_error("PortAudio init failed");
    }
    pa_initialized_ = true;

    PaDeviceIndex device = (mic_device_ >= 0)
      ? static_cast<PaDeviceIndex>(mic_device_)
      : Pa_GetDefaultInputDevice();
    if (device == paNoDevice) {
      throw std::runtime_error("未找到可用的麦克风输入设备");
    }

    const PaDeviceInfo *dev_info = Pa_GetDeviceInfo(device);
    PaStreamParameters input_params;
    input_params.device = device;
    input_params.channelCount = 1;
    input_params.sampleFormat = paInt16;
    input_params.suggestedLatency = dev_info->defaultLowInputLatency;
    input_params.hostApiSpecificStreamInfo = nullptr;

    err = Pa_OpenStream(&pa_stream_,
                        &input_params,    // input
                        nullptr,          // no output
                        kSampleRate,
                        kChunkSize,
                        paClipOff,
                        nullptr,          // no callback (blocking read)
                        nullptr);
    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "麦克风打开失败: %s", Pa_GetErrorText(err));
      throw std::runtime_error("Microphone open failed");
    }

    err = Pa_StartStream(pa_stream_);
    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "音频流启动失败: %s", Pa_GetErrorText(err));
      throw std::runtime_error("Audio stream start failed");
    }

    RCLCPP_INFO(this->get_logger(), "麦克风已打开: %s (device=%d)",
                dev_info->name, device);
  }

  /** @brief 初始化 sherpa-onnx 关键词识别器。 */
  void initSherpa() {
    SherpaOnnxKeywordSpotterConfig config;
    memset(&config, 0, sizeof(config));

    // 特征配置
    config.feat_config.sample_rate = kSampleRate;
    config.feat_config.feature_dim = 80;

    // 模型路径 — zipformer transducer
    std::string base = model_dir_;
    model_encoder_ = base + "/encoder-epoch-99-avg-1-chunk-16-left-64.onnx";
    model_decoder_ = base + "/decoder-epoch-99-avg-1-chunk-16-left-64.onnx";
    model_joiner_  = base + "/joiner-epoch-99-avg-1-chunk-16-left-64.onnx";
    model_tokens_  = base + "/tokens.txt";

    config.model_config.tokens = model_tokens_.c_str();
    config.model_config.num_threads = 2;
    config.model_config.provider = "cpu";
    config.model_config.transducer.encoder = model_encoder_.c_str();
    config.model_config.transducer.decoder = model_decoder_.c_str();
    config.model_config.transducer.joiner  = model_joiner_.c_str();

    // 关键词
    keywords_buf_ = std::string(kWakeKeyword) + "\n";
    config.keywords_buf = keywords_buf_.c_str();
    config.keywords_buf_size = static_cast<int32_t>(keywords_buf_.size());

    config.max_active_paths = 4;
    config.keywords_threshold = 0.5f;
    config.keywords_score = 1.0f;

    sherpa_spotter_ = SherpaOnnxCreateKeywordSpotter(&config);
    if (!sherpa_spotter_) {
      RCLCPP_ERROR(this->get_logger(),
                   "SherpaOnnxCreateKeywordSpotter 失败，"
                   "请检查模型文件是否完整:\n  %s\n  %s\n  %s\n  %s",
                   model_encoder_.c_str(), model_decoder_.c_str(),
                   model_joiner_.c_str(), model_tokens_.c_str());
      throw std::runtime_error("sherpa-onnx keyword spotter init failed");
    }

    sherpa_stream_ = SherpaOnnxCreateKeywordStream(sherpa_spotter_);
    if (!sherpa_stream_) {
      SherpaOnnxDestroyKeywordSpotter(sherpa_spotter_);
      sherpa_spotter_ = nullptr;
      throw std::runtime_error("sherpa-onnx stream creation failed");
    }

    RCLCPP_INFO(this->get_logger(), "唤醒词模型加载完成 | 关键词=\"%s\"",
                kWakeKeyword);
  }

  // ═══════════════════════════════════════════════════════════════
  // 主循环（定时器回调）
  // ═══════════════════════════════════════════════════════════════

  /** @brief 定时器回调 — 从麦克风读一帧音频并处理。 */
  void listenLoop() {
    // 读取一帧
    int16_t audio_int16[kChunkSize];
    PaError err = Pa_ReadStream(pa_stream_, audio_int16, kChunkSize);
    if (err != paNoError) {
      RCLCPP_WARN(this->get_logger(), "音频读取错误: %s", Pa_GetErrorText(err));
      return;
    }

    // 计算 RMS 能量
    float sum_sq = 0.0f;
    for (int i = 0; i < kChunkSize; ++i) {
      sum_sq += static_cast<float>(audio_int16[i]) * audio_int16[i];
    }
    float rms = std::sqrt(sum_sq / static_cast<float>(kChunkSize));

    rclcpp::Time now = this->now();

    if (listening_for_wake_) {
      // ── 唤醒词监听 ──────────────────────────────────────────
      // 转为 float32 喂入 sherpa-onnx
      float audio_float32[kChunkSize];
      for (int i = 0; i < kChunkSize; ++i) {
        audio_float32[i] = audio_int16[i] / 32768.0f;
      }
      SherpaOnnxOnlineStreamAcceptWaveform(
        sherpa_stream_, kSampleRate, audio_float32, kChunkSize);

      while (SherpaOnnxIsKeywordStreamReady(sherpa_spotter_, sherpa_stream_)) {
        SherpaOnnxDecodeKeywordStream(sherpa_spotter_, sherpa_stream_);
        const SherpaOnnxKeywordResult *result =
          SherpaOnnxGetKeywordResult(sherpa_spotter_, sherpa_stream_);
        if (result && result->keyword && strlen(result->keyword) > 0) {
          RCLCPP_INFO(this->get_logger(),
                      "检测到唤醒词: \"%s\"", result->keyword);

          // 重置流以继续监听后续唤醒
          SherpaOnnxDestroyKeywordResult(result);
          SherpaOnnxDestroyOnlineStream(sherpa_stream_);
          sherpa_stream_ = SherpaOnnxCreateKeywordStream(sherpa_spotter_);

          // 切换到录音模式
          listening_for_wake_ = false;
          is_recording_ = true;
          min_record_passed_ = false;  // 重置最小时长标志
          record_buffer_.clear();
          record_buffer_.reserve(static_cast<size_t>(kSampleRate * kMaxRecordSec));
          record_start_time_ = now;
          last_voice_time_ = now;

          // 保存当前帧
          record_buffer_.insert(record_buffer_.end(),
                                audio_int16, audio_int16 + kChunkSize);

          RCLCPP_INFO(this->get_logger(), "唤醒成功，开始录音...");
          return;
        }
        SherpaOnnxDestroyKeywordResult(result);
      }
    } else {
      // ── 录音阶段 ────────────────────────────────────────────
      record_buffer_.insert(record_buffer_.end(),
                            audio_int16, audio_int16 + kChunkSize);

      if (rms > energy_thr_) {
        last_voice_time_ = now;          // 有声音，刷新静音计时
      }

      double record_dur   = (now - record_start_time_).seconds();

      // 1. 硬超时，强制结束
      if (record_dur > kMaxRecordSec) {
        RCLCPP_INFO(this->get_logger(), "录音超时 (%.1fs)，强制结束", record_dur);
        finishRecording();
        return;
      }
      // 2. 检测是否刚达到最小时长，若是则重置静音时钟
      if (!min_record_passed_ && record_dur >= kMinRecordSec) {
        min_record_passed_ = true;
        last_voice_time_   = now;  // 静音超时时钟从此刻重新计时
      }
      // 3. 未到最小时长，不允许因静音而结束
      if (!min_record_passed_) {
        return;
      }
      // 4. 到了最小时长，且静音超时 → 结束
      double silence_dur = (now - last_voice_time_).seconds();
      if (silence_dur > silence_to_) {
        RCLCPP_INFO(this->get_logger(),
                    "静音超时 (%.1fs)，录音结束 (共%.1fs)",
                    silence_dur, record_dur);
        finishRecording();
      }
    }
  }

  // ═══════════════════════════════════════════════════════════════
  // 录音完成
  // ═══════════════════════════════════════════════════════════════

  /** @brief 录音结束：写 WAV → HTTP POST → 解析 → 发布 → 重置。 */
  void finishRecording() {
    // ── 写临时 WAV 文件 ────────────────────────────────────────
    char tmp_path[] = "/tmp/fcr_voice_XXXXXX.wav";
    // mkstemps 创建带后缀的临时文件
    int fd = mkstemps(tmp_path, 4);  // 4 = ".wav" 长度
    if (fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "创建临时文件失败");
      resetState();
      return;
    }
    close(fd);  // 关闭 fd，后面用 ofstream 写入
    std::string wav_path(tmp_path);

    if (!writeWav(wav_path, record_buffer_)) {
      std::remove(wav_path.c_str());
      resetState();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "录音文件: %s (%.1fs, %zu samples)",
                wav_path.c_str(),
                record_buffer_.size() / static_cast<double>(kSampleRate),
                record_buffer_.size());

    // ── 调用云端 ASR ──────────────────────────────────────────
    auto cmd_msg = callCloudAsr(wav_path);

    // ── 发布 ASR 文本；新版模式由本地意图模型统一解释 ─────────
    if (!cmd_msg.raw_text.empty()) {
      auto text_msg = std_msgs::msg::String();
      text_msg.data = cmd_msg.raw_text;
      text_pub_->publish(text_msg);
    }
    if (publish_cloud_intents_) {
      cmd_pub_->publish(cmd_msg);
    }

    std::string intents_str;
    for (size_t i = 0; i < cmd_msg.intents.size(); ++i) {
      if (i > 0) intents_str += ", ";
      intents_str += cmd_msg.intents[i];
    }
    RCLCPP_INFO(
      this->get_logger(),
      "ASR 完成: intents=[%s], raw_text=\"%s\", cloud_intents_published=%s",
      intents_str.c_str(), cmd_msg.raw_text.c_str(),
      publish_cloud_intents_ ? "true" : "false");

    // ── 清理临时文件 ──────────────────────────────────────────
    std::remove(wav_path.c_str());

    // ── 重置状态 ──────────────────────────────────────────────
    resetState();
  }

  /** @brief 重置到唤醒监听状态。 */
  void resetState() {
    is_recording_      = false;
    listening_for_wake_ = true;
    min_record_passed_ = false;
    record_buffer_.clear();
    record_buffer_.shrink_to_fit();
  }

  // ═══════════════════════════════════════════════════════════════
  // WAV 写入
  // ═══════════════════════════════════════════════════════════════

  /**
   * @brief 将 int16 采样数据写入 16-bit 单声道 WAV 文件。
   * @param path    输出路径
   * @param samples PCM 采样数据
   * @return 成功返回 true
   */
  static bool writeWav(const std::string &path,
                       const std::vector<int16_t> &samples) {
    std::ofstream f(path, std::ios::binary);
    if (!f.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("wake_up_node"),
                   "无法写入 WAV 文件: %s", path.c_str());
      return false;
    }

    uint32_t data_size  = static_cast<uint32_t>(samples.size()) * sizeof(int16_t);
    uint32_t file_size  = 36 + data_size;
    uint16_t num_ch     = 1;
    uint32_t sr         = static_cast<uint32_t>(kSampleRate);
    uint16_t bits_per   = 16;
    uint16_t block_align = num_ch * (bits_per / 8);
    uint32_t byte_rate  = sr * block_align;

    auto write16 = [&](uint16_t v) { f.write(reinterpret_cast<const char *>(&v), 2); };
    auto write32 = [&](uint32_t v) { f.write(reinterpret_cast<const char *>(&v), 4); };

    f.write("RIFF", 4);     write32(file_size);
    f.write("WAVE", 4);
    f.write("fmt ", 4);     write32(16);          // chunk size
    write16(1);                                    // PCM
    write16(num_ch);
    write32(sr);
    write32(byte_rate);
    write16(block_align);
    write16(bits_per);
    f.write("data", 4);     write32(data_size);
    f.write(reinterpret_cast<const char *>(samples.data()),
            static_cast<std::streamsize>(data_size));

    return true;
  }

  // ═══════════════════════════════════════════════════════════════
  // HTTP 云端 ASR 调用
  // ═══════════════════════════════════════════════════════════════

  /**
   * @brief 将音频文件发往云端 ASR，解析 JSON 返回 VoiceCommand 消息。
   *
   * Mock 接口约定：
   *   POST {asr_url}
   *   Content-Type: multipart/form-data
   *   Body: audio=<wav_file>
   *
   *   期望返回:
   *   {
   *     "intents": ["start_following"],
   *     "confidences": [0.97],
   *     "raw_text": "跟上我",
   *     "distance": -1,
   *     "unit": "",
   *     "speed": "",
   *     "target_desc": "",
   *     "follow": false
   *   }
   */
  external_control_pkg::msg::VoiceCommand callCloudAsr(const std::string &wav_path) {
    external_control_pkg::msg::VoiceCommand cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "voice_controller";

    CURL *curl = curl_easy_init();
    if (!curl) {
      RCLCPP_WARN(this->get_logger(), "libcurl 初始化失败");
      fillUnknown(cmd);
      return cmd;
    }

    curl_mime *mime = curl_mime_init(curl);
    curl_mimepart *part = curl_mime_addpart(mime);
    curl_mime_name(part, "audio");
    curl_mime_filedata(part, wav_path.c_str());

    std::string response;
    curl_easy_setopt(curl, CURLOPT_URL, asr_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);

    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
      RCLCPP_WARN(this->get_logger(), "云端服务调用失败: %s",
                  curl_easy_strerror(res));
      fillUnknown(cmd);
    } else {
      long http_code = 0;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
      if (http_code == 200) {
        try {
          auto json = nlohmann::json::parse(response);
          cmd.intents     = json.value("intents",
                            std::vector<std::string>{"unknown"});
          cmd.confidences = json.value("confidences",
                            std::vector<float>{0.0f});
          cmd.raw_text    = json.value("raw_text", std::string{});
          cmd.distance    = json.value("distance", -1.0f);
          cmd.unit        = json.value("unit", std::string{});
          cmd.speed       = json.value("speed", std::string{});
          cmd.target_desc = json.value("target_desc", std::string{});
          cmd.follow      = json.value("follow", false);
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), "JSON 解析失败: %s", e.what());
          fillUnknown(cmd);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "云端服务返回 HTTP %ld", http_code);
        fillUnknown(cmd);
      }
    }

    curl_mime_free(mime);
    curl_easy_cleanup(curl);
    return cmd;
  }

  /** @brief 降级：填充 unknown 指令。 */
  static void fillUnknown(external_control_pkg::msg::VoiceCommand &cmd) {
    cmd.intents     = {"unknown"};
    cmd.confidences = {0.0f};
    cmd.raw_text    = {};
    cmd.distance    = -1.0f;
    cmd.follow      = false;
  }

  // ── 成员变量 ─────────────────────────────────────────────────────

  // ROS2 基础设施
  rclcpp::Publisher<external_control_pkg::msg::VoiceCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 参数
  std::string asr_url_;
  std::string cmd_topic_;
  std::string text_topic_;
  std::string model_dir_;
  float energy_thr_;
  double silence_to_;
  int mic_device_;
  bool publish_cloud_intents_ = true;

  // 模型路径（生命周期与节点相同，c_str() 指针稳定）
  std::string model_encoder_;
  std::string model_decoder_;
  std::string model_joiner_;
  std::string model_tokens_;
  std::string keywords_buf_;

  // PortAudio
  bool pa_initialized_ = false;
  PaStream *pa_stream_ = nullptr;

  // sherpa-onnx
  const SherpaOnnxKeywordSpotter *sherpa_spotter_ = nullptr;
  const SherpaOnnxOnlineStream   *sherpa_stream_  = nullptr;

  // 状态机
  bool listening_for_wake_ = true;
  bool is_recording_       = false;
  bool min_record_passed_  = false;  // 是否已过最短录音时长
  std::vector<int16_t> record_buffer_;
  rclcpp::Time record_start_time_{};
  rclcpp::Time last_voice_time_{};
};

}  // namespace external_control_pkg

// ═══════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<external_control_pkg::WakeUpNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

