#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace sony_camera_pkg {

using FrameCallback = std::function<void(const std::vector<uint8_t>& jpeg_data,
                                         uint32_t width, uint32_t height)>;

class SonyCameraStreamer {
public:
  SonyCameraStreamer();
  ~SonyCameraStreamer();

  SonyCameraStreamer(const SonyCameraStreamer&) = delete;
  SonyCameraStreamer& operator=(const SonyCameraStreamer&) = delete;

  bool init();
  bool enumerateDevices(std::vector<std::string>& out_devices);
  bool connect(int camera_index);
  bool disconnect();
  bool release();

  bool startStreaming(FrameCallback callback);
  void stopStreaming();
  bool isInitialized() const;
  bool isConnected() const;
  uint64_t consecutiveCaptureFailures() const;

private:
  struct Impl;

  int64_t device_handle_ = 0;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> streaming_{false};
  std::atomic<uint64_t> consecutive_capture_failures_{0};
  std::mutex stream_mutex_;
  FrameCallback frame_callback_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sony_camera_pkg
