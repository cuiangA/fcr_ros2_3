#include "sony_camera_pkg/sony_camera_streamer.hpp"

#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "CameraRemote_SDK.h"
#include "CrDeviceProperty.h"
#include "CrError.h"
#include "CrImageDataBlock.h"
#include "ICrCameraObjectInfo.h"
#include "IDeviceCallback.h"

namespace sony_camera_pkg {

namespace SDK = SCRSDK;

// ────────────────────────────────────────────────────────
//  IDeviceCallback — receives connection/property events
// ────────────────────────────────────────────────────────
namespace {

std::string sdk_code(CrInt32u code) {
  std::ostringstream stream;
  stream << "0x" << std::hex << std::uppercase << std::setw(8)
         << std::setfill('0') << code;
  return stream.str();
}

class StreamerCallback : public SDK::IDeviceCallback {
public:
  explicit StreamerCallback(std::atomic<bool>* public_connected)
      : public_connected_(public_connected) {}

  std::atomic<bool> connected{false};
  std::atomic<bool> disconnect_requested{false};
  mutable std::mutex mtx;
  std::string model_id;

  void OnConnected(SDK::DeviceConnectionVersioin /*version*/) override {
    std::lock_guard<std::mutex> lock(mtx);
    connected = true;
    public_connected_->store(true);
    std::cout << "[SonyCamera] Connected to " << model_id << std::endl;
  }

  void OnDisconnected(CrInt32u /*error*/) override {
    std::lock_guard<std::mutex> lock(mtx);
    connected = false;
    public_connected_->store(false);
    if (!disconnect_requested) {
      std::cerr << "[SonyCamera] Unexpected disconnect from " << model_id << std::endl;
    }
  }

  void OnError(CrInt32u error) override {
    std::cerr << "[SonyCamera] Error: " << sdk_code(error) << std::endl;
  }

  void OnWarning(CrInt32u warning) override {
    if (warning == SDK::CrWarning_Connect_Reconnecting) {
      std::cout << "[SonyCamera] Reconnecting..." << std::endl;
    }
  }

  void OnPropertyChanged() override {}
  void OnPropertyChangedCodes(CrInt32u /*num*/, CrInt32u* /*codes*/) override {}
  void OnLvPropertyChanged() override {}
  void OnLvPropertyChangedCodes(CrInt32u /*num*/, CrInt32u* /*codes*/) override {}
  void OnCompleteDownload(CrChar* /*filename*/, CrInt32u /*type*/) override {}
  void OnNotifyContentsTransfer(CrInt32u /*notify*/, SDK::CrContentHandle /*handle*/,
                                CrChar* /*filename*/) override {}
  void OnWarningExt(CrInt32u /*warning*/, CrInt32 /*p1*/, CrInt32 /*p2*/,
                    CrInt32 /*p3*/) override {}
  void OnNotifyFTPTransferResult(CrInt32u /*notify*/, CrInt32u /*ok*/,
                                 CrInt32u /*fail*/) override {}
  void OnNotifyRemoteTransferResult(CrInt32u /*notify*/, CrInt32u /*per*/,
                                    CrChar* /*filename*/) override {}
  void OnNotifyRemoteTransferResult(CrInt32u /*notify*/, CrInt32u /*per*/,
                                    CrInt8u* /*data*/, CrInt64u /*size*/) override {}
  void OnNotifyRemoteTransferContentsListChanged(CrInt32u /*notify*/, CrInt32u /*slot*/,
                                                 CrInt32u /*add*/) override {}
  void OnNotifyRemoteFirmwareUpdateResult(CrInt32u /*notify*/,
                                          const void* /*param*/) override {}
  void OnReceivePlaybackTimeCode(CrInt32u /*tc*/) override {}
  void OnReceivePlaybackData(CrInt8u /*mt*/, CrInt32 /*ds*/, CrInt8u* /*d*/,
                             CrInt64 /*pts*/, CrInt64 /*dts*/, CrInt32 /*p1*/,
                             CrInt32 /*p2*/) override {}
  void OnNotifyMonitorUpdated(CrInt32u /*type*/, CrInt32u /*frameNo*/) override {}
  void OnNotifyPostViewImage(CrChar* /*fn*/, CrInt32u /*sz*/) override {}
  void OnCompleteOperation(CrInt32u /*code*/,
                           SDK::CrOperationResultData* /*result*/) override {}

private:
  std::atomic<bool>* public_connected_;
};

}  // namespace

// ────────────────────────────────────────────────────────
//  SonyCameraStreamer implementation
// ────────────────────────────────────────────────────────

struct SonyCameraStreamer::Impl {
  explicit Impl(std::atomic<bool>* public_connected)
      : callback(public_connected) {}

  StreamerCallback callback;
  std::thread stream_thread;
  std::atomic<bool> stream_stop{false};
};

SonyCameraStreamer::SonyCameraStreamer()
    = default;

SonyCameraStreamer::~SonyCameraStreamer() {
  release();
}

bool SonyCameraStreamer::init() {
  if (initialized_) return true;

  std::cout << "[SonyCamera] Initializing CRSDK..." << std::endl;
  if (!SDK::Init()) {
    std::cerr << "[SonyCamera] SDK::Init() failed" << std::endl;
    return false;
  }
  std::cout << "[SonyCamera] SDK initialized. Version "
            << ((SDK::GetSDKVersion() & 0xFF000000) >> 24) << "."
            << ((SDK::GetSDKVersion() & 0x00FF0000) >> 16) << "."
            << ((SDK::GetSDKVersion() & 0x0000FF00) >> 8) << std::endl;
  initialized_ = true;
  return true;
}

bool SonyCameraStreamer::enumerateDevices(std::vector<std::string>& out_devices) {
  if (!initialized_) {
    std::cerr << "[SonyCamera] SDK is not initialized" << std::endl;
    return false;
  }

  SDK::ICrEnumCameraObjectInfo* enum_info = nullptr;
  auto err = SDK::EnumCameraObjects(&enum_info, 3);
  if (CR_FAILED(err) || enum_info == nullptr) {
    std::cerr << "[SonyCamera] No cameras found. Is the camera connected via USB?"
              << std::endl;
    return false;
  }

  auto count = enum_info->GetCount();
  out_devices.clear();
  for (CrInt32u i = 0; i < count; ++i) {
    auto* info = enum_info->GetCameraObjectInfo(i);
    std::ostringstream oss;
    oss << "[" << (i + 1) << "] " << info->GetModel() << " ("
        << info->GetConnectionTypeName() << ")";
    out_devices.push_back(oss.str());
  }
  enum_info->Release();
  return true;
}

bool SonyCameraStreamer::connect(int camera_index) {
  if (!initialized_) {
    std::cerr << "[SonyCamera] SDK is not initialized" << std::endl;
    return false;
  }
  if (device_handle_ != 0) {
    disconnect();
  }

  SDK::ICrEnumCameraObjectInfo* enum_info = nullptr;
  auto err = SDK::EnumCameraObjects(&enum_info, 3);
  if (CR_FAILED(err) || enum_info == nullptr) {
    std::cerr << "[SonyCamera] No cameras found" << std::endl;
    return false;
  }

  auto count = enum_info->GetCount();
  if (camera_index < 1 || static_cast<CrInt32u>(camera_index) > count) {
    std::cerr << "[SonyCamera] Camera index " << camera_index << " out of range [1,"
              << count << "]" << std::endl;
    enum_info->Release();
    return false;
  }

  auto* info = enum_info->GetCameraObjectInfo(camera_index - 1);
  std::cout << "[SonyCamera] Connecting to " << info->GetModel() << " ("
            << info->GetConnectionTypeName() << ")..." << std::endl;

  impl_ = std::make_unique<Impl>(&connected_);
  impl_->callback.model_id = info->GetModel();

  SDK::CrDeviceHandle handle = 0;
  err = SDK::Connect(info, &impl_->callback, &handle, SDK::CrSdkControlMode_Remote,
                     SDK::CrReconnecting_ON);

  enum_info->Release();

  if (CR_FAILED(err)) {
    std::cerr << "[SonyCamera] Connect failed: "
              << sdk_code(static_cast<CrInt32u>(err)) << std::endl;
    if (handle != 0) {
      SDK::ReleaseDevice(handle);
    }
    impl_.reset();
    return false;
  }

  device_handle_ = static_cast<int64_t>(handle);

  // Wait for OnConnected callback
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!impl_->callback.connected) {
    if (std::chrono::steady_clock::now() > deadline) {
      std::cerr << "[SonyCamera] Connection timeout" << std::endl;
      impl_->callback.disconnect_requested = true;
      SDK::Disconnect(handle);
      const auto disconnect_deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (impl_->callback.connected &&
             std::chrono::steady_clock::now() < disconnect_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      SDK::ReleaseDevice(handle);
      device_handle_ = 0;
      connected_ = false;
      impl_.reset();
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  connected_ = true;
  std::cout << "[SonyCamera] Connected successfully." << std::endl;
  return true;
}

bool SonyCameraStreamer::disconnect() {
  stopStreaming();
  if (device_handle_ == 0) {
    connected_ = false;
    impl_.reset();
    return true;
  }

  std::cout << "[SonyCamera] Disconnecting..." << std::endl;
  auto handle = static_cast<SDK::CrDeviceHandle>(device_handle_);

  if (impl_) {
    impl_->callback.disconnect_requested = true;
  }

  const auto disconnect_error = SDK::Disconnect(handle);

  // Wait for OnDisconnected
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (impl_ && impl_->callback.connected) {
    if (std::chrono::steady_clock::now() > deadline) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  const auto release_error = SDK::ReleaseDevice(handle);
  device_handle_ = 0;
  connected_ = false;
  impl_.reset();
  if (CR_FAILED(disconnect_error)) {
    std::cerr << "[SonyCamera] Disconnect failed: "
              << sdk_code(static_cast<CrInt32u>(disconnect_error)) << std::endl;
  }
  if (CR_FAILED(release_error)) {
    std::cerr << "[SonyCamera] ReleaseDevice failed: "
              << sdk_code(static_cast<CrInt32u>(release_error)) << std::endl;
  }
  return !CR_FAILED(disconnect_error) && !CR_FAILED(release_error);
}

bool SonyCameraStreamer::release() {
  if (!initialized_) return true;

  disconnect();
  std::cout << "[SonyCamera] Releasing SDK..." << std::endl;
  const bool released = SDK::Release();
  initialized_ = false;
  return released;
}

bool SonyCameraStreamer::startStreaming(FrameCallback callback) {
  if (!connected_) {
    std::cerr << "[SonyCamera] Not connected, cannot start streaming" << std::endl;
    return false;
  }
  if (streaming_) return true;
  if (!impl_) {
    std::cerr << "[SonyCamera] Connection state is incomplete" << std::endl;
    return false;
  }

  frame_callback_ = std::move(callback);
  streaming_ = true;
  impl_->stream_stop = false;

  impl_->stream_thread = std::thread([this]() {
    auto handle = static_cast<SDK::CrDeviceHandle>(device_handle_);

    while (!impl_->stream_stop && impl_->callback.connected) {
      CrInt32 num = 0;
      SDK::CrLiveViewProperty* lv_prop = nullptr;
      auto err = SDK::GetLiveViewProperties(handle, &lv_prop, &num);
      if (CR_FAILED(err)) {
        consecutive_capture_failures_.fetch_add(1, std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      SDK::ReleaseLiveViewProperties(handle, lv_prop);

      SDK::CrImageInfo img_info;
      err = SDK::GetLiveViewImageInfo(handle, &img_info);
      if (CR_FAILED(err)) {
        consecutive_capture_failures_.fetch_add(1, std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      CrInt32u buf_size = img_info.GetBufferSize();
      if (buf_size < 1) {
        consecutive_capture_failures_.fetch_add(1, std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      std::vector<CrInt8u> image_buffer(buf_size);
      SDK::CrImageDataBlock image_data;
      image_data.SetSize(buf_size);
      image_data.SetData(image_buffer.data());

      err = SDK::GetLiveViewImage(handle, &image_data);

      if (CR_SUCCEEDED(err) && image_data.GetImageSize() > 0) {
        consecutive_capture_failures_.store(0, std::memory_order_relaxed);
        auto* raw = reinterpret_cast<const uint8_t*>(image_data.GetImageData());
        std::vector<uint8_t> jpeg(raw, raw + image_data.GetImageSize());

        std::lock_guard<std::mutex> lock(stream_mutex_);
        if (frame_callback_) {
          frame_callback_(jpeg, img_info.GetWidth(), img_info.GetHeight());
        }
      } else if (err == SDK::CrWarning_Frame_NotUpdated) {
        // Frame hasn't changed since last poll — not an error
        consecutive_capture_failures_.store(0, std::memory_order_relaxed);
      } else if (err != SDK::CrError_None) {
        consecutive_capture_failures_.fetch_add(1, std::memory_order_relaxed);
        std::cerr << "[SonyCamera] LiveView error: "
                  << sdk_code(static_cast<CrInt32u>(err)) << std::endl;
      }
    }
    streaming_ = false;
  });

  return true;
}

void SonyCameraStreamer::stopStreaming() {
  if (!impl_) {
    streaming_ = false;
    return;
  }

  impl_->stream_stop = true;
  if (impl_->stream_thread.joinable()) {
    impl_->stream_thread.join();
  }
  streaming_ = false;
}

bool SonyCameraStreamer::isConnected() const { return connected_; }

bool SonyCameraStreamer::isInitialized() const { return initialized_; }

uint64_t SonyCameraStreamer::consecutiveCaptureFailures() const {
  return consecutive_capture_failures_.load(std::memory_order_relaxed);
}

}  // namespace sony_camera_pkg
