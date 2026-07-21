/**
 * @file gimbal_interface_impl.cpp
 * @brief 云台接口的工厂函数实现（仿真和真实硬件）。
 *
 * DJIRS2Gimbal: 通过 SocketCAN + DJI R SDK 协议控制真实 RS2 云台。
 * SimulatedGimbal: 内存仿真，用于无硬件时的开发测试。
 */

#include "robot_platform_pkg/hardware_interfaces/gimbal_interface.hpp"
#include "robot_platform_pkg/protocol/dji_rs2_protocol.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

// Linux SocketCAN
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_platform_pkg {

// ═══════════════════════════════════════════════════════════════════════
// DJIRS2Gimbal — 真实 DJI RS2 云台（SocketCAN + DJI R SDK 协议）
// ═══════════════════════════════════════════════════════════════════════

class DJIRS2Gimbal : public IGimbalInterface {
public:
  DJIRS2Gimbal() : sock_(-1), stopped_(false), connected_(false) {}

  ~DJIRS2Gimbal() override { shutdown(); }

  bool init(const std::string& can_interface) override {
    // DJI RS2 R SDK over CAN uses fixed IDs: host->gimbal 0x223, gimbal->host 0x222.
    can_if_name_ = can_interface;

    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
      std::cerr << "[DJIRS2Gimbal] 创建 SocketCAN 套接字失败: " << strerror(errno) << std::endl;
      return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_if_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      std::cerr << "[DJIRS2Gimbal] CAN 接口 '" << can_if_name_
                << "' 未找到: " << strerror(errno) << std::endl;
      close(sock_);
      sock_ = -1;
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      std::cerr << "[DJIRS2Gimbal] SocketCAN bind 失败: " << strerror(errno) << std::endl;
      close(sock_);
      sock_ = -1;
      return false;
    }

    // 设置接收过滤器：仅接收 CAN ID 0x222（gimbal → host 响应帧）
    struct can_filter rfilter;
    rfilter.can_id = dji_rs2::RECV_CAN_ID;
    rfilter.can_mask = CAN_SFF_MASK;
    setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    // 设置接收超时为 100ms，避免 recv_thread_ 忙等
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100 ms
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 初始化控制字节
    position_ctrl_byte_ = dji_rs2::BIT_ABS_CTRL;  // 绝对控制模式
    speed_ctrl_byte_ = dji_rs2::BIT_SPEED_ENABLE | dji_rs2::BIT_FOCAL_DISABLE;

    connected_ = true;
    stopped_ = false;
    recv_thread_ = std::thread(&DJIRS2Gimbal::recvLoop, this);

    std::cout << "[DJIRS2Gimbal] 已连接 '" << can_if_name_
              << "' (发→0x223, 收←0x222)" << std::endl;
    return true;
  }

  void sendCommand(const vision_servo_msgs::msg::GimbalCmd& cmd) override {
    if (!connected_) return;

    // rad/s → 0.1 deg/s，限幅在 [-3600, 3600] (即 ±360 deg/s)
    // DJI SDK 将速度声明为 uint16_t 但直接按原始字节传输；
    // 我们将 int16 有符号值 reinterpret 为 uint16 位模式，
    // 云台固件端解释为有符号值即可实现双向控制。
    constexpr double RAD2TENTH_DEG = 1800.0 / M_PI;  // 1 rad/s ≈ 573 tenth-deg/s

    uint16_t yaw_rate = 0;
    uint16_t pitch_rate = 0;

    if (!cmd.hold_yaw) {
      double tenth_deg = std::clamp(cmd.yaw_rate * RAD2TENTH_DEG, -3600.0, 3600.0);
      yaw_rate = static_cast<uint16_t>(static_cast<int16_t>(tenth_deg));
    }
    if (!cmd.hold_pitch) {
      double tenth_deg = std::clamp(cmd.pitch_rate * RAD2TENTH_DEG, -3600.0, 3600.0);
      pitch_rate = static_cast<uint16_t>(static_cast<int16_t>(tenth_deg));
    }

    auto frame = dji_rs2::build_speed_command(yaw_rate, 0, pitch_rate, speed_ctrl_byte_);
    sendFrame(frame);
  }

  void readState(float& yaw, float& pitch,
                 float& yaw_rate, float& pitch_rate) override {
    // 周期性查询云台位置（10 Hz），响应由接收线程异步缓存
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_query_time_).count() >= 100) {
      last_query_time_ = now;
      auto query_frame = dji_rs2::build_query_position_command();
      recordPendingQuery(query_frame);
      sendFrame(query_frame);
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    constexpr double TENTH_DEG2RAD = M_PI / 1800.0;  // 0.1 deg → rad
    yaw = static_cast<float>(cached_yaw_tenth_deg_ * TENTH_DEG2RAD);
    pitch = static_cast<float>(cached_pitch_tenth_deg_ * TENTH_DEG2RAD);
    if (hasRecentFeedbackLocked(std::chrono::steady_clock::now())) {
      yaw_rate = cached_yaw_rate_rad_s_;
      pitch_rate = cached_pitch_rate_rad_s_;
    } else {
      yaw_rate = 0.0f;
      pitch_rate = 0.0f;
    }
  }

  void home() override {
    sendPositionCommand(0.0f, 0.0f, 0.5f);
    std::cout << "[DJIRS2Gimbal] 回中指令已发送" << std::endl;
  }

  void sendPositionCommand(float yaw, float pitch, float duration_sec) override {
    sendPositionFrame(yaw, pitch, duration_sec, position_ctrl_byte_, "位置");
  }

  void sendIncrementalPositionCommand(
    float yaw_delta, float pitch_delta, float duration_sec) override {
    sendPositionFrame(yaw_delta, pitch_delta, duration_sec, 0x00, "增量位置");
  }

  void emergencyStop() override {
    if (!connected_) return;
    // 发送零速指令
    auto frame = dji_rs2::build_speed_command(0, 0, 0, speed_ctrl_byte_);
    sendFrame(frame);
    std::cout << "[DJIRS2Gimbal] 急停指令已发送" << std::endl;
  }

  void shutdown() override {
    if (!connected_) return;
    emergencyStop();
    stopped_ = true;
    if (recv_thread_.joinable())
      recv_thread_.join();
    if (sock_ >= 0) {
      close(sock_);
      sock_ = -1;
    }
    connected_ = false;
    std::cout << "[DJIRS2Gimbal] 已关闭" << std::endl;
  }

  bool isConnected() const override {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return connected_ && hasRecentFeedbackLocked(std::chrono::steady_clock::now());
  }

  GimbalStatusSnapshot getStatus() const override {
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    GimbalStatusSnapshot status;
    status.connected = connected_ && hasRecentFeedbackLocked(now);
    status.last_rx_age_sec = lastRxAgeSecLocked(now);
    status.tx_count = tx_count_.load();
    status.rx_count = rx_count_.load();
    status.crc_error_count = crc_error_count_.load();
    status.can_error_count = can_error_count_.load();
    status.parse_error_count = parse_error_count_.load();
    return status;
  }

private:
  static constexpr double kTenthDegToRad = M_PI / 1800.0;
  static constexpr double kFeedbackTimeoutSec = 1.0;

  void sendPositionFrame(
    float yaw, float pitch, float duration_sec, uint8_t ctrl_byte, const char* label) {
    if (!connected_) return;

    constexpr double RAD2TENTH_DEG = 1800.0 / M_PI;
    const auto yaw_tenth = static_cast<int16_t>(
      std::clamp(std::lround(static_cast<double>(yaw) * RAD2TENTH_DEG), -1800L, 1800L));
    const auto pitch_tenth = static_cast<int16_t>(
      std::clamp(std::lround(static_cast<double>(pitch) * RAD2TENTH_DEG), -1800L, 1800L));
    const auto time_ms = static_cast<uint16_t>(
      std::clamp(std::lround(static_cast<double>(duration_sec) * 1000.0), 100L, 10000L));

    auto frame = dji_rs2::build_position_command(
      yaw_tenth, 0, pitch_tenth, time_ms, ctrl_byte);
    sendFrame(frame);
    // Do not print every control frame: at 50 Hz synchronous terminal output
    // adds measurable jitter to the command path. Keep a 1 Hz diagnostic.
    const auto now = std::chrono::steady_clock::now();
    if (now - last_position_log_time_ >= std::chrono::seconds(1)) {
      last_position_log_time_ = now;
      std::cout << "[DJIRS2Gimbal] " << label << "指令 yaw=" << yaw_tenth
                << " pitch=" << pitch_tenth << " time_ms=" << time_ms
                << " ctrl=0x" << std::hex << static_cast<int>(ctrl_byte)
                << std::dec << std::endl;
    }
  }

  static int normalizeTenthDegDelta(int current, int previous) {
    int delta = current - previous;
    while (delta > 1800) {
      delta -= 3600;
    }
    while (delta < -1800) {
      delta += 3600;
    }
    return delta;
  }

  bool hasRecentFeedbackLocked(std::chrono::steady_clock::time_point now) const {
    if (!has_state_sample_) {
      return false;
    }
    return std::chrono::duration<double>(now - last_state_update_time_).count()
      <= kFeedbackTimeoutSec;
  }

  float lastRxAgeSecLocked(std::chrono::steady_clock::time_point now) const {
    if (!has_state_sample_) {
      return -1.0f;
    }
    return static_cast<float>(
      std::chrono::duration<double>(now - last_state_update_time_).count());
  }

  void recordPendingQuery(const std::vector<uint8_t>& frame) {
    if (frame.size() < 10) {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_query_seq_hi_ = frame[8];
    pending_query_seq_lo_ = frame[9];
    has_pending_query_seq_ = true;
  }

  bool matchesPendingQuery(const std::vector<uint8_t>& frame) const {
    if (frame.size() < 10 || frame[3] != dji_rs2::RSP_TYPE) {
      return true;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    return has_pending_query_seq_ &&
      frame[8] == pending_query_seq_hi_ &&
      frame[9] == pending_query_seq_lo_;
  }

  /// 通过 SocketCAN 发送完整协议帧（自动分片为 8 字节 CAN 帧）
  void sendFrame(const std::vector<uint8_t>& data) {
    if (sock_ < 0) return;

    size_t offset = 0;
    while (offset < data.size()) {
      struct can_frame frame;
      std::memset(&frame, 0, sizeof(frame));
      frame.can_id = dji_rs2::SEND_CAN_ID;
      frame.can_dlc = static_cast<uint8_t>(std::min(data.size() - offset, size_t(8)));
      std::memcpy(frame.data, data.data() + offset, frame.can_dlc);

      ssize_t n = write(sock_, &frame, sizeof(frame));
      if (n != static_cast<ssize_t>(sizeof(frame))) {
        can_error_count_++;
        std::cerr << "[DJIRS2Gimbal] CAN 发送失败: " << strerror(errno) << std::endl;
        break;
      }
      tx_count_++;
      offset += frame.can_dlc;
    }
  }

  /// 接收线程 — 从 SocketCAN 读取帧，重组协议帧，解析位置数据
  void recvLoop() {
    // 接收缓冲区和协议帧重组状态机
    std::vector<uint8_t> byte_buffer;
    std::vector<uint8_t> work_frame;
    size_t pack_len = 0;
    int step = 0;

    while (!stopped_) {
      struct can_frame frame;
      ssize_t n = read(sock_, &frame, sizeof(frame));
      if (n <= 0) {
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          can_error_count_++;
        }
        continue;
      }

      // 过滤：仅处理响应帧 CAN ID (0x222)
      if ((frame.can_id & CAN_SFF_MASK) != dji_rs2::RECV_CAN_ID)
        continue;
      rx_count_++;

      // 将 CAN 帧数据追加到字节缓冲区
      for (int i = 0; i < frame.can_dlc; ++i)
        byte_buffer.push_back(frame.data[i]);

      // 协议帧重组状态机（与 DJIR_SDK Handle.cpp 一致）
      size_t pos = 0;
      while (pos < byte_buffer.size()) {
        uint8_t b = byte_buffer[pos];

        switch (step) {
        case 0:  // 搜索 SOF
          if (b == dji_rs2::SOF) {
            work_frame.push_back(b);
            step = 1;
          }
          pos++;
          break;

        case 1:  // 长度低字节
          pack_len = b;
          work_frame.push_back(b);
          step = 2;
          pos++;
          break;

        case 2:  // 长度高字节 (仅低 2-bit)
          pack_len |= ((b & 0x03) << 8);
          work_frame.push_back(b);
          step = 3;
          pos++;
          break;

        case 3:  // 读取至 12 字节，验证 CRC-16 header
          work_frame.push_back(b);
          pos++;
          if (work_frame.size() == 12) {
            if (checkHeadCrc(work_frame)) {
              step = 4;
            } else {
              crc_error_count_++;
              step = 0;
              work_frame.clear();
            }
          }
          break;

        case 4:  // 读取至完整帧，验证 CRC-32
          work_frame.push_back(b);
          pos++;
          if (work_frame.size() == pack_len) {
            if (checkPackCrc(work_frame)) {
              processFrame(work_frame);
            } else {
              crc_error_count_++;
            }
            work_frame.clear();
            step = 0;
          }
          break;

        default:
          step = 0;
          work_frame.clear();
          pos++;
          break;
        }
      }

      // 清除已处理的字节
      byte_buffer.erase(byte_buffer.begin(), byte_buffer.begin() + pos);
    }
  }

  /// 验证帧头 CRC-16 (覆盖字节 0..9 → 对照字节 10-11)
  static bool checkHeadCrc(const std::vector<uint8_t>& data) {
    if (data.size() < 12) return false;
    uint16_t computed = dji_rs2::crc16_compute(data.data(), 10);
    uint16_t received = static_cast<uint16_t>(data[10] | (data[11] << 8));
    return computed == received;
  }

  /// 验证整帧 CRC-32 (覆盖字节 0..N-5 → 对照最后 4 字节)
  static bool checkPackCrc(const std::vector<uint8_t>& data) {
    if (data.size() < 8) return false;
    uint32_t computed = dji_rs2::crc32_compute(data.data(), data.size() - 4);
    uint32_t received =
        static_cast<uint32_t>(data[data.size() - 4] |
                              (data[data.size() - 3] << 8) |
                              (data[data.size() - 2] << 16) |
                              (data[data.size() - 1] << 24));
    return computed == received;
  }

  /// 解析帧并更新缓存的位置状态
  void processFrame(const std::vector<uint8_t>& frame) {
    if (!matchesPendingQuery(frame)) {
      return;
    }

    int16_t yaw, roll, pitch;
    if (dji_rs2::parse_position_response(frame, yaw, roll, pitch)) {
      const auto now = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (has_state_sample_) {
        const double dt = std::chrono::duration<double>(
          now - last_state_update_time_).count();
        if (dt > 1e-4) {
          cached_yaw_rate_rad_s_ = static_cast<float>(
            normalizeTenthDegDelta(yaw, cached_yaw_tenth_deg_) * kTenthDegToRad / dt);
          cached_pitch_rate_rad_s_ = static_cast<float>(
            normalizeTenthDegDelta(pitch, cached_pitch_tenth_deg_) * kTenthDegToRad / dt);
        }
      } else {
        cached_yaw_rate_rad_s_ = 0.0f;
        cached_pitch_rate_rad_s_ = 0.0f;
      }
      cached_yaw_tenth_deg_ = yaw;
      cached_roll_tenth_deg_ = roll;
      cached_pitch_tenth_deg_ = pitch;
      last_state_update_time_ = now;
      has_state_sample_ = true;
      has_pending_query_seq_ = false;
    } else {
      parse_error_count_++;
    }
  }

  int sock_;                      ///< SocketCAN 文件描述符
  std::string can_if_name_;       ///< CAN 接口名（如 "can0"）
  std::thread recv_thread_;       ///< 接收线程
  std::atomic<bool> stopped_;
  std::atomic<bool> connected_;
  std::atomic<uint32_t> tx_count_{0};
  std::atomic<uint32_t> rx_count_{0};
  std::atomic<uint32_t> crc_error_count_{0};
  std::atomic<uint32_t> can_error_count_{0};
  std::atomic<uint32_t> parse_error_count_{0};

  uint8_t position_ctrl_byte_;    ///< 位置控制标志
  uint8_t speed_ctrl_byte_;       ///< 速度控制标志
  std::chrono::steady_clock::time_point last_query_time_{};
  std::chrono::steady_clock::time_point last_position_log_time_{};

  // 缓存的云台状态 (0.1° 单位)
  mutable std::mutex state_mutex_;
  int16_t cached_yaw_tenth_deg_ = 0;
  int16_t cached_roll_tenth_deg_ = 0;
  int16_t cached_pitch_tenth_deg_ = 0;
  float cached_yaw_rate_rad_s_ = 0.0f;
  float cached_pitch_rate_rad_s_ = 0.0f;
  std::chrono::steady_clock::time_point last_state_update_time_{};
  bool has_state_sample_ = false;
  uint8_t pending_query_seq_hi_ = 0;
  uint8_t pending_query_seq_lo_ = 0;
  bool has_pending_query_seq_ = false;
};

// ═══════════════════════════════════════════════════════════════════════
// SimulatedGimbal — 仿真云台
// ═══════════════════════════════════════════════════════════════════════

class SimulatedGimbal : public IGimbalInterface {
public:
  bool init(const std::string& can_interface) override {
    (void)can_interface;
    last_update_ = Clock::now();
    std::cout << "[SimGimbal] 初始化（仿真模式）" << std::endl;
    return true;
  }

  void sendCommand(const vision_servo_msgs::msg::GimbalCmd& cmd) override {
    std::lock_guard<std::mutex> lock(mutex_);
    integrate_locked();

    yaw_rate_ = cmd.hold_yaw ? 0.0f : clamp(cmd.yaw_rate, -max_yaw_rate_, max_yaw_rate_);
    pitch_rate_ = cmd.hold_pitch ? 0.0f : clamp(cmd.pitch_rate, -max_pitch_rate_, max_pitch_rate_);
    tx_count_++;
  }

  void readState(float& yaw, float& pitch,
                 float& yaw_rate, float& pitch_rate) override {
    std::lock_guard<std::mutex> lock(mutex_);
    integrate_locked();
    yaw = yaw_;
    pitch = pitch_;
    yaw_rate = yaw_rate_;
    pitch_rate = pitch_rate_;
    rx_count_++;
  }

  void home() override {
    std::lock_guard<std::mutex> lock(mutex_);
    yaw_ = 0.0f;
    pitch_ = 0.0f;
    yaw_rate_ = 0.0f;
    pitch_rate_ = 0.0f;
    last_update_ = Clock::now();
  }

  void sendPositionCommand(float yaw, float pitch, float duration_sec) override {
    (void)duration_sec;
    std::lock_guard<std::mutex> lock(mutex_);
    integrate_locked();
    yaw_ = clamp(yaw, -yaw_limit_, yaw_limit_);
    pitch_ = clamp(pitch, -pitch_limit_, pitch_limit_);
    yaw_rate_ = 0.0f;
    pitch_rate_ = 0.0f;
    tx_count_++;
  }

  void sendIncrementalPositionCommand(
    float yaw_delta, float pitch_delta, float duration_sec) override {
    (void)duration_sec;
    std::lock_guard<std::mutex> lock(mutex_);
    integrate_locked();
    yaw_ = clamp(yaw_ + yaw_delta, -yaw_limit_, yaw_limit_);
    pitch_ = clamp(pitch_ + pitch_delta, -pitch_limit_, pitch_limit_);
    yaw_rate_ = 0.0f;
    pitch_rate_ = 0.0f;
    tx_count_++;
  }

  void emergencyStop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    integrate_locked();
    yaw_rate_ = 0.0f;
    pitch_rate_ = 0.0f;
  }

  void shutdown() override { emergencyStop(); }
  bool isConnected() const override { return true; }

  GimbalStatusSnapshot getStatus() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    GimbalStatusSnapshot status;
    status.connected = true;
    status.last_rx_age_sec = 0.0f;
    status.tx_count = tx_count_;
    status.rx_count = rx_count_;
    return status;
  }

private:
  using Clock = std::chrono::steady_clock;

  static float clamp(float value, float lower, float upper) {
    return std::max(lower, std::min(upper, value));
  }

  void integrate_locked() {
    const auto now = Clock::now();
    const float dt = std::chrono::duration<float>(now - last_update_).count();
    last_update_ = now;

    yaw_ = clamp(yaw_ + yaw_rate_ * dt, -yaw_limit_, yaw_limit_);
    pitch_ = clamp(pitch_ + pitch_rate_ * dt, -pitch_limit_, pitch_limit_);
  }

  mutable std::mutex mutex_;
  Clock::time_point last_update_{Clock::now()};
  float yaw_ = 0.0f;
  float pitch_ = 0.0f;
  float yaw_rate_ = 0.0f;
  float pitch_rate_ = 0.0f;
  float yaw_limit_ = 2.6f;
  float pitch_limit_ = 1.2f;
  float max_yaw_rate_ = 1.0f;
  float max_pitch_rate_ = 1.0f;
  uint32_t tx_count_ = 0;
  uint32_t rx_count_ = 0;
};

// ── 工厂函数 ──────────────────────────────────────────────────────────

std::unique_ptr<IGimbalInterface> make_dji_rs2_gimbal() {
  return std::make_unique<DJIRS2Gimbal>();
}

std::unique_ptr<IGimbalInterface> make_simulated_gimbal() {
  return std::make_unique<SimulatedGimbal>();
}

}  // namespace robot_platform_pkg
