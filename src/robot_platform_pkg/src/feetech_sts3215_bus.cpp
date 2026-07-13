#include "robot_platform_pkg/hardware_interfaces/feetech_sts3215_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <sstream>
#include <vector>

#ifdef __linux__
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace robot_platform_pkg {
namespace {
constexpr uint8_t kHeader = 0xFF;
constexpr uint8_t kInstructionPing = 0x01;
constexpr uint8_t kInstructionRead = 0x02;
constexpr uint8_t kInstructionWrite = 0x03;
constexpr uint8_t kInstructionSyncWrite = 0x83;
constexpr uint8_t kBroadcastId = 0xFE;

// STS3215 protocol-0 control table. These are the same registers used by LeRobot.
constexpr uint8_t kReturnDelayTime = 7;
constexpr uint8_t kPhase = 18;
constexpr uint8_t kOperatingMode = 33;
constexpr uint8_t kTorqueEnable = 40;
constexpr uint8_t kAcceleration = 41;
constexpr uint8_t kGoalVelocity = 46;
constexpr uint8_t kPresentVelocity = 58;
constexpr uint8_t kPresentVoltage = 62;
constexpr uint8_t kMaximumAcceleration = 85;
constexpr uint8_t kVelocityMode = 1;

uint8_t checksum(const uint8_t* bytes, size_t begin, size_t end) {
  unsigned sum = 0;
  for (size_t i = begin; i < end; ++i) sum += bytes[i];
  return static_cast<uint8_t>(~sum);
}
}  // namespace

FeetechSts3215Bus::~FeetechSts3215Bus() { close(); }

bool FeetechSts3215Bus::open(const std::string& device, int baudrate) {
#ifndef __linux__
  (void)device;
  (void)baudrate;
  setError("Feetech serial transport is supported only on Linux");
  return false;
#else
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    setError("open(" + device + "): " + std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    setError("tcgetattr: " + std::string(std::strerror(errno)));
    close();
    return false;
  }
  cfmakeraw(&tty);
  speed_t speed{};
  switch (baudrate) {
    case 115200: speed = B115200; break;
#ifdef B500000
    case 500000: speed = B500000; break;
#endif
#ifdef B1000000
    case 1000000: speed = B1000000; break;
#endif
    default:
      setError("unsupported baudrate: " + std::to_string(baudrate));
      close();
      return false;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(CSTOPB | CRTSCTS);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    setError("tcsetattr: " + std::string(std::strerror(errno)));
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  last_error_.clear();
  return true;
#endif
}

void FeetechSts3215Bus::close() {
#ifdef __linux__
  if (fd_ >= 0) ::close(fd_);
#endif
  fd_ = -1;
}

bool FeetechSts3215Bus::isOpen() const { return fd_ >= 0; }
const std::string& FeetechSts3215Bus::lastError() const { return last_error_; }
void FeetechSts3215Bus::setError(const std::string& message) { last_error_ = message; }

bool FeetechSts3215Bus::transmit(const uint8_t* data, size_t size) {
#ifndef __linux__
  (void)data; (void)size; return false;
#else
  if (!isOpen()) { setError("serial port is closed"); return false; }
  tcflush(fd_, TCIFLUSH);
  size_t sent = 0;
  while (sent < size) {
    const ssize_t n = ::write(fd_, data + sent, size - sent);
    if (n > 0) { sent += static_cast<size_t>(n); continue; }
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) continue;
    setError("serial write: " + std::string(std::strerror(errno)));
    return false;
  }
  if (tcdrain(fd_) != 0) { setError("tcdrain failed"); return false; }
  return true;
#endif
}

bool FeetechSts3215Bus::receiveStatus(uint8_t expected_id, uint8_t* data, size_t size) {
#ifndef __linux__
  (void)expected_id; (void)data; (void)size; return false;
#else
  std::vector<uint8_t> packet;
  packet.reserve(size + 6);
  const auto deadline_ms = 100;
  while (true) {
    pollfd pfd{fd_, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, deadline_ms);
    if (ready <= 0) { setError(ready == 0 ? "servo response timeout" : "poll failed"); return false; }
    uint8_t buffer[64];
    const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
    if (n <= 0) continue;
    packet.insert(packet.end(), buffer, buffer + n);

    while (packet.size() >= 2 && !(packet[0] == kHeader && packet[1] == kHeader)) packet.erase(packet.begin());
    if (packet.size() < 4) continue;
    const size_t total = static_cast<size_t>(packet[3]) + 4;
    if (packet.size() < total) continue;
    if (packet[2] != expected_id || packet[3] != size + 2) { setError("unexpected servo status packet"); return false; }
    if (packet[total - 1] != checksum(packet.data(), 2, total - 1)) { setError("servo checksum mismatch"); return false; }
    if (packet[4] != 0) { setError("servo reported error bits: " + std::to_string(packet[4])); return false; }
    if (size > 0) std::copy_n(packet.begin() + 5, size, data);
    return true;
  }
#endif
}

bool FeetechSts3215Bus::ping(uint8_t id) {
  uint8_t packet[] = {kHeader, kHeader, id, 2, kInstructionPing, 0};
  packet[5] = checksum(packet, 2, 5);
  if (!transmit(packet, sizeof(packet))) return false;
  return receiveStatus(id, nullptr, 0);
}

bool FeetechSts3215Bus::writeRegister(uint8_t id, uint8_t address, const uint8_t* data, size_t size) {
  std::vector<uint8_t> packet{kHeader, kHeader, id, static_cast<uint8_t>(size + 3), kInstructionWrite, address};
  packet.insert(packet.end(), data, data + size);
  packet.push_back(checksum(packet.data(), 2, packet.size()));
  if (!transmit(packet.data(), packet.size())) return false;
  return receiveStatus(id, nullptr, 0);
}

bool FeetechSts3215Bus::readRegister(uint8_t id, uint8_t address, uint8_t* data, size_t size) {
  uint8_t packet[] = {kHeader, kHeader, id, 4, kInstructionRead, address, static_cast<uint8_t>(size), 0};
  packet[7] = checksum(packet, 2, 7);
  return transmit(packet, sizeof(packet)) && receiveStatus(id, data, size);
}

bool FeetechSts3215Bus::configureVelocityMode(const std::array<uint8_t, 3>& ids) {
  for (const auto id : ids) {
    const uint8_t off = 0;
    const uint8_t return_delay = 0;
    const uint8_t acceleration = 254;
    uint8_t phase = 0;
    if (!writeRegister(id, kReturnDelayTime, &return_delay, 1) ||
        !writeRegister(id, kTorqueEnable, &off, 1) ||
        !readRegister(id, kPhase, &phase, 1)) return false;
    if (phase & 0x10) {
      phase = static_cast<uint8_t>(phase & ~0x10);
      if (!writeRegister(id, kPhase, &phase, 1)) return false;
    }
    if (!writeRegister(id, kOperatingMode, &kVelocityMode, 1) ||
        !writeRegister(id, kMaximumAcceleration, &acceleration, 1) ||
        !writeRegister(id, kAcceleration, &acceleration, 1)) return false;
    const uint8_t on = 1;
    if (!writeRegister(id, kTorqueEnable, &on, 1)) return false;
  }
  return true;
}

uint16_t FeetechSts3215Bus::encodeSignedMagnitude(int16_t value) {
  const uint16_t magnitude = static_cast<uint16_t>(std::min<int>(std::abs(static_cast<int>(value)), 0x7FFF));
  return value < 0 ? static_cast<uint16_t>(magnitude | 0x8000) : magnitude;
}

int16_t FeetechSts3215Bus::decodeSignedMagnitude(uint16_t value) {
  const int16_t magnitude = static_cast<int16_t>(value & 0x7FFF);
  return (value & 0x8000) ? static_cast<int16_t>(-magnitude) : magnitude;
}

bool FeetechSts3215Bus::syncWriteVelocity(const std::array<uint8_t, 3>& ids,
                                          const std::array<int16_t, 3>& velocities) {
  std::vector<uint8_t> packet{kHeader, kHeader, kBroadcastId, 0, kInstructionSyncWrite, kGoalVelocity, 2};
  for (size_t i = 0; i < ids.size(); ++i) {
    const uint16_t raw = encodeSignedMagnitude(velocities[i]);
    packet.push_back(ids[i]);
    packet.push_back(static_cast<uint8_t>(raw & 0xFF));
    packet.push_back(static_cast<uint8_t>(raw >> 8));
  }
  packet[3] = static_cast<uint8_t>(packet.size() - 3);
  packet.push_back(checksum(packet.data(), 2, packet.size()));
  return transmit(packet.data(), packet.size());  // Broadcast instructions have no status response.
}

bool FeetechSts3215Bus::readVelocity(uint8_t id, int16_t& velocity) {
  uint8_t bytes[2]{};
  if (!readRegister(id, kPresentVelocity, bytes, sizeof(bytes))) return false;
  velocity = decodeSignedMagnitude(static_cast<uint16_t>(bytes[0] | (bytes[1] << 8)));
  return true;
}

bool FeetechSts3215Bus::readVoltage(uint8_t id, float& volts) {
  uint8_t raw{};
  if (!readRegister(id, kPresentVoltage, &raw, 1)) return false;
  volts = static_cast<float>(raw) / 10.0F;
  return true;
}

bool FeetechSts3215Bus::stop(const std::array<uint8_t, 3>& ids) {
  return syncWriteVelocity(ids, {0, 0, 0});
}

}  // namespace robot_platform_pkg
