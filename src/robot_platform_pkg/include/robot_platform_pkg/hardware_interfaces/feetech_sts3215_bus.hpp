#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace robot_platform_pkg {

/** Minimal Linux serial driver for the Feetech STS3215 protocol used by LeKiwi. */
class FeetechSts3215Bus {
public:
  FeetechSts3215Bus() = default;
  ~FeetechSts3215Bus();

  FeetechSts3215Bus(const FeetechSts3215Bus&) = delete;
  FeetechSts3215Bus& operator=(const FeetechSts3215Bus&) = delete;

  bool open(const std::string& device, int baudrate);
  void close();
  bool isOpen() const;
  const std::string& lastError() const;

  bool ping(uint8_t id);
  bool configureVelocityMode(const std::array<uint8_t, 3>& ids);
  bool syncWriteVelocity(const std::array<uint8_t, 3>& ids,
                         const std::array<int16_t, 3>& velocities);
  bool readVelocity(uint8_t id, int16_t& velocity);
  bool readVoltage(uint8_t id, float& volts);
  bool stop(const std::array<uint8_t, 3>& ids);

  static uint16_t encodeSignedMagnitude(int16_t value);
  static int16_t decodeSignedMagnitude(uint16_t value);

private:
  bool writeRegister(uint8_t id, uint8_t address, const uint8_t* data, size_t size);
  bool readRegister(uint8_t id, uint8_t address, uint8_t* data, size_t size);
  bool transmit(const uint8_t* data, size_t size);
  bool receiveStatus(uint8_t expected_id, uint8_t* data, size_t size);
  void setError(const std::string& message);

  int fd_{-1};
  std::string last_error_;
};

}  // namespace robot_platform_pkg
