/**
 * @file chassis_interface_impl.cpp
 * @brief Simulated and Feetech STS3215 implementations of the LeKiwi chassis.
 */

#include "robot_platform_pkg/hardware_interfaces/chassis_interface.hpp"
#include "robot_platform_pkg/hardware_interfaces/feetech_sts3215_bus.hpp"

#include <Eigen/Core>
#include <Eigen/QR>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <mutex>
#include <stdexcept>

namespace robot_platform_pkg {
namespace {
constexpr double kSts3215StepsPerRevolution = 4096.0;

class LekiwiChassis final : public IChassisInterface {
public:
  explicit LekiwiChassis(LekiwiChassisConfig config)
    : config_(config),
      ids_{static_cast<uint8_t>(config.left_wheel_id),
           static_cast<uint8_t>(config.back_wheel_id),
           static_cast<uint8_t>(config.right_wheel_id)} {
    if (!validConfig()) throw std::invalid_argument("invalid LeKiwi chassis configuration");
    // LeRobot's verified LeKiwi order is left/back/right at 150/-90/30 degrees.
    const std::array<double, 3> angles{5.0 * M_PI / 6.0, -M_PI / 2.0, M_PI / 6.0};
    for (size_t i = 0; i < angles.size(); ++i) {
      inverse_kinematics_(i, 0) = std::cos(angles[i]) / config_.wheel_radius;
      inverse_kinematics_(i, 1) = std::sin(angles[i]) / config_.wheel_radius;
      inverse_kinematics_(i, 2) = config_.base_radius / config_.wheel_radius;
    }
    forward_kinematics_ =
      inverse_kinematics_.completeOrthogonalDecomposition().pseudoInverse();
  }

  ~LekiwiChassis() override { shutdown(); }

  bool init(const std::string& device, int baudrate) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!bus_.open(device, baudrate)) {
      std::cerr << "[LekiwiChassis] " << bus_.lastError() << '\n';
      return false;
    }
    for (const auto id : ids_) {
      if (!bus_.ping(id)) {
        std::cerr << "[LekiwiChassis] wheel ID " << static_cast<int>(id)
                  << " not responding: " << bus_.lastError() << '\n';
        bus_.close();
        return false;
      }
    }
    if (!bus_.configureVelocityMode(ids_) || !bus_.stop(ids_)) {
      std::cerr << "[LekiwiChassis] motor configuration failed: " << bus_.lastError() << '\n';
      bus_.close();
      return false;
    }
    connected_ = true;
    return true;
  }

  void sendCommand(const geometry_msgs::msg::Twist& cmd) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_) return;
    const Eigen::Vector3d body(cmd.linear.x, cmd.linear.y, cmd.angular.z);
    Eigen::Vector3d wheel_rad_s = inverse_kinematics_ * body;

    // STS3215 velocity units used by LeRobot: 4096 raw steps per revolution/second.
    Eigen::Vector3d raw = wheel_rad_s * (kSts3215StepsPerRevolution / (2.0 * M_PI));
    const double peak = raw.cwiseAbs().maxCoeff();
    if (peak > config_.max_raw_velocity) raw *= config_.max_raw_velocity / peak;

    const std::array<int16_t, 3> command{
      clampRaw(raw[0]), clampRaw(raw[1]), clampRaw(raw[2])};
    if (!bus_.syncWriteVelocity(ids_, command)) {
      connected_ = false;
      std::cerr << "[LekiwiChassis] velocity write failed: " << bus_.lastError() << '\n';
    }
  }

  nav_msgs::msg::Odometry readOdometry() override {
    std::lock_guard<std::mutex> lock(mutex_);
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.orientation.w = 1.0;
    if (!connected_) return odom;

    std::array<int16_t, 3> raw{};
    for (size_t i = 0; i < ids_.size(); ++i) {
      if (!bus_.readVelocity(ids_[i], raw[i])) {
        connected_ = false;
        std::cerr << "[LekiwiChassis] velocity read failed: " << bus_.lastError() << '\n';
        bus_.stop(ids_);
        return odom;
      }
    }
    const double raw_to_rad_s = 2.0 * M_PI / kSts3215StepsPerRevolution;
    const Eigen::Vector3d wheel(raw[0] * raw_to_rad_s,
                                raw[1] * raw_to_rad_s,
                                raw[2] * raw_to_rad_s);
    const Eigen::Vector3d body = forward_kinematics_ * wheel;
    odom.twist.twist.linear.x = body[0];
    odom.twist.twist.linear.y = body[1];
    odom.twist.twist.angular.z = body[2];
    return odom;
  }

  void emergencyStop() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (bus_.isOpen() && !bus_.stop(ids_)) {
      std::cerr << "[LekiwiChassis] stop failed: " << bus_.lastError() << '\n';
    }
  }

  void shutdown() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (bus_.isOpen()) {
      bus_.stop(ids_);
      bus_.close();
    }
    connected_ = false;
  }

  bool isConnected() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return connected_ && bus_.isOpen();
  }

  float readBatteryVoltage() override {
    std::lock_guard<std::mutex> lock(mutex_);
    float volts = 0.0F;
    if (connected_ && !bus_.readVoltage(ids_[0], volts)) connected_ = false;
    return volts;
  }

private:
  bool validConfig() const {
    const auto valid_id = [](int id) { return id > 0 && id < 254; };
    return valid_id(config_.left_wheel_id) && valid_id(config_.back_wheel_id) &&
           valid_id(config_.right_wheel_id) &&
           config_.left_wheel_id != config_.back_wheel_id &&
           config_.left_wheel_id != config_.right_wheel_id &&
           config_.back_wheel_id != config_.right_wheel_id &&
           config_.wheel_radius > 0.0 && config_.base_radius > 0.0 &&
           config_.max_raw_velocity > 0 && config_.max_raw_velocity <= 32767;
  }

  int16_t clampRaw(double value) const {
    return static_cast<int16_t>(std::lround(std::clamp(
      value, -static_cast<double>(config_.max_raw_velocity),
      static_cast<double>(config_.max_raw_velocity))));
  }

  LekiwiChassisConfig config_;
  std::array<uint8_t, 3> ids_;
  Eigen::Matrix3d inverse_kinematics_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d forward_kinematics_{Eigen::Matrix3d::Zero()};
  mutable std::mutex mutex_;
  FeetechSts3215Bus bus_;
  bool connected_{false};
};

class SimulatedChassis final : public IChassisInterface {
public:
  bool init(const std::string&, int) override { return true; }
  void sendCommand(const geometry_msgs::msg::Twist& cmd) override { last_cmd_ = cmd; }
  nav_msgs::msg::Odometry readOdometry() override {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist = last_cmd_;
    return odom;
  }
  void emergencyStop() override { last_cmd_ = geometry_msgs::msg::Twist(); }
  void shutdown() override { emergencyStop(); }
  bool isConnected() const override { return true; }
  float readBatteryVoltage() override { return 24.0F; }
private:
  geometry_msgs::msg::Twist last_cmd_;
};
}  // namespace

std::unique_ptr<IChassisInterface> make_lekiwi_chassis(const LekiwiChassisConfig& config) {
  return std::make_unique<LekiwiChassis>(config);
}

std::unique_ptr<IChassisInterface> make_simulated_chassis() {
  return std::make_unique<SimulatedChassis>();
}

}  // namespace robot_platform_pkg
