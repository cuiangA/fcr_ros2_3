/**
 * @file imu_interface_impl.cpp
 * @brief IMU 接口的工厂函数实现（仿真和真实硬件）。
 */

#include "robot_platform_pkg/hardware_interfaces/imu_interface.hpp"
#include <iostream>

namespace robot_platform_pkg {

// ── IMU 仿真实现 ──────────────────────────────────────────────────
class SimulatedIMU : public IIMUInterface {
public:
  bool init(const std::string& device, int baudrate) override {
    (void)device; (void)baudrate;
    std::cout << "[SimIMU] 初始化（仿真模式）" << std::endl;
    return true;
  }
  sensor_msgs::msg::Imu read() override {
    sensor_msgs::msg::Imu imu;
    // 仿真：返回零位姿（表示水平、静止状态）
    imu.orientation.w = 1.0;
    imu.orientation.x = 0.0;
    imu.orientation.y = 0.0;
    imu.orientation.z = 0.0;
    imu.angular_velocity.x = 0.0;
    imu.angular_velocity.y = 0.0;
    imu.angular_velocity.z = 0.0;
    imu.linear_acceleration.x = 0.0;
    imu.linear_acceleration.y = 0.0;
    imu.linear_acceleration.z = 9.81;  // 重力加速度
    return imu;
  }
  bool isConnected() const override { return true; }
  void shutdown() override {}
};

std::unique_ptr<IIMUInterface> make_bno055_imu() {
  // TODO: 实现真实 BNO055 IMU I2C 通信
  // 参考数据手册：BNO055 数据手册 rev 1.4
  return nullptr;
}

std::unique_ptr<IIMUInterface> make_simulated_imu() {
  return std::make_unique<SimulatedIMU>();
}

}  // namespace robot_platform_pkg
