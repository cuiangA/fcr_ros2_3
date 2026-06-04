/**
 * @file gimbal_interface_impl.cpp
 * @brief 云台接口的工厂函数实现（仿真和真实硬件）。
 */

#include "robot_platform_pkg/hardware_interfaces/gimbal_interface.hpp"
#include <iostream>

namespace robot_platform_pkg {

// ── 云台仿真实现 ──────────────────────────────────────────────────
class SimulatedGimbal : public IGimbalInterface {
public:
  bool init(const std::string& can_interface, int can_id) override {
    (void)can_interface; (void)can_id;
    std::cout << "[SimGimbal] 初始化（仿真模式）" << std::endl;
    return true;
  }
  void sendCommand(const vision_servo_msgs::msg::GimbalCmd& cmd) override {
    (void)cmd;
    // 仿真：不实际发送，仅更新内部状态
  }
  void readState(float& yaw, float& pitch,
                 float& yaw_rate, float& pitch_rate) override {
    yaw = 0.0f;
    pitch = 0.0f;
    yaw_rate = 0.0f;
    pitch_rate = 0.0f;
  }
  void home() override {}
  void emergencyStop() override {}
  void shutdown() override {}
  bool isConnected() const override { return true; }
};

std::unique_ptr<IGimbalInterface> make_dji_rs2_gimbal() {
  // TODO: 实现真实 DJI RS2 云台 CAN 总线通信
  // 参考协议：DJI RS2 CAN 总线协议 v2.1
  return nullptr;
}

std::unique_ptr<IGimbalInterface> make_simulated_gimbal() {
  return std::make_unique<SimulatedGimbal>();
}

}  // namespace robot_platform_pkg
