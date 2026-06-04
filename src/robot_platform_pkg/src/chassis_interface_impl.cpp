/**
 * @file chassis_interface_impl.cpp
 * @brief 底盘接口的工厂函数实现（仿真和真实硬件）。
 */

#include "robot_platform_pkg/hardware_interfaces/chassis_interface.hpp"
#include <iostream>

namespace robot_platform_pkg {

// ── 底盘仿真实现 ──────────────────────────────────────────────────
class SimulatedChassis : public IChassisInterface {
public:
  bool init(const std::string& device, int baudrate) override {
    (void)device; (void)baudrate;
    std::cout << "[SimChassis] 初始化（仿真模式）" << std::endl;
    return true;
  }
  void sendCommand(const geometry_msgs::msg::Twist& cmd) override {
    (void)cmd;
    // 仿真：不实际发送，仅更新内部状态
  }
  nav_msgs::msg::Odometry readOdometry() override {
    nav_msgs::msg::Odometry odom;
    // 仿真：返回零位姿
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
    return odom;
  }
  void emergencyStop() override {}
  void shutdown() override {}
  bool isConnected() const override { return true; }
  float readBatteryVoltage() override { return 24.0f; }
};

std::unique_ptr<IChassisInterface> make_lekiwi_chassis() {
  // TODO: 实现真实 LEKIWI 底盘串口通信
  // 参考协议：LEKIWI 三全向轮底盘串口协议 v1.2
  return nullptr;  // 暂未实现真实硬件接口
}

std::unique_ptr<IChassisInterface> make_simulated_chassis() {
  return std::make_unique<SimulatedChassis>();
}

}  // namespace robot_platform_pkg
