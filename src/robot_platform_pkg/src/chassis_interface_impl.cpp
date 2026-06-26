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
    // 仿真：不实际发送，仅缓存最近一次命令，作为底盘原始速度反馈。
    last_cmd_ = cmd;
  }
  nav_msgs::msg::Odometry readOdometry() override {
    nav_msgs::msg::Odometry odom;
    // 仿真：位姿由 odometry_node 统一积分，这里只提供底盘原始速度反馈。
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist = last_cmd_;
    return odom;
  }
  void emergencyStop() override { last_cmd_ = geometry_msgs::msg::Twist(); }
  void shutdown() override {}
  bool isConnected() const override { return true; }
  float readBatteryVoltage() override { return 24.0f; }

private:
  geometry_msgs::msg::Twist last_cmd_;
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
