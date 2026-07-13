/**
 * @file chassis_interface.hpp
 * @brief 底盘抽象接口 — 隔离真实硬件与仿真实现，实现代码复用。
 *
 * IChassisInterface 定义了 LEKIWI 三全向轮底盘的统一操作契约。
 * 所有底盘操作（发送指令、读取里程计、急停）均通过此接口调用，
 * 使得上层节点无需关心底层是真实串口通信还是仿真模拟。
 *
 * 工厂函数：
 *   - make_lekiwi_chassis()      → 真实底盘（串口通信）
 *   - make_simulated_chassis()   → 仿真底盘（内存模拟）
 */

#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <string>

namespace robot_platform_pkg {

struct LekiwiChassisConfig {
  int left_wheel_id{7};
  int back_wheel_id{8};
  int right_wheel_id{9};
  double wheel_radius{0.05};
  double base_radius{0.125};
  int max_raw_velocity{3000};
};

class IChassisInterface {
public:
  virtual ~IChassisInterface() = default;

  /**
   * @brief 初始化底盘硬件（打开串口/CAN，配置电机驱动器）。
   * @param device   设备路径（如 /dev/ttyUSB0）
   * @param baudrate 通信波特率
   * @return 初始化成功返回 true
   */
  virtual bool init(const std::string& device, int baudrate) = 0;

  /// 向底盘发送速度指令（底盘坐标系下）
  virtual void sendCommand(const geometry_msgs::msg::Twist& cmd) = 0;

  /// 读取当前底盘状态（里程计、电池电压、运行状态）
  virtual nav_msgs::msg::Odometry readOdometry() = 0;

  /// 紧急停止：立即切断电机输出
  virtual void emergencyStop() = 0;

  /// 关闭底盘连接，释放资源
  virtual void shutdown() = 0;

  /// 检查硬件是否连接且健康
  virtual bool isConnected() const = 0;

  /// 读取电池电压 (V)
  virtual float readBatteryVoltage() = 0;
};

// ── 工厂函数 ──────────────────────────────────────────────────────

/// 创建真实 LEKIWI 底盘接口（串口通信）
std::unique_ptr<IChassisInterface> make_lekiwi_chassis(
  const LekiwiChassisConfig& config = LekiwiChassisConfig{});

/// 创建仿真的底盘接口（用于测试和开发）
std::unique_ptr<IChassisInterface> make_simulated_chassis();

}  // namespace robot_platform_pkg
