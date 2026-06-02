/**
 * @file imu_interface.hpp
 * @brief IMU 抽象接口 — 隔离真实 BNO055 硬件与仿真实现。
 *
 * IIMUInterface 定义了惯性测量单元的统一操作契约。
 * BNO055 是 9 轴绝对姿态传感器（加速度计 + 陀螺仪 + 磁力计），
 * 通过 I2C 总线通信，内置传感器融合可直接输出欧拉角/四元数。
 *
 * 工厂函数：
 *   - make_bno055_imu()      → 真实 BNO055 IMU（I2C 通信）
 *   - make_simulated_imu()   → 仿真 IMU（内存模拟）
 */

#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include <string>

namespace robot_platform_pkg {

class IIMUInterface {
public:
  virtual ~IIMUInterface() = default;

  /**
   * @brief 初始化 IMU 硬件（打开 I2C 总线，配置传感器）。
   * @param device   I2C 设备路径（如 /dev/i2c-1）
   * @param baudrate 通信速率（0 = 使用芯片默认值）
   * @return 初始化成功返回 true
   */
  virtual bool init(const std::string& device, int baudrate) = 0;

  /// 读取最新的 IMU 数据（包含方向四元数、角速度、线加速度）
  virtual sensor_msgs::msg::Imu read() = 0;

  /// 检查传感器是否连接且正常
  virtual bool isConnected() const = 0;

  /// 关闭 IMU 连接，释放资源
  virtual void shutdown() = 0;
};

// ── 工厂函数 ──────────────────────────────────────────────────────

/// 创建真实 BNO055 IMU 接口（I2C 通信）
std::unique_ptr<IIMUInterface> make_bno055_imu();

/// 创建仿真的 IMU 接口（用于测试和开发）
std::unique_ptr<IIMUInterface> make_simulated_imu();

}  // namespace robot_platform_pkg
