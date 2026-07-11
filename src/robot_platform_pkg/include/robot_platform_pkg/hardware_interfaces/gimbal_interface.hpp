/**
 * @file gimbal_interface.hpp
 * @brief 云台抽象接口 — 隔离真实 DJI RS2 硬件与仿真实现。
 *
 * IGimbalInterface 定义了云台设备的统一操作契约。
 * DJI RS2 云台通过 CAN 总线通信，支持速度控制和状态读取。
 *
 * 工厂函数：
 *   - make_dji_rs2_gimbal()      → 真实 DJI RS2 云台（CAN 通信）
 *   - make_simulated_gimbal()    → 仿真云台（内存模拟）
 */

#pragma once

#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <vision_servo_msgs/msg/platform_state.hpp>
#include <cstdint>
#include <memory>
#include <string>

namespace robot_platform_pkg {

struct GimbalStatusSnapshot {
  bool connected = false;
  float last_rx_age_sec = -1.0f;
  uint32_t tx_count = 0;
  uint32_t rx_count = 0;
  uint32_t crc_error_count = 0;
  uint32_t can_error_count = 0;
  uint32_t parse_error_count = 0;
};

class IGimbalInterface {
public:
  virtual ~IGimbalInterface() = default;

  /**
   * @brief 初始化云台（打开 CAN 总线，配置电机驱动器）。
   * @param can_interface CAN 接口名称（如 "can0"）
   *
   * 当前 DJI RS2 后端使用 DJI R SDK 协议固定 CAN ID：
   * host → gimbal 为 0x223，gimbal → host 为 0x222。
   * @return 初始化成功返回 true
   */
  virtual bool init(const std::string& can_interface) = 0;

  /// 向云台发送速度指令（yaw_rate, pitch_rate）
  virtual void sendCommand(const vision_servo_msgs::msg::GimbalCmd& cmd) = 0;

  /**
   * @brief 读取当前云台状态。
   * @param[out] yaw        当前偏航角 (rad)
   * @param[out] pitch      当前俯仰角 (rad)
   * @param[out] yaw_rate   偏航角速度 (rad/s)
   * @param[out] pitch_rate 俯仰角速度 (rad/s)
   */
  virtual void readState(float& yaw, float& pitch,
                         float& yaw_rate, float& pitch_rate) = 0;

  /// 云台归零（回到机械零点位置）
  virtual void home() = 0;

  /// 紧急停止：立即切断电机输出
  virtual void emergencyStop() = 0;

  /// 关闭云台连接，释放资源
  virtual void shutdown() = 0;

  /// 检查硬件是否连接且健康
  virtual bool isConnected() const = 0;

  /// 读取硬件连接和传输诊断快照
  virtual GimbalStatusSnapshot getStatus() const = 0;
};

// ── 工厂函数 ──────────────────────────────────────────────────────

/// 创建真实 DJI RS2 云台接口（CAN 总线通信）
std::unique_ptr<IGimbalInterface> make_dji_rs2_gimbal();

/// 创建仿真的云台接口（用于测试和开发）
std::unique_ptr<IGimbalInterface> make_simulated_gimbal();

}  // namespace robot_platform_pkg
