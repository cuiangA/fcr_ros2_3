/**
 * @file can_utils.hpp
 * @brief CAN 总线工具 — 标准 CAN 2.0 帧结构 + legacy 简单速度帧 helper。
 *
 * CanFrame：标准 CAN 帧的数据结构（ID + DLC + 8 字节数据负载）。
 *
 * 注意：当前 gimbal_driver_node 的真实 DJI RS2 后端不使用本文件。
 * 实机路径使用 protocol/dji_rs2_protocol.hpp 中的 DJI R SDK v2.2 协议封装，
 * 并通过 SocketCAN 固定 ID 0x223/0x222 收发。
 */

#pragma once

#include <cstdint>
#include <array>

namespace robot_platform_pkg {

/// 标准 CAN 2.0 帧（兼容 11-bit 和 29-bit 标识符）
struct CanFrame {
  uint32_t id;                ///< CAN 标识符（11-bit 标准帧或 29-bit 扩展帧）
  uint8_t dlc;                ///< 数据长度码（Data Length Code，0-8 字节）
  std::array<uint8_t, 8> data; ///< 8 字节数据负载

  CanFrame() : id(0), dlc(0) { data.fill(0); }
};

/**
 * @class DJIRS2Protocol
 * @brief legacy 简单速度帧编码器，当前真实 RS2 驱动未使用。
 *
 * 当前真实 RS2 驱动使用 dji_rs2::SEND_CAN_ID(0x223) 和
 * dji_rs2::RECV_CAN_ID(0x222)。
 */
class DJIRS2Protocol {
public:
  static constexpr uint32_t DEFAULT_CAN_ID = 0x223;

  /**
   * @brief 将云台角速度指令编码为 CAN 帧。
   * @param yaw_rate   偏航角速度 (rad/s)，编码为 int16（0.1 deg/s/LSB）
   * @param pitch_rate 俯仰角速度 (rad/s)，编码为 int16（0.1 deg/s/LSB）
   * @param hold_yaw   是否启用偏航保持模式（电机锁定当前位置）
   * @return 编码后的 CAN 帧
   */
  static CanFrame encodeVelocityCommand(float yaw_rate, float pitch_rate,
                                        bool hold_yaw = false);

  /// 计算 DJI RS2 协议的 XOR 校验和（对 8 字节数据逐字节异或）
  static uint8_t computeChecksum(const std::array<uint8_t, 8>& data);
};

}  // namespace robot_platform_pkg
