/**
 * @file dji_rs2_protocol.hpp
 * @brief DJI R SDK protocol v2.2 encoder/decoder for RS2 gimbal.
 *
 * Vendored from ConstantRobotics DJIR_SDK.
 * Implements CRC-16 (header), CRC-32 (full frame), frame assembly,
 * and response parsing. Pure C++ — no platform dependencies.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robot_platform_pkg {
namespace dji_rs2 {

// ── CAN bus identifiers ─────────────────────────────────────────────
constexpr uint32_t SEND_CAN_ID = 0x223;   // host → gimbal command
constexpr uint32_t RECV_CAN_ID = 0x222;   // gimbal → host response

// ── Protocol constants ──────────────────────────────────────────────
constexpr uint8_t SOF          = 0xAA;
constexpr uint8_t CMD_TYPE     = 0x03;    // command frame
constexpr uint8_t RSP_TYPE     = 0x20;    // response frame
constexpr uint8_t CMD_SET      = 0x0E;    // gimbal control command set
constexpr uint8_t CMD_ID_POS   = 0x00;    // position control
constexpr uint8_t CMD_ID_SPEED = 0x01;    // speed control
constexpr uint8_t CMD_ID_QUERY = 0x02;    // query gimbal info

// Position control byte bit flags
constexpr uint8_t BIT_ABS_CTRL  = 0x01;   // 0=incremental, 1=absolute
constexpr uint8_t BIT_YAW_INV   = 0x02;   // invert yaw
constexpr uint8_t BIT_ROLL_INV  = 0x04;   // invert roll
constexpr uint8_t BIT_PITCH_INV = 0x08;   // invert pitch

// Speed control byte bit flags
constexpr uint8_t BIT_SPEED_ENABLE = 0x80; // enable speed control
constexpr uint8_t BIT_FOCAL_DISABLE = 0x08; // disable focal compensation

// ── CRC-16 (header CRC) ─────────────────────────────────────────────
// Poly=0x8005, XorIn=0xc55c, ReflectIn/Out=true, XorOut=0x0000
uint16_t crc16_compute(const uint8_t* data, size_t len);

// ── CRC-32 (full-frame CRC) ─────────────────────────────────────────
// Poly=0x04c11db7, XorIn=0xc55c0000, ReflectIn/Out=true, XorOut=0x00000000
uint32_t crc32_compute(const uint8_t* data, size_t len);

// ── Frame builders ──────────────────────────────────────────────────

/**
 * Build a complete DJI R SDK speed-control frame.
 * @param yaw   yaw speed   [0.1 deg/s, 0..3600]
 * @param roll  roll speed  [0.1 deg/s]
 * @param pitch pitch speed [0.1 deg/s]
 * @param speed_ctrl_byte   BIT_SPEED_ENABLE | BIT_FOCAL_DISABLE
 */
std::vector<uint8_t> build_speed_command(uint16_t yaw, uint16_t roll,
                                         uint16_t pitch, uint8_t speed_ctrl_byte);

/**
 * Build a complete position-control frame.
 * @param yaw, roll, pitch  angle [0.1 deg, -1800..+1800]
 * @param time_ms           motion duration [ms, min=100]
 * @param pos_ctrl_byte     BIT_ABS_CTRL | invert flags
 */
std::vector<uint8_t> build_position_command(int16_t yaw, int16_t roll,
                                            int16_t pitch, uint16_t time_ms,
                                            uint8_t pos_ctrl_byte);

/**
 * Build a query-current-position frame.
 */
std::vector<uint8_t> build_query_position_command();

// ── Response parsers ────────────────────────────────────────────────

/**
 * Parse a received protocol frame for position data.
 * Handles response frames (cmd_type=0x20) where cmd_set=0x0E, cmd_id=0x02.
 *
 * @param frame   complete received frame (already CRC-validated)
 * @param yaw     [out] yaw angle   [0.1 deg]
 * @param roll    [out] roll angle  [0.1 deg]
 * @param pitch   [out] pitch angle [0.1 deg]
 * @return true if position data was extracted
 */
bool parse_position_response(const std::vector<uint8_t>& frame,
                             int16_t& yaw, int16_t& roll, int16_t& pitch);

}  // namespace dji_rs2
}  // namespace robot_platform_pkg
