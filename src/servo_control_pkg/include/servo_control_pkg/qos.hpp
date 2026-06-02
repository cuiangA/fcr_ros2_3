/**
 * @file qos.hpp
 * @brief 伺服控制层的统一 QoS 配置。
 *
 * 控制指令（底盘速度、云台指令）使用 RELIABLE QoS：
 * 丢失控制帧可能导致机器人失控，因此必须可靠传输。
 *
 * 平台状态使用 TRANSIENT_LOCAL：
 * 迟加入的节点（如可视化工具）可以获取最新的平台状态。
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace servo_control_pkg::qos {

/// 控制指令（速度/云台）：可靠传输，队列深度 10
inline rclcpp::QoS control_cmd() {
  return rclcpp::QoS(10).reliable();
}

/// 伺服状态反馈：可靠传输，队列深度 5（监控用途，允许少量丢失）
inline rclcpp::QoS servo_state() {
  return rclcpp::QoS(5).reliable();
}

/// 平台状态：可靠 + TRANSIENT_LOCAL（迟加入节点可获取最新状态）
inline rclcpp::QoS platform_state() {
  return rclcpp::QoS(10).reliable().transient_local();
}

}  // namespace servo_control_pkg::qos
