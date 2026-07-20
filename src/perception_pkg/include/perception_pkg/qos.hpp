/**
 * @file qos.hpp
 * @brief 感知管线统一的 QoS（服务质量）配置。
 *
 * 集中定义 QoS 配置的目的是确保发布者和订阅者之间的兼容性。
 * ROS2 中 QoS 不匹配是最常见的无声故障原因（话题静默不连接）。
 * 通过在此处统一管理所有 QoS 配置，避免了分散定义导致的不一致。
 *
 * QoS 配置说明：
 *   - image()：       BEST_EFFORT + KEEP_LAST(1) — 传感器数据允许丢帧
 *   - perception()：  RELIABLE + KEEP_LAST(1)    — 只保留最新感知结果
 *   - camera_info()： BEST_EFFORT + VOLATILE     — 兼容常见厂商相机驱动
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

namespace perception_pkg::qos {

/**
 * @brief 原始相机图像的 QoS 配置。
 *
 * BEST_EFFORT（尽力而为）+ KEEP_LAST(depth=1) + VOLATILE：
 * 适用于高频传感器数据流，丢弃旧帧优于排队积压过时数据。
 */
inline rclcpp::QoS image() {
  return rclcpp::SensorDataQoS().keep_last(1);
}

/**
 * @brief 检测/跟踪结果的 QoS 配置。
 *
 * RELIABLE（可靠）+ KEEP_LAST(depth=1)：
 * 感知结果用于闭环控制，旧帧排队比偶发丢帧更危险，因此只保留最新帧。
 */
inline rclcpp::QoS perception() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

// 兼容现有调用点；新代码使用 perception()。
inline rclcpp::QoS detections() { return perception(); }

/**
 * @brief 相机内参/标定信息的 QoS 配置。
 *
 * 厂商驱动对 CameraInfo 的 QoS 并不统一。订阅端使用 BEST_EFFORT +
 * VOLATILE，可以同时兼容 SensorDataQoS 和 RELIABLE 发布者。
 */
inline rclcpp::QoS camera_info() {
  return rclcpp::SensorDataQoS().keep_last(1);
}

}  // namespace perception_pkg::qos
