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
 *   - detections()：  RELIABLE + KEEP_LAST(5)    — 检测结果不能丢失
 *   - camera_info()： RELIABLE + TRANSIENT_LOCAL — 迟加入节点也能获取
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
  return rclcpp::SensorDataQoS();
}

/**
 * @brief 检测/跟踪结果的 QoS 配置。
 *
 * RELIABLE（可靠）+ KEEP_LAST(depth=5)：
 * 检测结果对下游控制至关重要，不允许丢帧；5 帧缓存容忍短暂的处理抖动。
 */
inline rclcpp::QoS detections() {
  return rclcpp::QoS(5).reliable();
}

/**
 * @brief 相机内参/标定信息的 QoS 配置。
 *
 * RELIABLE + TRANSIENT_LOCAL：
 * TRANSIENT_LOCAL 替代 ROS1 的 latching 机制——
 * 迟加入的订阅者也能收到最后一次发布的内参数据。
 */
inline rclcpp::QoS camera_info() {
  return rclcpp::QoS(1).reliable().transient_local();
}

}  // namespace perception_pkg::qos
