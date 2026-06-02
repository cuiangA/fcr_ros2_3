/**
 * @file imu_driver_node.cpp
 * @brief IMU 驱动节点 — 读取惯性传感器数据并以 ROS2 消息的形式发布。
 *
 * 通过 `use_sim` 参数可选择两种后端：
 *   - 真实模式：通过 I2C 连接 BNO055 9 轴绝对姿态传感器
 *   - 仿真模式：无需硬件的模拟 IMU，用于测试
 *
 * @note 发布话题：/imu/data（sensor_msgs::msg::Imu，SensorDataQoS）
 */

#include "robot_platform_pkg/hardware_interfaces/imu_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace robot_platform_pkg {

/**
 * @class IMUDriverNode
 * @brief 以固定频率读取 IMU 并发布 sensor_msgs/Imu 消息的 ROS2 节点。
 *
 * 参数（均可在运行时声明）：
 *   - use_sim  (bool,   默认 false):       true = 仿真，false = 真实 BNO055
 *   - device   (string, 默认 /dev/i2c-1):  I2C 设备路径（仅真实模式）
 *   - baudrate (int,    默认 0):           I2C 波特率（0 = 使用芯片默认值）
 *   - rate_hz  (double, 默认 100.0):       发布频率，单位 Hz
 *   - frame_id (string, 默认 "imu_link"):  消息头中的 TF 坐标系名称
 */
class IMUDriverNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数 — 声明参数、初始化 IMU 后端、创建发布者和定时器。
   */
  explicit IMUDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("imu_driver", options)
  {
    // ── 1. 声明参数并设置合理默认值 ──────────────────────────────
    this->declare_parameter("use_sim", false);
    this->declare_parameter("device", "/dev/i2c-1");
    this->declare_parameter("baudrate", 0);
    this->declare_parameter("rate_hz", 100.0);
    this->declare_parameter("frame_id", "imu_link");

    bool use_sim = this->get_parameter("use_sim").as_bool();
    double rate = this->get_parameter("rate_hz").as_double();

    // ── 2. 实例化 IMU 后端（真实或仿真） ─────────────────────────
    if (use_sim) {
      imu_ = make_simulated_imu();               // 无需真实硬件
    } else {
      imu_ = make_bno055_imu();                  // BNO055 通过 I2C 连接
      std::string device = this->get_parameter("device").as_string();
      imu_->init(device, 0);                     // 0 = 使用默认波特率
    }

    // ── 3. 使用 SensorDataQoS 创建发布者 ─────────────────────────
    // SensorDataQoS = BEST_EFFORT + KEEP_LAST(depth=1) + VOLATILE。
    // 适用于高频传感器数据流：丢弃旧数据优于排队积压过时数据。
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS());

    // ── 4. 创建挂钟定时器，触发周期性读取 ───────────────────────
    read_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&IMUDriverNode::publish_imu, this));

    RCLCPP_INFO(get_logger(), "IMU 驱动已启动 (sim=%d, rate=%.1fHz)", use_sim, rate);
  }

private:
  /**
   * @brief 定时器回调 — 从 IMU 读取一次采样数据并发布。
   *
   * 按 `rate_hz` 指定的频率调用。在发布前填充时间戳和 frame_id。
   */
  void publish_imu() {
    auto msg = imu_->read();                                            // 阻塞读取
    msg.header.stamp = this->now();                                    // ROS 时间戳
    msg.header.frame_id = this->get_parameter("frame_id").as_string(); // TF 坐标系
    imu_pub_->publish(msg);
  }

  // ── 成员变量 ───────────────────────────────────────────────────

  std::unique_ptr<IIMUInterface> imu_;                                  ///< IMU 后端（仿真或真实）
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;        ///< 发布者 → /imu/data
  rclcpp::TimerBase::SharedPtr read_timer_;                            ///< 周期性读取定时器
};

}  // namespace robot_platform_pkg

/**
 * @brief 入口函数 — 初始化 ROS2、驱动节点、完成后关闭。
 *
 * 使用单线程默认执行器（rclcpp::spin）。如需多线程执行，
 * 可替换为 rclcpp::executors::MultiThreadedExecutor。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_platform_pkg::IMUDriverNode>());
  rclcpp::shutdown();
  return 0;
}
