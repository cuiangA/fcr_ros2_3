/**
 * @file serial_utils.hpp
 * @brief 跨平台串口通信封装 — 提供 RAII 风格的串口操作接口。
 *
 * SerialPort 类封装了操作系统底层的串口操作（open/close/read/write/flush），
 * 通过 void* handle_ 存储平台特定的句柄（Linux termios / Windows HANDLE），
 * 实现跨平台兼容。
 *
 * 使用 RAII（Resource Acquisition Is Initialization）模式：
 *   构造函数获取资源，析构函数自动释放，避免资源泄漏。
 */

#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <memory>

namespace robot_platform_pkg {

class SerialPort {
public:
  SerialPort() = default;

  /// 析构时自动关闭串口，释放资源
  ~SerialPort();

  /**
   * @brief 打开串口设备。
   * @param device   设备路径（Linux: /dev/ttyUSB0, Windows: COM3）
   * @param baudrate 波特率（115200, 921600 等）
   * @return 打开成功返回 true
   */
  bool open(const std::string& device, int baudrate);

  /// 关闭串口
  void close();

  /// 检查串口是否已打开
  bool isOpen() const;

  /**
   * @brief 从串口读取数据。
   * @param buffer 接收缓冲区
   * @param size   期望读取的字节数
   * @return 实际读取的字节数（<= 0 表示错误或超时）
   */
  int read(uint8_t* buffer, size_t size);

  /**
   * @brief 向串口写入数据。
   * @param buffer 发送缓冲区
   * @param size   要写入的字节数
   * @return 实际写入的字节数
   */
  int write(const uint8_t* buffer, size_t size);

  /// 清空串口的输入/输出缓冲区
  void flush();

private:
  void* handle_ = nullptr;  ///< 平台特定句柄（Linux: fd, Windows: HANDLE）
};

}  // namespace robot_platform_pkg
