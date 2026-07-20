/**
 * @file mvp_follow_controller_node.cpp
 * @brief Stage-1 MVP 跟随控制器 — 图像空间 + 深度 + 云台偏航的三通道解耦控制。
 *
 * ── 在系统中的位置 ─────────────────────────────────────────────────
 *
 *   mvp_follow_controller_node 是 servo_manager_node 的轻量替代，
 *   直接订阅感知目标并输出底盘/云台指令，跳过完整的 6-DOF 视觉伺服链路：
 *
 *   /target/current  ──→ [mvp_follow_controller_node] ──→ /cmd_vel (底盘)
 *   /platform/state  ──→      │                         ──→ /cmd_gimbal (云台)
 *                             │
 *                    三通道解耦 P 控制 + IBVS
 *
 *   与 servo_manager_node 的关键区别：
 *     - 不通过 pluginlib 加载控制器，逻辑自包含在单文件内
 *     - 不计算完整 6-DOF 相机速度，而是直接在图像/深度空间做 P 控制
 *     - 不经过 ControlAllocator 分配，底盘和云台的职责硬编码分工
 *     - 适合快速原型验证、轻量仿真、以及脱离完整视觉伺服链路的独立测试
 *
 * ── 控制架构：两级串联 ──────────────────────────────────────────
 *
 *   图像水平误差 ex ──→ [云台偏航] ──→ 云台偏航角 q_yaw ──→ [底盘偏航]
 *   图像垂直误差 ey ──→ [云台俯仰]                                  │
 *   深度误差     ez ──→ [底盘前向] ←────────────────────────────────┘
 *
 *   这是一个典型的串级/级联控制结构：
 *
 *   ┌──────────────────────────────────────────────────────────────────┐
 *   │  内环（云台，高带宽）                                              │
 *   │                                                                    │
 *   │    ex = (pixel_x - cx) / cx    归一化水平误差 [-1, 1]              │
 *   │    ey = (pixel_y - cy) / cy    归一化垂直误差 [-1, 1]              │
 *   │                                                                    │
 *   │    ┌─ P 控制:  gimbal_yaw   = -K_gimbal_x × ex                    │
 *   │    │           gimbal_pitch = -K_gimbal_y × ey                    │
 *   │    │                                                               │
 *   │    └─ IBVS:   使用单点特征角速度交互矩阵 Lw(2×3)                    │
 *   │               ω = -λ · (LwᵀLw + δ²I)⁻¹ · Lwᵀ · e                 │
 *   │               gimbal_yaw = -ωy, gimbal_pitch = -ωx               │
 *   │                                                                    │
 *   │    云台质量小、响应快（>10Hz），确保目标始终在视野中央              │
 *   └──────────────────────────┬───────────────────────────────────────┘
 *                              │ 云台转动产生偏航角 q_yaw
 *                              ▼
 *   ┌──────────────────────────────────────────────────────────────────┐
 *   │  外环（底盘，低带宽）                                              │
 *   │                                                                    │
 *   │    base_vx = K_base_z × ez       深度误差 → 前向速度               │
 *   │              ez = depth - desired_distance                         │
 *   │                                                                    │
 *   │    base_wz = sign × K_base_yaw × wrap(q_yaw-q_center)             │
 *   │              if |q_yaw| > deadband else 0                          │
 *   │                                                                    │
 *   │    底盘惯性大、响应慢（~2Hz），缓慢消除云台偏角，                     │
 *   │    使云台逐步回归机械中心，保持运动学裕量                            │
 *   └──────────────────────────────────────────────────────────────────┘
 *
 *   这种设计的好处：
 *     1. 响应快：云台先动，图像误差几乎零延迟被补偿
 *     2. 不丢目标：底盘转弯时云台已提前对准，目标始终在视野内
 *     3. 自然解耦：图像误差和深度/偏航控制天然分层，无需显式的
 *        6-DOF 分配矩阵（对比 servo_manager 的 ControlAllocator）
 *     4. 代码简单：纯 P 控制 + 死区 + 低通滤波，无需求解伪逆（P 模式）
 *
 * ── 信号处理链（每条控制通道）────────────────────────────────────
 *
 *   原始测量 → 低通滤波 → 死区 → P 增益 → 限幅 → 低通滤波 → 输出
 *   (raw)     (α=0.5)   (dead)  (K_p)    (clamp)  (α=0.3)   (cmd)
 *
 *   两级低通滤波的原因：
 *     - 第一级（filter_alpha_error/depth）：抑制传感器噪声和检测抖动
 *     - 第二级（filter_alpha_cmd）：防止相邻帧指令跳变，保护执行器
 *   deadband 的作用：微小误差不输出指令，避免执行器高频微振
 *
 * ── 控制回路（control_loop, 50 Hz = 20ms 周期）──────────────────
 *
 *   每帧执行：
 *     1. 检查目标是否超时（target_timeout_ = 0.5s）
 *     2. 提取像素中心 + 深度，归一化误差
 *     3. 对误差做低通滤波和死区
 *     4. 计算云台指令（P 或 IBVS）
 *     5. 计算底盘指令（深度 + 云台偏航角）
 *     6. 对输出指令做低通滤波和限幅
 *     7. 发布 /cmd_vel（TwistStamped, frame_id=base_link）
 *     8. 发布 /cmd_gimbal（GimbalCmd: yaw_rate, pitch_rate）
 *
 * ── 线程安全 ──────────────────────────────────────────────────────
 *
 *   单节点内只有两个线程竞争共享状态：
 *     - target_callback（订阅者线程，~10-30 Hz）
 *     - control_loop（定时器线程，50 Hz）
 *     - platform_state_callback（订阅者线程，~50 Hz）
 *
 *   当前采用无锁设计：每个回调只写入自己拥有的变量，control_loop 只读取。
 *   active_target_ / has_valid_target_ 的写入和读取之间没有原子保护，
 *   这在 50Hz 控制频率下是可接受的（最多一帧延迟，不会造成崩溃）。
 *   如需严格线程安全，可将 active_target_ 改为 atomic<optional<Target>>。
 *
 * ── 话题 QoS 策略 ─────────────────────────────────────────────────
 *
 *   - /target/current：control_cmd()（Best-effort, 低延迟）
 *   - /platform/state：platform_state()（Best-effort, 低延迟）
 *   - /cmd_vel：control_cmd()（Best-effort, 控制指令不重传）
 *   - /cmd_gimbal：control_cmd()（Best-effort, 控制指令不重传）
 */

#include "servo_control_pkg/qos.hpp"
#include "servo_control_pkg/mvp_safety.hpp"
#include "servo_control_pkg/mvp_yaw_control.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <vision_servo_msgs/msg/platform_state.hpp>
#include <vision_servo_msgs/msg/target.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace servo_control_pkg {

namespace {

using vision_servo_msgs::msg::Target;

/**
 * @brief 检查值是否为正有限数（用于验证深度等物理量）。
 *
 * NaN 或负深度在物理上无意义，inf 通常来自除零或传感器故障。
 * 将它们过滤掉可以避免控制律产生垃圾指令。
 */
bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

/// 将 alpha 钳位到 [0, 1]，保证低通滤波器的稳定性。
double clamp_unit_alpha(double alpha)
{
  return std::clamp(alpha, 0.0, 1.0);
}

/**
 * @brief 一阶低通滤波器（指数平滑）。
 *
 * y[k] = α·x[k] + (1-α)·y[k-1]
 *
 * α = 1: 不过滤（输出 = 输入）
 * α = 0: 完全过滤（输出冻结）
 * α = 0.3: 典型值，约 3 帧达到稳态的 63%
 *
 * 用于以下场景：
 *   - filter_alpha_error (0.5): 图像误差平滑，抑制检测器噪声
 *   - filter_alpha_depth (0.3): 深度估计平滑，深度传感器噪声较大
 *   - filter_alpha_cmd  (0.3): 输出指令平滑，保护执行机构
 */
double low_pass(double current, double last, double alpha)
{
  alpha = clamp_unit_alpha(alpha);
  return alpha * current + (1.0 - alpha) * last;
}

/**
 * @brief 死区滤波器 — 将幅值小于 deadband 的信号置零。
 *
 * 目的：避免微小误差引起执行器高频微振（"抖动"）。
 * 例如 ex_deadband=0.05 意味着只有目标偏离图像中心超过 5% 时才输出指令。
 */
double apply_deadband(double value, double deadband)
{
  return std::abs(value) < deadband ? 0.0 : value;
}

/**
 * @brief 判断 bbox 坐标是否为归一化值 [0, 1]。
 *
 * 感知管线可能输出像素坐标或归一化坐标，此函数用于区分两者。
 * 若四个 bbox 值均在 [0, 1] 内则判定为归一化坐标。
 */
bool bbox_looks_normalized(const Target& target)
{
  return std::all_of(target.bbox.begin(), target.bbox.end(), [](float value) {
    return std::isfinite(value) && value >= 0.0f && value <= 1.0f;
  });
}

/// 判断 bbox 是否包含正面积，避免默认 [0,0,0,0] 被误当作有效观测。
bool bbox_has_positive_area(const Target& target)
{
  return std::all_of(
    target.bbox.begin(), target.bbox.end(),
    [](float value) { return std::isfinite(value); }) &&
    target.bbox[2] > target.bbox[0] &&
    target.bbox[3] > target.bbox[1];
}

}  // namespace

class MvpFollowControllerNode final : public rclcpp::Node
{
public:
  explicit MvpFollowControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("mvp_follow_controller_node", options)
  {
    load_parameters();

    // ═══════════════════════════════════════════════════════════════════════
    // 1. 订阅者 — 接收上游感知和目标数据
    // ═══════════════════════════════════════════════════════════════════════

    // 当前跟踪目标（单目标，由 tracker 或外部节点选出）。
    // 消息包含目标的 3D 位姿（相机系）、bbox、置信度等。
    target_sub_ = create_subscription<vision_servo_msgs::msg::TargetArray>(
      target_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MvpFollowControllerNode::target_callback, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MvpFollowControllerNode::camera_info_callback, this, std::placeholders::_1));

    // 平台状态：底盘当前线速度/角速度 + 云台 yaw/pitch 角度。
    // control_loop 需要云台偏航角 q_yaw 作为底盘偏航控制的外环反馈量。
    platform_sub_ = create_subscription<vision_servo_msgs::msg::PlatformState>(
      platform_state_topic_, qos::platform_state(),
      std::bind(&MvpFollowControllerNode::platform_state_callback, this, std::placeholders::_1));

    // ═══════════════════════════════════════════════════════════════════════
    // 2. 发布者 — 向下游执行器发送指令
    // ═══════════════════════════════════════════════════════════════════════

    // 底盘速度指令：支持 TwistStamped（推荐，带时间戳和 frame_id）和
    // Twist（兼容 Gazebo 插件等老旧接口），通过 use_twist_stamped 参数切换。
    if (use_twist_stamped_) {
      cmd_vel_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_, qos::control_cmd());
    } else {
      cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, qos::control_cmd());
    }

    // 云台速率指令：yaw_rate + pitch_rate，由云台驱动节点（gimbal_driver）消费。
    cmd_gimbal_pub_ = create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      cmd_gimbal_topic_, qos::control_cmd());

    // ═══════════════════════════════════════════════════════════════════════
    // 3. 控制回路定时器 — 50 Hz 主循环
    // ═══════════════════════════════════════════════════════════════════════
    //
    // 使用 wall timer 保证稳定的 20ms 周期，不受仿真时钟暂停/加速影响。
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0)));
    control_timer_ = create_wall_timer(
      period, std::bind(&MvpFollowControllerNode::control_loop, this));

    last_valid_target_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "MVP follow controller started: target=%s, camera_info=%s, platform=%s, "
      "cmd_vel=%s (%s), cmd_gimbal=%s, gates[gimbal=%d,yaw=%d,translation=%d], "
      "signs[yaw=%.1f,pitch=%.1f,base_yaw=%.1f], yaw_center=%s",
      target_topic_.c_str(), camera_info_topic_.c_str(), platform_state_topic_.c_str(),
      cmd_vel_topic_.c_str(), use_twist_stamped_ ? "TwistStamped" : "Twist",
      cmd_gimbal_topic_.c_str(), enable_gimbal_tracking_, enable_base_yaw_,
      enable_base_translation_, yaw_sign_, pitch_sign_, base_yaw_sign_,
      capture_gimbal_yaw_center_on_startup_ ? "startup" : "fixed");
  }

  /**
   * @brief 析构 — 安全停车。
   *
   * 节点退出前发布零速度指令：
   *   - 底盘：线速度/角速度均为 0（停止移动）
   *   - 云台：yaw_rate/pitch_rate 均为 0（停止转动，但 hold_*=false 不锁定）
   */
  ~MvpFollowControllerNode() override
  {
    publish_zero_command();
  }

private:
  // ═══════════════════════════════════════════════════════════════════════════
  // 数据结构
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * @struct ControlCommand
   * @brief 一次控制周期的输出指令集合。
   *
   * 四个自由度：
   *   base_vx      — 底盘前向线速度 (m/s)，正 = 前进
   *   base_wz      — 底盘偏航角速度 (rad/s)，正 = 左转（ROS 约定）
   *   gimbal_yaw_vel   — 云台偏航角速率 (rad/s)
   *   gimbal_pitch_vel — 云台俯仰角速率 (rad/s)
   */
  struct ControlCommand
  {
    double base_vx = 0.0;
    double base_wz = 0.0;
    double gimbal_yaw_vel = 0.0;
    double gimbal_pitch_vel = 0.0;
  };

  // ═══════════════════════════════════════════════════════════════════════════
  // 参数加载
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * @brief 声明带描述的 ROS 参数。
   *
   * 模板辅助函数，避免每行重复写 ParameterDescriptor。
   * 描述信息在 ros2 param describe 和命令行自动补全中可见。
   */
  template <typename T>
  T declare_described_parameter(
    const std::string& name, const T& default_value, const std::string& description)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description = description;
    return declare_parameter<T>(name, default_value, descriptor);
  }

  /**
   * @brief 加载并缓存所有 ROS 参数。
   *
   * 参数分为以下几类：
   *   A) 话题映射：输入/输出话题名，支持 remap 和自定义命名空间
   *   B) 相机参数：内参 fx/fy/cx/cy 和图像分辨率
   *   C) 控制增益：四通道的 P 增益（gimbal_x, gimbal_y, base_z, base_yaw）
   *   D) 死区参数：四通道的 deadband 阈值
   *   E) 限幅参数：各通道的最大输出值
   *   F) 滤波参数：两级低通滤波的平滑系数 α
   *   G) 调度参数：控制频率、目标超时
   *   H) 调试参数：mock 云台状态（无需 PlatformState 话题即可独立运行）
   */
  void load_parameters()
  {
    // ── A) 话题映射 ──────────────────────────────────────────────────
    target_topic_ = declare_described_parameter<std::string>(
      "target_topic", "/target/current",
      "TargetArray input topic for the MVP controller.");
    platform_state_topic_ = declare_described_parameter<std::string>(
      "platform_state_topic", "/platform/state",
      "PlatformState topic carrying gimbal angles.");
    camera_info_topic_ = declare_described_parameter<std::string>(
      "camera_info_topic", "/sony/camera_info",
      "CameraInfo topic used to configure image dimensions and intrinsics.");
    cmd_vel_topic_ = declare_described_parameter<std::string>(
      "cmd_vel_topic", "/cmd_vel",
      "Base velocity command topic.");
    cmd_gimbal_topic_ = declare_described_parameter<std::string>(
      "cmd_gimbal_topic", "/cmd_gimbal",
      "Gimbal velocity command topic.");

    // ── B) 消息格式 + 仿真开关 ──────────────────────────────────────
    use_twist_stamped_ = declare_described_parameter<bool>(
      "use_twist_stamped", true,
      "Publish geometry_msgs/TwistStamped on cmd_vel when true.");
    use_mock_gimbal_state_ = declare_described_parameter<bool>(
      "use_mock_gimbal_state", true,
      "Use mock_q_yaw/mock_q_pitch instead of PlatformState.");
    enable_gimbal_tracking_ = declare_described_parameter<bool>(
      "enable_gimbal_tracking", true,
      "Allow the MVP controller to command gimbal yaw and pitch.");
    enable_base_yaw_ = declare_described_parameter<bool>(
      "enable_base_yaw", false,
      "Allow chassis angular.z commands from measured gimbal yaw.");
    enable_base_translation_ = declare_described_parameter<bool>(
      "enable_base_translation", false,
      "Allow chassis linear.x only when metric depth passes every safety gate.");

    // ── C) 相机参数 ──────────────────────────────────────────────────
    // fx/fy/cx/cy ≤ 0 时自动推断为 image_width/2, image_height/2（简易估计）。
    image_width_ = declare_described_parameter<int>(
      "image_width", 640, "Input image width in pixels.");
    image_height_ = declare_described_parameter<int>(
      "image_height", 480, "Input image height in pixels.");
    control_rate_hz_ = declare_described_parameter<double>(
      "control_rate_hz", 50.0, "Control loop frequency in Hz.");
    require_camera_info_ = declare_described_parameter<bool>(
      "require_camera_info", false,
      "Publish only zero commands until a valid CameraInfo has been received.");
    center_reference_ = declare_described_parameter<std::string>(
      "center_reference", "image_center",
      "Tracking goal: image_center or principal_point.");

    // ── D) 期望深度 ─────────────────────────────────────────────────
    // 底盘前向速度 = K_base_z × (当前深度 - desired_distance)
    desired_distance_ = declare_described_parameter<double>(
      "desired_distance", 2.0, "Desired target distance in meters.");

    // ── E) P 控制增益 ───────────────────────────────────────────────
    //
    // 增益的选择遵循"云台快、底盘慢"的原则：
    //   K_gimbal_x = 0.8: 目标偏离图像中心 50% → 云台 0.4 rad/s 偏航
    //   K_gimbal_y = 0.6: 目标偏离图像中心 50% → 云台 0.3 rad/s 俯仰
    //   K_base_z   = 0.4: 目标距离期望 1m → 底盘 0.4 m/s 前进
    //   K_base_yaw = 0.5: 云台偏航 0.5rad(≈30°) → 底盘 0.25 rad/s 偏航
    k_gimbal_x_ = declare_described_parameter<double>(
      "K_gimbal_x", 0.8,
      "Gimbal yaw proportional gain on normalized image x error.");
    k_gimbal_y_ = declare_described_parameter<double>(
      "K_gimbal_y", 0.6,
      "Gimbal pitch proportional gain on normalized image y error.");
    k_base_z_ = declare_described_parameter<double>(
      "K_base_z", 0.4,
      "Base forward proportional gain on depth error.");
    k_base_yaw_ = declare_described_parameter<double>(
      "K_base_yaw", 0.5,
      "Base yaw proportional gain on gimbal yaw angle.");
    base_yaw_sign_ = declare_described_parameter<double>(
      "base_yaw_sign", -1.0,
      "Direction multiplier from relative gimbal yaw to ROS base angular.z.");
    yaw_sign_ = declare_described_parameter<double>(
      "yaw_sign", -1.0,
      "Hardware direction multiplier applied to the gimbal yaw command.");
    pitch_sign_ = declare_described_parameter<double>(
      "pitch_sign", -1.0,
      "Hardware direction multiplier applied to the gimbal pitch command.");

    // ── F) IBVS 参数（云台的可选控制方式）───────────────────────────
    //
    // use_ibvs_gimbal=true 时，云台不直接用 P 控制 ex/ey，而是：
    //   1. 将像素坐标转为归一化坐标 (x, y) = ((u-cx)/fx, (v-cy)/fy)
    //   2. 对期望位置 (0,0)（图像主点）计算误差 e = [x, y]ᵀ
    //   3. 用单点特征角速度交互矩阵 Lw(2×3) 求解相机角速度 ω
    //   4. 从 ω 中提取云台 yaw/pitch 分量
    //
    // P 控制和 IBVS 的主要区别：
    //   - P 控制：线性映射 ex→yaw_rate，简单但忽略了特征的透视投影非线性
    //   - IBVS： 考虑特征在图像平面上的投影几何，在大偏离时更准确
    use_ibvs_gimbal_ = declare_described_parameter<bool>(
      "use_ibvs_gimbal", true,
      "Use point-feature IBVS for gimbal yaw/pitch commands.");
    ibvs_gimbal_gain_ = declare_described_parameter<double>(
      "ibvs_gimbal_gain", 0.8,
      "IBVS gain used by the MVP gimbal-only controller.");
    ibvs_damping_ = declare_described_parameter<double>(
      "ibvs_damping", 0.01,
      "Damping coefficient for the gimbal IBVS pseudo-inverse.");

    // ── G) 相机内参（≤0 时用 image_width/2 回退）───────────────────
    camera_fx_ = declare_described_parameter<double>(
      "camera_fx", 0.0,
      "Camera fx in pixels; <=0 falls back to image_width / 2.");
    camera_fy_ = declare_described_parameter<double>(
      "camera_fy", 0.0,
      "Camera fy in pixels; <=0 falls back to image_height / 2.");
    camera_cx_ = declare_described_parameter<double>(
      "camera_cx", 0.0,
      "Camera principal point x; <=0 falls back to image_width / 2.");
    camera_cy_ = declare_described_parameter<double>(
      "camera_cy", 0.0,
      "Camera principal point y; <=0 falls back to image_height / 2.");

    // ── H) 死区参数 ─────────────────────────────────────────────────
    //
    // 死区防止微小误差引起的高频振荡。单位为归一化误差 / 米 / 弧度。
    //   ex_deadband = 0.05: 目标在图像中心 ±5% 内不做偏航补偿
    //   ey_deadband = 0.05: 目标在图像中心 ±5% 内不做俯仰补偿
    //   depth_deadband = 0.2: 目标距离期望 ±20cm 内不调整底盘速度
    //   q_yaw_deadband = 0.0873: 云台偏航 ±5°(≈0.087rad) 内底盘不转动
    ex_deadband_ = declare_described_parameter<double>(
      "ex_deadband", 0.05, "Deadband for normalized image x error.");
    ey_deadband_ = declare_described_parameter<double>(
      "ey_deadband", 0.05, "Deadband for normalized image y error.");
    depth_deadband_ = declare_described_parameter<double>(
      "depth_deadband", 0.2, "Deadband for depth error in meters.");
    q_yaw_deadband_ = declare_described_parameter<double>(
      "q_yaw_deadband", 0.0873, "Deadband for gimbal yaw angle in radians.");
    min_depth_confidence_ = declare_described_parameter<double>(
      "min_depth_confidence", 0.6,
      "Minimum Target.depth_confidence required for chassis translation.");
    min_valid_depth_ = declare_described_parameter<double>(
      "min_valid_depth", 0.3, "Minimum accepted metric target depth in meters.");
    max_valid_depth_ = declare_described_parameter<double>(
      "max_valid_depth", 10.0, "Maximum accepted metric target depth in meters.");

    // ── I) 执行器限幅 ───────────────────────────────────────────────
    //
    // 防止控制律输出超出执行器物理能力的指令。
    // 云台限幅 (0.8/0.6 rad/s) 比底盘限幅 (0.4 m/s, 0.6 rad/s) 宽松，
    // 反映了云台的高动态能力。
    max_gimbal_yaw_vel_ = declare_described_parameter<double>(
      "max_gimbal_yaw_vel", 0.8, "Maximum absolute gimbal yaw velocity in rad/s.");
    max_gimbal_pitch_vel_ = declare_described_parameter<double>(
      "max_gimbal_pitch_vel", 0.6, "Maximum absolute gimbal pitch velocity in rad/s.");
    max_base_vx_ = declare_described_parameter<double>(
      "max_base_vx", 0.4, "Maximum absolute base forward velocity in m/s.");
    max_base_wz_ = declare_described_parameter<double>(
      "max_base_wz", 0.6, "Maximum absolute base yaw velocity in rad/s.");

    // ── J) 低通滤波系数 ─────────────────────────────────────────────
    //
    // α 越大响应越快但噪声抑制越弱，α 越小越平滑但延迟越大。
    //   filter_alpha_error = 0.5: 图像误差滤波（目标检测噪声中等）
    //   filter_alpha_depth = 0.3: 深度滤波（单目深度估计噪声较大，需更多平滑）
    //   filter_alpha_cmd  = 0.3: 输出指令滤波（保护执行器）
    filter_alpha_error_ = declare_described_parameter<double>(
      "filter_alpha_error", 0.5, "Low-pass alpha for image errors.");
    filter_alpha_depth_ = declare_described_parameter<double>(
      "filter_alpha_depth", 0.3, "Low-pass alpha for target depth.");
    filter_alpha_cmd_ = declare_described_parameter<double>(
      "filter_alpha_cmd", 0.3, "Low-pass alpha for output commands.");

    // ── K) 目标超时 ─────────────────────────────────────────────────
    //
    // 超过此时间未收到有效目标，控制回路发布零速度并停止。
    // 0.5s 约等于 25 个控制帧（50Hz），足够容忍检测器偶尔漏检一两帧。
    target_timeout_ = declare_described_parameter<double>(
      "target_timeout", 0.5,
      "Seconds before the target is treated as lost.");

    // ── L) Mock 云台角度（当 use_mock_gimbal_state=true 时生效）─────────
    //
    // 用于独立测试（不需要真正的 PlatformState 话题）。
    // mock_q_yaw=0 意味着"假装云台始终居中"，底盘偏航通道将不输出。
    mock_q_yaw_ = declare_described_parameter<double>(
      "mock_q_yaw", 0.0, "Mock gimbal yaw angle in radians.");
    mock_q_pitch_ = declare_described_parameter<double>(
      "mock_q_pitch", 0.0, "Mock gimbal pitch angle in radians.");
    capture_gimbal_yaw_center_on_startup_ = declare_described_parameter<bool>(
      "capture_gimbal_yaw_center_on_startup", false,
      "Treat the first valid real gimbal yaw sample as the base-forward reference.");
    configured_gimbal_yaw_center_ = declare_described_parameter<double>(
      "gimbal_yaw_center", 0.0,
      "Fixed real gimbal yaw reference when startup capture is disabled.");

    if (center_reference_ != "image_center" && center_reference_ != "principal_point") {
      throw std::invalid_argument(
        "center_reference must be 'image_center' or 'principal_point'");
    }
    if (!std::isfinite(yaw_sign_) || std::abs(yaw_sign_) < 1e-9 ||
        !std::isfinite(pitch_sign_) || std::abs(pitch_sign_) < 1e-9) {
      throw std::invalid_argument("yaw_sign and pitch_sign must be finite and non-zero");
    }
    if (!std::isfinite(base_yaw_sign_) || std::abs(base_yaw_sign_) < 1e-9 ||
        !std::isfinite(configured_gimbal_yaw_center_)) {
      throw std::invalid_argument(
        "base_yaw_sign must be finite/non-zero and gimbal_yaw_center must be finite");
    }
    if (min_depth_confidence_ < 0.0 || min_depth_confidence_ > 1.0 ||
        !finite_positive(min_valid_depth_) ||
        !std::isfinite(max_valid_depth_) || max_valid_depth_ <= min_valid_depth_) {
      throw std::invalid_argument("invalid metric depth safety gate configuration");
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // 订阅回调
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * @brief 目标回调 — 缓存最新目标并验证其有效性。
   *
   * 目标选择策略：
   *   1. 若 target_array.tracking_id ≥ 0，优先选择 id 匹配的目标（跨帧跟踪一致性）
   *   2. 若未找到匹配，回退到置信度最高的目标
   *   3. 若 targets 数组为空，has_valid_target_ 保持 false
   *
   * 仅缓存目标数据，不在此回调中执行控制。实际的伺服计算在 control_loop()
   * 中以固定频率进行，实现感知与控制的解耦。
   */
  void target_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto selected = select_target(*msg);
    if (!selected || !is_valid_target(*selected)) {
      has_valid_target_ = false;
      return;
    }

    active_target_ = *selected;
    has_valid_target_ = true;
    last_valid_target_time_ = now();
  }

  /**
   * @brief 平台状态回调 — 缓存云台当前角度。
   *
   * 仅缓存 gimbal_yaw 和 gimbal_pitch，这两个值是底盘偏航控制的反馈量。
   * 底盘速度（chassis_velocity）在此控制器中不使用（与 servo_manager 不同）。
   *
   * @note 当 use_mock_gimbal_state_=true 时，此回调的结果被 current_gimbal_yaw() 忽略。
   */
  void platform_state_callback(const vision_servo_msgs::msg::PlatformState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!msg->gimbal_connected) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for a connected gimbal before accepting PlatformState yaw");
      return;
    }
    if (!std::isfinite(msg->gimbal_yaw) || !std::isfinite(msg->gimbal_pitch)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring non-finite gimbal state in PlatformState");
      return;
    }
    if (!platform_state_received_ && capture_gimbal_yaw_center_on_startup_) {
      captured_gimbal_yaw_center_ = msg->gimbal_yaw;
      gimbal_yaw_center_captured_ = true;
      RCLCPP_INFO(
        get_logger(), "Captured gimbal forward reference: yaw=%.6f rad",
        captured_gimbal_yaw_center_);
    }
    platform_state_received_ = true;
    platform_q_yaw_ = msg->gimbal_yaw;
    platform_q_pitch_ = msg->gimbal_pitch;
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    if (msg->width == 0 || msg->height == 0 ||
        !finite_positive(msg->k[0]) || !finite_positive(msg->k[4]) ||
        !std::isfinite(msg->k[2]) || !std::isfinite(msg->k[5])) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid CameraInfo for MVP tracking");
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    image_width_ = static_cast<int>(msg->width);
    image_height_ = static_cast<int>(msg->height);
    camera_fx_ = msg->k[0];
    camera_fy_ = msg->k[4];
    camera_cx_ = msg->k[2];
    camera_cy_ = msg->k[5];
    camera_info_received_ = true;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // 目标选择与验证
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * @brief 从 TargetArray 中选择伺服目标。
   *
   * 优先级：
   *   1. 若 tracking_id ≥ 0: 查找 id == tracking_id 的目标（跨帧跟踪）
   *   2. 否则: 选择 confidence 最高的目标
   *
   * @return 选中的目标，若数组为空则返回 nullopt。
   */
  std::optional<Target> select_target(const vision_servo_msgs::msg::TargetArray& msg) const
  {
    if (msg.targets.empty()) {
      return std::nullopt;
    }

    if (msg.tracking_id >= 0) {
      const auto tracked = std::find_if(
        msg.targets.begin(), msg.targets.end(),
        [&msg](const Target& target) { return target.id == msg.tracking_id; });
      if (tracked != msg.targets.end()) {
        return *tracked;
      }
    }

    return *std::max_element(
      msg.targets.begin(), msg.targets.end(),
      [](const Target& lhs, const Target& rhs) {
        return lhs.confidence < rhs.confidence;
      });
  }

  /**
   * @brief 验证目标数据是否可用于控制。
   *
   * 必要条件：目标在当前帧可见、状态允许控制、像素中心有限。
   * 深度不是云台/底盘偏航的前置条件；它由 translation_depth_valid()
   * 单独验证，避免纯2D轨迹被错误拒绝。
   *
   * 无效目标会导致控制律产生 NaN 指令，必须在此过滤。
   */
  bool is_valid_target(const Target& target) const
  {
    const auto center = target_center_pixels(target);
    return mvp_safety::target_visible_for_control(target) &&
      std::isfinite(center.first) && std::isfinite(center.second);
  }

  bool translation_depth_valid(const Target& target) const
  {
    return mvp_safety::metric_depth_valid(
      target, min_depth_confidence_, min_valid_depth_, max_valid_depth_);
  }

  /**
   * @brief 提取目标在图像平面上的中心像素坐标。
   *
   * 坐标来源（按优先级）：
   *   1. target.center：感知管线直接输出的像素/归一化中心
   *   2. target.bbox：若无 center，从 bbox 取平均值 (x1+x2)/2, (y1+y2)/2
   *
   * 归一化坐标检测与转换：
   *   - 若 bbox 所有值 ∈ [0,1] 且 center 也在此范围内，判定为归一化坐标
   *   - 归一化坐标自动乘以 image_width_/image_height_ 转为像素坐标
   *
   * @return (pixel_x, pixel_y) — 像素坐标，原点为图像左上角。
   */
  std::pair<double, double> target_center_pixels(const Target& target) const
  {
    double cx = target.center[0];
    double cy = target.center[1];
    const bool bbox_valid = bbox_has_positive_area(target);
    const bool center_finite = std::isfinite(cx) && std::isfinite(cy);
    const bool default_zero_center_without_bbox =
      center_finite && cx == 0.0 && cy == 0.0 && !bbox_valid;

    // center 字段可能为空或保持默认零值，回退到 bbox 中心。
    if (!center_finite || default_zero_center_without_bbox) {
      if (!bbox_valid) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {nan, nan};
      }
      cx = 0.5 * (target.bbox[0] + target.bbox[2]);
      cy = 0.5 * (target.bbox[1] + target.bbox[3]);
    }

    // 检测归一化坐标 → 转换为像素坐标
    if (bbox_looks_normalized(target) &&
        cx >= 0.0 && cx <= 1.0 && cy >= 0.0 && cy <= 1.0) {
      cx *= static_cast<double>(image_width_);
      cy *= static_cast<double>(image_height_);
    }

    return {cx, cy};
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // 主控制回路
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * @brief 主控制回路入口（50 Hz 定时器回调）。
   *
   * 每帧执行：
   *   1. 检查目标是否超时 → 超时则发布零速度并重置滤波器
   *   2. 调用 compute_control() 计算底盘+云台指令
   *   3. 调用 publish_command() 发布指令
   */
  void control_loop()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto current_time = now();
    if ((require_camera_info_ && !camera_info_received_) ||
        !has_recent_target(current_time)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MVP not ready (camera_info=%d, target=%d); publishing zero commands",
        camera_info_received_, has_valid_target_);
      has_valid_target_ = false;
      reset_measurement_filters();
      reset_command_filters();
      publish_zero_command();
      return;
    }

    const auto command = compute_control();
    publish_command(command);
  }

  /**
   * @brief 判断最近是否收到有效目标。
   *
   * 超时条件：当前时间 - 最后一次有效目标时间 > target_timeout_。
   * 使用 0.5s 超时容忍检测器偶尔漏检 1-2 帧。
   */
  bool has_recent_target(const rclcpp::Time& current_time) const
  {
    if (!has_valid_target_) {
      return false;
    }
    return (current_time - last_valid_target_time_).seconds() <= target_timeout_;
  }

  /**
   * @brief 核心控制律 — 三通道解耦 P 控制 + 可选的云台 IBVS。
   *
   * ── 信号流 ───────────────────────────────────────────────────────
   *
   *   像素中心 (px, py)     深度 depth            云台偏航 q_yaw
   *        │                     │                      │
   *        ▼                     ▼                      ▼
   *   归一化误差              深度误差                死区判断
   *   ex = (px-cx)/cx     ez = depth-desired    |q_yaw| > 0.087?
   *   ey = (py-cy)/cy           │                      │
   *        │                     │                      ▼
   *        ▼                     ▼                base_wz =
   *   低通滤波(α=0.5)      低通滤波(α=0.3)    K_base_yaw × q_yaw
   *        │                     │
   *        ▼                     ▼
   *   死区(0.05)            死区(0.2m)
   *        │                     │
   *   ┌────┴────┐               ▼
   *   ▼         ▼          base_vx =
   *  云台 P    云台 IBVS   K_base_z × ez
   *  (线性)    (交互矩阵)
   *   │         │                │
   *   └────┬────┘                │
   *        ▼                     ▼
   *   低通滤波(α=0.3)      低通滤波(α=0.3)
   *        │                     │
   *        ▼                     ▼
   *   限幅(±max)            限幅(±max)
   *        │                     │
   *        ▼                     ▼
   *   gimbal_yaw_vel       base_vx
   *   gimbal_pitch_vel     base_wz
   *
   * ── 两级串联控制（偏航通道）────────────────────────────────────
   *
   *   目标在图像右侧(ex>0) → 云台向右转 → q_yaw > 0 → 底盘向右转
   *   底盘右转使机器人对准目标 → 云台偏航角 q_yaw 减小 → 底盘停止
   *
   *   这种两级串联保证了：
   *     - 云台先锁定目标（内环，高带宽）
   *     - 底盘缓慢对正（外环，低带宽）
   *     - 目标始终不离开视野
   *
   * @return ControlCommand 包含四条经过滤波和限幅的指令。
   */
  ControlCommand compute_control()
  {
    // ── 步骤 1: 提取像素中心，计算归一化图像误差 ─────────────────
    //
    // 归一化：将像素偏移除以半幅图像尺寸，映射到 [-1, 1]。
    //   ex = +1: 目标在最右侧
    //   ex = -1: 目标在最左侧
    //   ex =  0: 目标在水平中心
    const auto center = target_center_pixels(active_target_);
    const double half_width = 0.5 * static_cast<double>(image_width_);
    const double half_height = 0.5 * static_cast<double>(image_height_);
    const double desired_x = center_reference_ == "principal_point" && camera_cx_ > 0.0
      ? camera_cx_ : half_width;
    const double desired_y = center_reference_ == "principal_point" && camera_cy_ > 0.0
      ? camera_cy_ : half_height;

    const double ex_raw = (center.first - desired_x) / half_width;
    const double ey_raw = (center.second - desired_y) / half_height;
    const double depth_raw = active_target_.position[2];
    const bool depth_valid = translation_depth_valid(active_target_);

    // ── 步骤 2: 第一级低通滤波（传感器噪声抑制）─────────────────
    //
    // 在计算控制量之前先对原始测量做平滑，避免检测器跳动导致指令抖振。
    // 首次收到有效目标时直接初始化滤波器状态，跳过平滑过渡。
    if (!error_filter_initialized_) {
      filtered_ex_ = ex_raw;
      filtered_ey_ = ey_raw;
      error_filter_initialized_ = true;
    } else {
      filtered_ex_ = low_pass(ex_raw, filtered_ex_, filter_alpha_error_);
      filtered_ey_ = low_pass(ey_raw, filtered_ey_, filter_alpha_error_);
    }

    if (enable_base_translation_ && depth_valid) {
      if (!depth_filter_initialized_) {
        filtered_depth_ = depth_raw;
        depth_filter_initialized_ = true;
      } else {
        filtered_depth_ = low_pass(depth_raw, filtered_depth_, filter_alpha_depth_);
      }
    } else {
      depth_filter_initialized_ = false;
      filtered_depth_ = 0.0;
    }

    // ── 步骤 3: 死区 — 微小误差不产生指令 ───────────────────────
    const double ex = apply_deadband(filtered_ex_, ex_deadband_);
    const double ey = apply_deadband(filtered_ey_, ey_deadband_);
    const double ez = enable_base_translation_ && depth_valid
      ? apply_deadband(filtered_depth_ - desired_distance_, depth_deadband_)
      : 0.0;
    const double q_yaw = current_gimbal_yaw();

    // ── 步骤 4: 底盘前向速度（深度控制） ─────────────────────────
    //
    //   base_vx = K_base_z × (depth - desired_distance)
    //
    // 目标太远 (depth > desired) → 正前向速度（靠近）
    // 目标太近 (depth < desired) → 负前向速度（后退）
    // 注意 ez 已经过死区，|ez| < depth_deadband_ 时 base_vx = 0。
    double base_vx = enable_base_translation_
      ? std::clamp(k_base_z_ * ez, -max_base_vx_, max_base_vx_)
      : 0.0;
    if (!enable_base_translation_ || !depth_valid) {
      // Metric depth is a hard safety gate, not a signal to low-pass. A stale
      // residual command must never survive loss of depth confidence.
      filtered_base_vx_ = 0.0;
    }

    // ── 步骤 5: 底盘偏航速度（云台偏航角跟踪） ───────────────────
    //
    //   base_wz = base_yaw_sign × K_base_yaw × q_yaw
    //   q_yaw = wrap(measured_yaw - startup_or_fixed_center)
    //
    // q_yaw 是云台相对于底盘中心的偏航角度。
    // 当云台转动跟踪目标后，q_yaw 偏离 0，底盘通过转动自身将云台带回中心。
    // 死区 0.087rad (≈5°) 防止底盘在云台微调时频繁晃动。
    double base_wz = 0.0;
    if (enable_base_yaw_ &&
        (use_mock_gimbal_state_ || platform_state_received_) &&
        std::abs(q_yaw) >= q_yaw_deadband_) {
      base_wz = mvp_yaw_control::base_yaw_command(
        q_yaw, k_base_yaw_, base_yaw_sign_, q_yaw_deadband_, max_base_wz_);
    }

    // ── 步骤 6: 云台指令（图像误差跟踪） ─────────────────────────
    //
    // 两种模式可选：
    //   use_ibvs_gimbal_=true:  单点特征 IBVS 角速度求解（考虑透视投影几何）
    //   use_ibvs_gimbal_=false: 纯 P 控制，软件符号由 yaw_sign/pitch_sign 标定
    const auto gimbal_command = enable_gimbal_tracking_
      ? (use_ibvs_gimbal_
          ? compute_ibvs_gimbal_velocity(center.first, center.second)
          : compute_proportional_gimbal_velocity(ex, ey))
      : std::pair<double, double>{0.0, 0.0};
    double gimbal_yaw_vel = gimbal_command.first;
    double gimbal_pitch_vel = gimbal_command.second;

    // ── 步骤 7: 第二级低通滤波（输出指令平滑） ───────────────────
    //
    // 在限幅之前做平滑，确保相邻帧指令连续，保护执行机构。
    // α=0.3 意味着约 3 帧（60ms）达到目标值的 63%。
    filtered_base_vx_ = low_pass(base_vx, filtered_base_vx_, filter_alpha_cmd_);
    filtered_base_wz_ = low_pass(base_wz, filtered_base_wz_, filter_alpha_cmd_);
    filtered_gimbal_yaw_vel_ = low_pass(
      gimbal_yaw_vel, filtered_gimbal_yaw_vel_, filter_alpha_cmd_);
    filtered_gimbal_pitch_vel_ = low_pass(
      gimbal_pitch_vel, filtered_gimbal_pitch_vel_, filter_alpha_cmd_);

    // ── 步骤 8: 限幅并打包指令 ────────────────────────────────────
    ControlCommand command;
    command.base_vx = std::clamp(filtered_base_vx_, -max_base_vx_, max_base_vx_);
    command.base_wz = std::clamp(filtered_base_wz_, -max_base_wz_, max_base_wz_);
    command.gimbal_yaw_vel = std::clamp(
      filtered_gimbal_yaw_vel_, -max_gimbal_yaw_vel_, max_gimbal_yaw_vel_);
    command.gimbal_pitch_vel = std::clamp(
      filtered_gimbal_pitch_vel_, -max_gimbal_pitch_vel_, max_gimbal_pitch_vel_);

    // ── 步骤 9: 节流调试日志 ─────────────────────────────────────
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 500,
      "MVP ex=%.3f ey=%.3f depth=%.3f depth_ok=%d ez=%.3f q_yaw=%.3f | "
      "vx=%.3f wz=%.3f gyaw=%.3f gpitch=%.3f",
      ex, ey, filtered_depth_, depth_valid, ez, q_yaw,
      command.base_vx, command.base_wz,
      command.gimbal_yaw_vel, command.gimbal_pitch_vel);

    return command;
  }

  /**
   * @brief 纯 P 控制云台 — 线性映射图像误差到角速度。
   *
   *   gimbal_yaw_vel   = yaw_sign   × K_gimbal_x × ex
   *   gimbal_pitch_vel = pitch_sign × K_gimbal_y × ey
   *
   * yaw_sign/pitch_sign 是真机方向标定参数，必须使执行后的像素误差绝对值减小。
   *
   * 这是最简单的控制方式，在目标接近图像中心时工作良好。
   * 当目标偏离中心较远时，透视投影的非线性会使响应不够精确。
   *
   * @return (yaw_rate, pitch_rate) — 已限幅到 [±max_gimbal_yaw_vel, ±max_gimbal_pitch_vel]。
   */
  std::pair<double, double> compute_proportional_gimbal_velocity(double ex, double ey) const
  {
    return {
      std::clamp(yaw_sign_ * k_gimbal_x_ * ex,
                 -max_gimbal_yaw_vel_, max_gimbal_yaw_vel_),
      std::clamp(pitch_sign_ * k_gimbal_y_ * ey,
                 -max_gimbal_pitch_vel_, max_gimbal_pitch_vel_)
    };
  }

  /**
   * @brief 单点特征 IBVS 角速度求解 — 云台的高级控制方式。
   *
   * ── 理论背景 ───────────────────────────────────────────────────
   *
   * 对位于归一化坐标 (x, y) = ((u-cx)/fx, (v-cy)/fy) 的点特征，
   * 其特征运动与相机角速度的关系由角速度交互矩阵 Lw(2×3) 描述：
   *
   *   Lw = [  xy    -(1+x²)   y  ]
   *        [ 1+y²    -xy     -x  ]
   *
   *   ẋ = Lw · ω    (ω = [ωx, ωy, ωz]ᵀ)
   *
   * 期望特征位置为主点 (0, 0)（即目标在图像正中央），误差 e = [x, y]ᵀ。
   *
   * 控制律（阻尼最小二乘）：
   *   ω = -λ · (LwᵀLw + δ²I)⁻¹ · Lwᵀ · e
   *
   *   其中 λ = ibvs_gimbal_gain_（增益）
   *        δ = ibvs_damping_（阻尼系数，防止 Lw 奇异时 ω → ∞）
   *
   * ── 云台通道映射 ──────────────────────────────────────────────
   *
   * camera_optical_link 的角速度 → 云台关节角速度：
   *   gimbal_yaw_rate   = -ωy_camera    (绕 optical y 的旋转 → 云台偏航)
   *   gimbal_pitch_rate = -ωx_camera    (绕 optical x 的旋转 → 云台俯仰)
   *
   * ωz_camera（绕光轴的 roll）两轴云台无法执行，当前 MVP 忽略。
   *
   * ── IBVS vs P 控制对比 ────────────────────────────────────────
   *
   *   | 条件             | P 控制              | IBVS                 |
   *   | 目标在中心附近   | 效果相当            | 效果相当               |
   *   | 目标偏离中心较远 | 线性近似不准确       | 考虑透视几何，更准确    |
   *   | 计算复杂度       | O(1)                | O(3³) 矩阵分解         |
   *
   * @param pixel_x 目标中心的像素 x 坐标
   * @param pixel_y 目标中心的像素 y 坐标
   * @return (yaw_rate, pitch_rate) — 已限幅到 [±max_gimbal_yaw_vel, ±max_gimbal_pitch_vel]。
   */
  std::pair<double, double> compute_ibvs_gimbal_velocity(double pixel_x, double pixel_y) const
  {
    // 获取相机内参（≤0 时用半幅图像尺寸回退，相当于视场角 ~90° 的估计）
    const double fx = camera_fx_ > 0.0 ? camera_fx_ : 0.5 * static_cast<double>(image_width_);
    const double fy = camera_fy_ > 0.0 ? camera_fy_ : 0.5 * static_cast<double>(image_height_);
    const double cx = center_reference_ == "principal_point" && camera_cx_ > 0.0
      ? camera_cx_ : 0.5 * static_cast<double>(image_width_);
    const double cy = center_reference_ == "principal_point" && camera_cy_ > 0.0
      ? camera_cy_ : 0.5 * static_cast<double>(image_height_);

    // 焦距非法 → 零输出（安全回退）
    if (fx <= 1e-6 || fy <= 1e-6) {
      return {0.0, 0.0};
    }

    // ── 归一化坐标 ─────────────────────────────────────────────
    // (x, y) 在归一化图像平面上，单位为"焦距倍数"。
    // 主点 (cx, cy) 对应 (0, 0)，即期望的特征位置。
    const double x = (pixel_x - cx) / fx;
    const double y = (pixel_y - cy) / fy;

    // ── 构建角速度交互矩阵 Lw(2×3) ────────────────────────────
    //
    // Lw = [  xy    -(1+x²)   y  ]
    //      [ 1+y²    -xy     -x  ]
    //
    // 这是交互矩阵 L(2×6) = [Lv | Lw] 的角速度部分。
    // 由于云台仅做旋转（不参与平移），只取 Lw 子矩阵。
    Eigen::Matrix<double, 2, 3> angular_interaction;
    angular_interaction <<
      x * y, -(1.0 + x * x), y,
      1.0 + y * y, -x * y, -x;

    // ── 特征误差：期望 (0,0) vs 当前 (x,y) ────────────────────
    const Eigen::Vector2d error(x, y);

    // ── 阻尼最小二乘求解 ω ─────────────────────────────────────
    //
    // 标准最小二乘：ω = -λ · Lw⁺ · e，其中 Lw⁺ = (LwᵀLw)⁻¹Lwᵀ
    // 阻尼最小二乘：ω = -λ · (LwᵀLw + δ²I)⁻¹ · Lwᵀ · e
    //
    // δ²I 项确保即使 Lw 秩亏（如目标恰好在主点时 Lw 的列线性相关），
    // 矩阵仍可逆，ω 不会发散到无穷大。δ 通常取小值（0.01）。
    const double damping = std::max(0.0, ibvs_damping_);
    const Eigen::Matrix3d normal =
      angular_interaction.transpose() * angular_interaction +
      damping * damping * Eigen::Matrix3d::Identity();

    // LDLT 分解求解（比直接求逆更数值稳定）
    const Eigen::Vector3d camera_omega =
      -ibvs_gimbal_gain_ *
      normal.ldlt().solve(angular_interaction.transpose() * error);

    // ── 映射到云台关节空间 ─────────────────────────────────────
    //
    // camera_optical_link 的角速度到二轴云台关节角速度的映射：
    //   gimbal_yaw_rate   = -camera_wy  (optical y 轴旋转 → yaw 关节)
    //   gimbal_pitch_rate = -camera_wx  (optical x 轴旋转 → pitch 关节)
    //
    // camera_wz（光轴 roll）二轴云台无法执行，忽略。
    const double yaw_rate = std::clamp(
      yaw_sign_ * camera_omega.y(), -max_gimbal_yaw_vel_, max_gimbal_yaw_vel_);
    const double pitch_rate = std::clamp(
      pitch_sign_ * camera_omega.x(), -max_gimbal_pitch_vel_, max_gimbal_pitch_vel_);

    return {yaw_rate, pitch_rate};
  }

  /**
   * @brief 获取当前云台偏航角。
   *
   * 来源优先级：
   *   1. use_mock_gimbal_state_=true → 使用参数 mock_q_yaw_（固定值，用于独立测试）
   *   2. 从未收到 PlatformState → 使用 mock_q_yaw_ 回退 + 日志警告
   *   3. 正常模式 → 返回相对于启动/固定零位的最短角距离
   *
   * mock 模式默认 q_yaw=0，意味着底盘偏航通道始终输出零 — 适合单纯测试云台跟踪。
   *
   * @return 当前云台相对底盘正前方的偏航误差 (rad)，范围 [-pi, pi]。
   */
  double current_gimbal_yaw()
  {
    if (use_mock_gimbal_state_) {
      return mock_q_yaw_;
    }

    if (!platform_state_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No PlatformState received yet; using mock_q_yaw fallback");
      return mock_q_yaw_;
    }

    const double reference =
      capture_gimbal_yaw_center_on_startup_ && gimbal_yaw_center_captured_
      ? captured_gimbal_yaw_center_
      : configured_gimbal_yaw_center_;
    return mvp_yaw_control::wrapped_angle_error(platform_q_yaw_, reference);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // 指令发布
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * @brief 重置指令滤波器状态为零。
   *
   * 在目标丢失时调用，避免滤波器保留上一目标的历史值，
   * 导致恢复跟踪后产生过渡过程中的指令跳变。
   */
  void reset_command_filters()
  {
    filtered_base_vx_ = 0.0;
    filtered_base_wz_ = 0.0;
    filtered_gimbal_yaw_vel_ = 0.0;
    filtered_gimbal_pitch_vel_ = 0.0;
  }

  /// 重置测量滤波器，使重新捕获目标时不会混入上一目标的误差历史。
  void reset_measurement_filters()
  {
    error_filter_initialized_ = false;
    depth_filter_initialized_ = false;
    filtered_ex_ = 0.0;
    filtered_ey_ = 0.0;
    filtered_depth_ = 0.0;
  }

  /// 安全停车：发布所有通道的零指令。
  void publish_zero_command()
  {
    publish_command(ControlCommand{});
  }

  /**
   * @brief 发布底盘和云台指令。
   *
   * 发布两个话题：
   *   - /cmd_vel：TwistStamped(base_link) 或 Twist，取决于 use_twist_stamped_
   *   - /cmd_gimbal：GimbalCmd(yaw_rate, pitch_rate, hold_yaw, hold_pitch)
   *
   * hold_yaw/hold_pitch 在此控制器中始终为 false（不锁定），
   * 因为目标丢失时由 publish_zero_command() 直接发布零速度，
   * 不需要云台保持当前位置（与 servo_manager 的设计不同）。
   */
  void publish_command(const ControlCommand& command)
  {
    const auto stamp = now();

    // ── 底盘指令 ─────────────────────────────────────────────────
    if (use_twist_stamped_ && cmd_vel_stamped_pub_) {
      auto twist_stamped = geometry_msgs::msg::TwistStamped();
      twist_stamped.header.stamp = stamp;
      twist_stamped.header.frame_id = "base_link";      // 速度在机器人本体坐标系下
      twist_stamped.twist.linear.x = command.base_vx;    // 前向速度 (m/s)
      twist_stamped.twist.angular.z = command.base_wz;   // 偏航角速度 (rad/s)
      // linear.y (横向) 和 angular.x/y 保持 0（全向底盘暂不使用横向平移）
      cmd_vel_stamped_pub_->publish(twist_stamped);
    } else if (cmd_vel_pub_) {
      auto twist = geometry_msgs::msg::Twist();
      twist.linear.x = command.base_vx;
      twist.angular.z = command.base_wz;
      cmd_vel_pub_->publish(twist);
    }

    // ── 云台指令 ─────────────────────────────────────────────────
    auto gimbal_cmd = vision_servo_msgs::msg::GimbalCmd();
    gimbal_cmd.header.stamp = stamp;
    gimbal_cmd.yaw_rate = static_cast<float>(command.gimbal_yaw_vel);
    gimbal_cmd.pitch_rate = static_cast<float>(command.gimbal_pitch_vel);
    gimbal_cmd.hold_yaw = std::abs(command.gimbal_yaw_vel) < 1e-6;
    gimbal_cmd.hold_pitch = std::abs(command.gimbal_pitch_vel) < 1e-6;
    cmd_gimbal_pub_->publish(gimbal_cmd);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // 成员变量
  // ═══════════════════════════════════════════════════════════════════════════

  // ── A) 话题名称（支持 remap）────────────────────────────────────
  std::string target_topic_;
  std::string platform_state_topic_;
  std::string camera_info_topic_;
  std::string cmd_vel_topic_;
  std::string cmd_gimbal_topic_;

  // ── B) 消息格式与调试开关 ────────────────────────────────────────
  bool use_twist_stamped_ = true;         ///< true=TwistStamped, false=Twist
  bool use_mock_gimbal_state_ = true;     ///< true=使用参数 mock_q_yaw/pitch
  bool enable_gimbal_tracking_ = true;
  bool enable_base_yaw_ = false;
  bool enable_base_translation_ = false;

  // ── C) 相机参数 ──────────────────────────────────────────────────
  int image_width_ = 640;                 ///< 图像宽度 (px)
  int image_height_ = 480;                ///< 图像高度 (px)
  double control_rate_hz_ = 50.0;         ///< 控制回路频率 (Hz)
  bool require_camera_info_ = false;
  bool camera_info_received_ = false;
  std::string center_reference_ = "image_center";

  // ── D) 控制目标 ──────────────────────────────────────────────────
  double desired_distance_ = 2.0;         ///< 期望目标距离 (m)

  // ── E) P 控制增益（四通道）──────────────────────────────────────
  // 遵循"云台快、底盘慢"的原则：云台增益 (0.6-0.8) > 底盘增益 (0.4-0.5)
  double k_gimbal_x_ = 0.8;              ///< 云台偏航增益 (rad/s / normalized_error)
  double k_gimbal_y_ = 0.6;              ///< 云台俯仰增益 (rad/s / normalized_error)
  double k_base_z_ = 0.4;                ///< 底盘前向增益 (m/s / m_depth_error)
  double k_base_yaw_ = 0.5;              ///< 底盘偏航增益 (rad/s / rad_yaw_error)
  double base_yaw_sign_ = -1.0;           ///< 云台相对偏航到 ROS angular.z 的方向标定
  double yaw_sign_ = -1.0;
  double pitch_sign_ = -1.0;

  // ── F) IBVS 参数（云台高级控制）────────────────────────────────
  bool use_ibvs_gimbal_ = true;           ///< true=IBVS, false=纯P控制
  double ibvs_gimbal_gain_ = 0.8;         ///< IBVS 增益 λ
  double ibvs_damping_ = 0.01;            ///< 阻尼最小二乘系数 δ
  double camera_fx_ = 0.0;               ///< 焦距 fx (px), ≤0 时自动推断
  double camera_fy_ = 0.0;               ///< 焦距 fy (px), ≤0 时自动推断
  double camera_cx_ = 0.0;               ///< 主点 cx (px), ≤0 时自动推断
  double camera_cy_ = 0.0;               ///< 主点 cy (px), ≤0 时自动推断

  // ── G) 死区阈值 ─────────────────────────────────────────────────
  //
  // 信号低于死区时输出零，避免执行器围绕零点高频微振。
  double ex_deadband_ = 0.05;             ///< 水平误差死区（归一化单位 [-1,1]）
  double ey_deadband_ = 0.05;             ///< 垂直误差死区（归一化单位 [-1,1]）
  double depth_deadband_ = 0.2;           ///< 深度误差死区 (m)
  double q_yaw_deadband_ = 0.0873;        ///< 云台偏航死区 (rad ≈ 5°)
  double min_depth_confidence_ = 0.6;
  double min_valid_depth_ = 0.3;
  double max_valid_depth_ = 10.0;

  // ── H) 执行器限幅 ─────────────────────────────────────────────────
  //
  // 防止控制律输出超出电机/驱动的物理能力。
  double max_gimbal_yaw_vel_ = 0.8;       ///< 云台偏航最大角速度 (rad/s)
  double max_gimbal_pitch_vel_ = 0.6;     ///< 云台俯仰最大角速度 (rad/s)
  double max_base_vx_ = 0.4;              ///< 底盘最大线速度 (m/s)
  double max_base_wz_ = 0.6;              ///< 底盘最大角速度 (rad/s)

  // ── I) 低通滤波系数 ──────────────────────────────────────────────
  //
  // α ∈ [0, 1]：1 = 不过滤，值越小平滑越强。
  // 深度滤波系数最小（0.3），因为单目深度估计噪声最大。
  double filter_alpha_error_ = 0.5;       ///< 图像误差滤波系数
  double filter_alpha_depth_ = 0.3;       ///< 深度滤波系数
  double filter_alpha_cmd_ = 0.3;         ///< 输出指令滤波系数

  // ── J) 目标超时 ──────────────────────────────────────────────────
  double target_timeout_ = 0.5;           ///< 目标丢失判定超时 (s)

  // ── K) Mock 云台状态（独立测试用）──────────────────────────────
  double mock_q_yaw_ = 0.0;               ///< mock 云台偏航角 (rad)
  double mock_q_pitch_ = 0.0;             ///< mock 云台俯仰角 (rad)
  bool capture_gimbal_yaw_center_on_startup_ = false;
  double configured_gimbal_yaw_center_ = 0.0;
  double captured_gimbal_yaw_center_ = 0.0;
  bool gimbal_yaw_center_captured_ = false;

  // ── L) 运行时状态 — 目标数据 ────────────────────────────────────
  Target active_target_;                  ///< 当前伺服跟踪的目标（值拷贝）
  bool has_valid_target_ = false;         ///< 是否有有效目标可供控制
  rclcpp::Time last_valid_target_time_;   ///< 最后一次收到有效目标的时刻

  // ── M) 运行时状态 — 平台反馈 ────────────────────────────────────
  bool platform_state_received_ = false;  ///< 是否已收到过至少一次 PlatformState
  double platform_q_yaw_ = 0.0;           ///< 缓存的云台偏航角 (rad)
  double platform_q_pitch_ = 0.0;         ///< 缓存的云台俯仰角 (rad)

  // ── N) 运行时状态 — 滤波器记忆 ──────────────────────────────────
  //
  // 第一级滤波（传感器端）：抑制检测噪声
  bool error_filter_initialized_ = false; ///< 图像误差滤波器是否已初始化
  bool depth_filter_initialized_ = false; ///< 深度滤波器是否已初始化
  double filtered_ex_ = 0.0;              ///< 滤波后的水平误差
  double filtered_ey_ = 0.0;              ///< 滤波后的垂直误差
  double filtered_depth_ = 0.0;           ///< 滤波后的目标深度 (m)

  // 第二级滤波（输出端）：平滑指令跳变
  double filtered_base_vx_ = 0.0;         ///< 滤波后的底盘前向速度 (m/s)
  double filtered_base_wz_ = 0.0;         ///< 滤波后的底盘偏航角速度 (rad/s)
  double filtered_gimbal_yaw_vel_ = 0.0;  ///< 滤波后的云台偏航角速度 (rad/s)
  double filtered_gimbal_pitch_vel_ = 0.0;///< 滤波后的云台俯仰角速度 (rad/s)

  // ── O) ROS2 通信接口 ─────────────────────────────────────────────
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr target_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::PlatformState>::SharedPtr platform_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr cmd_gimbal_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;  ///< 50 Hz wall timer
  std::mutex state_mutex_;
};

}  // namespace servo_control_pkg

// ═══════════════════════════════════════════════════════════════════════════════
// 入口
// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // spin 阻塞主线程直到 SIGINT/SIGTERM 或节点退出。
  // 所有控制逻辑（50Hz 定时器、订阅回调）均在 spin 内部的事件循环中执行。
  rclcpp::spin(std::make_shared<servo_control_pkg::MvpFollowControllerNode>());
  rclcpp::shutdown();
  return 0;
}
