/**
 * @file servo_manager_node.cpp
 * @brief 伺服管理节点 — 视觉伺服控制系统的核心调度器。
 *
 * ── 在系统中的位置 ─────────────────────────────────────────────────
 *
 *   servo_manager_node 位于感知与执行之间：
 *
 *   /perception/targets_3d ──→ [servo_manager_node] ──→ /cmd_vel (底盘)
 *   /camera/camera_info    ──→    │                    ──→ /cmd_gimbal (云台)
 *   /platform/state        ──→    │                    ──→ /servo/state (监控)
 *                                 │
 *                          pluginlib::ClassLoader
 *                                 │
 *                    ┌────────────┼────────────┐
 *                    ▼            ▼            ▼
 *               IBVSController PBVSController (MPC/RL 占位)
 *
 * ── 控制回路（control_loop, 50 Hz = 20ms 周期）──────────────────
 *
 *   每帧执行以下流水线：
 *
 *     目标数据               控制器                分配器              执行器
 *   ┌──────────┐   ┌──────────────────┐   ┌──────────────┐   ┌──────────────┐
 *   │ Target   │──→│ computeVelocity()│──→│ allocate()   │──→│ /cmd_vel     │
 *   │ (bbox,   │   │ v = -λ·L⁺·(s-s*) │   │ 相机速度 →   │    │ /cmd_gimbal   │
 *   │  pos[3]) │   │ 6-DOF 相机速度    │   │ 底盘+云台指令│    │ /servo/state  │
 *   └──────────┘   └──────────────────┘   └──────────────┘   └──────────────┘
 *
 *   步骤详解：
 *     1. 从 active_target_ 读取当前帧目标（已在 target_callback 中缓存）
 *     2. 调用 controller_->computeVelocity(target, dt) → 6-DOF 相机速度
 *     3. 调用 allocator_.allocate(cam_vel, platform_state, dt) → 底盘 twist + 云台速率
 *     4. 发布 /cmd_vel（TwistStamped, frame_id=base_link）
 *     5. 发布 /cmd_gimbal（GimbalCmd: yaw_rate, pitch_rate, hold 标志位）
 *     6. 发布 /servo/state（ServoState: 误差、条件数、状态机阶段）
 *
 * ── 多控制器架构 ──────────────────────────────────────────────────
 *
 *   控制器通过 pluginlib 以共享库 (.so) 形式加载，支持运行时热切换：
 *     - 构造函数：根据 "controller_plugin" 参数加载初始控制器
 *     - set_mode_callback：响应 /servo/set_mode 服务，创建新控制器实例
 *     - 切换时保留相机标定状态，若已有伺服目标则自动重新配置
 *
 *   plugin_map 将 mode ID (0-4) 映射到插件类名：
 *     mode 0 → IBVSController（唯一完整实现）
 *     mode 1 → PBVSController
 *     mode 2-4 → IBVSController（HYBRID/MPC/RL 占位，待实现）
 *
 * ── 动作服务器 ────────────────────────────────────────────────────
 *
 *   VisualServo action (/servo/visual_servo) 支持长周期伺服任务：
 *     - 接受目标 ID、期望深度、收敛容差、超时时间
 *     - 阻塞等待目标出现和相机标定完成
 *     - 周期性发布反馈（当前误差、进度百分比）
 *     - TRACKING 状态持续跟随；到达时长/超时或取消后结束
 *     - 独占执行：同一时刻只允许一个 action
 *
 * ── 自动启动（auto_start）─────────────────────────────────────────
 *
 *   当 auto_start=true 时，节点在收到第一个有效目标后自动开始伺服，
 *   无需通过 action 显式触发。适用于仿真测试和一键启动场景。
 *
 * ── 线程安全 ──────────────────────────────────────────────────────
 *
 *   state_mutex_ 保护所有共享状态（controller_、last_target_、
 *   active_target_、last_platform_state_）。control_loop（定时器线程）、
 *   target_callback（订阅者线程）、set_mode_callback（服务线程）、
 *   execute_servo（action 线程）均通过此锁同步。
 *
 * ── 话题 QoS 策略 ─────────────────────────────────────────────────
 *
 *   - /perception/targets_3d：control_cmd()（Best-effort, 低延迟）
 *   - /platform/state：platform_state()（Best-effort, 低延迟）
 *   - /camera/camera_info：TRANSIENT_LOCAL（新加入节点可立即获取历史标定）
 *   - /cmd_vel, /cmd_gimbal：control_cmd()（Best-effort, 控制指令不重传）
 *   - /servo/state：servo_state()（Reliable, 状态监控不丢帧）
 */

#include "servo_control_pkg/servo_controller_base.hpp"
#include "servo_control_pkg/control_allocator.hpp"
#include "servo_control_pkg/qos.hpp"
#include <vision_servo_msgs/msg/target_array.hpp>
#include <vision_servo_msgs/msg/servo_state.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <vision_servo_msgs/msg/platform_state.hpp>
#include <vision_servo_msgs/srv/set_servo_mode.hpp>
#include <vision_servo_msgs/action/visual_servo.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace servo_control_pkg {

class ServoManagerNode : public rclcpp::Node {
public:
  using VisualServo = vision_servo_msgs::action::VisualServo;
  using GoalHandleVisualServo = rclcpp_action::ServerGoalHandle<VisualServo>;//它是服务端手里握着的某个具体 action 请求的句柄

  explicit ServoManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("servo_manager", options)
  {
    // ═══════════════════════════════════════════════════════════════════════
    // 1. 声明参数并加载控制器插件
    // ═══════════════════════════════════════════════════════════════════════
    //
    // 参数分为三类：
    //   A) 节点级参数：controller_plugin、auto_start、target_timeout 等
    //   B) 控制器公共参数（第 70-81 行）：通过 configureFromNode() 下发到
    //      插件内部。之所以在 servo_manager 侧声明并中转，是因为 pluginlib
    //      创建的 Node 名与 YAML 节点名（servo_manager）不匹配，ROS 参数
    //      系统无法将 YAML 参数自动路由到插件节点。由 servo_manager 统一
    //      声明后通过 configureFromNode() 手动同步，可保证调参生效。
    //   C) 分配器参数：gimbal_yaw_limit 等，传给 ControlAllocator::configure()。
    //
    // 默认控制器为 IBVS，可通过 YAML 覆盖为 PBVS。
    this->declare_parameter("controller_plugin", "servo_control_pkg::IBVSController");
    this->declare_parameter("allocation_ratio", 0.5);        // 分配比例（底盘 vs 云台）
    this->declare_parameter("gimbal_yaw_limit", M_PI);       // 云台偏航限位 (rad)
    this->declare_parameter("gimbal_pitch_limit", M_PI_2);   // 云台俯仰限位 (rad)
    this->declare_parameter("chassis_linear_limit", 1.0);    // 底盘线速度上限 (m/s)
    this->declare_parameter("chassis_angular_limit", 2.0);   // 底盘角速度上限 (rad/s)
    this->declare_parameter("auto_start", false);            // 是否自动开始伺服
    this->declare_parameter("target_timeout", 0.5);          // 目标丢失判定超时 (s)
    this->declare_parameter("publish_unstamped_cmd_vel", false);  // 仿真兼容：发布无时间戳 Twist

    // ── 控制器公共参数（中转：servo_manager → controller_->configureFromNode()）──
    this->declare_parameter("lambda_gain", 0.5);             // 基础控制增益 λ
    this->declare_parameter("max_linear_velocity", 1.0);     // 最大线速度 (m/s)
    this->declare_parameter("max_angular_velocity", 2.0);    // 最大角速度 (rad/s)
    this->declare_parameter("feature_tolerance", 0.01);      // 特征误差收敛阈值
    this->declare_parameter("desired_depth", 2.0);           // 期望目标深度 (m)
    this->declare_parameter("debug_print_rate", 2.0);        // 控制台调试打印频率 (Hz)，0=关闭
    // IBVS 专属参数（PBVS 会忽略它们）
    this->declare_parameter("gain_max", 1.0);                // 自适应增益 λ_max
    this->declare_parameter("gain_min", 0.1);                // 自适应增益 λ_min
    this->declare_parameter("error_threshold_slow", 0.05);   // 增益减速阈值
    this->declare_parameter("use_adaptive_gain", true);      // 是否启用自适应增益
    // PBVS 专属参数（IBVS 会忽略它们）
    this->declare_parameter("translational_gain", 0.5);      // PBVS 平移增益
    this->declare_parameter("rotational_gain", 0.3);         // PBVS 旋转增益

    std::string plugin_name = this->get_parameter("controller_plugin").as_string();
    auto_start_ = this->get_parameter("auto_start").as_bool();
    target_timeout_ = this->get_parameter("target_timeout").as_double();
    publish_unstamped_cmd_vel_ =
      this->get_parameter("publish_unstamped_cmd_vel").as_bool();

    // ── 插件加载：通过 pluginlib::ClassLoader 发现并实例化控制器 ──
    //
    // ClassLoader 在运行时搜索 plugins.xml 中注册的共享库 (.so)，
    // 通过基类 ServoControllerBase 的工厂方法创建实例。
    // 加载失败（.so 缺失、类名错误、链接错误）是致命错误，直接 throw。
    try {
      controller_loader_ = std::make_unique<pluginlib::ClassLoader<ServoControllerBase>>(
        "servo_control_pkg",                     // 包名（用于定位 plugins.xml）
        "servo_control_pkg::ServoControllerBase"); // 基类全限定名
      controller_ = controller_loader_->createSharedInstance(plugin_name);
      // 立即将 servo_manager 侧的参数同步到控制器内部
      controller_->configureFromNode(*this);
      RCLCPP_INFO(get_logger(), "已加载控制器插件: %s", plugin_name.c_str());
    } catch (const pluginlib::PluginlibException& e) {
      RCLCPP_ERROR(get_logger(), "控制器插件加载失败: %s", e.what());
      throw;  // 致命错误，节点无法继续运行
    }

    // ═══════════════════════════════════════════════════════════════════════
    // 2. 配置控制分配器
    // ═══════════════════════════════════════════════════════════════════════
    //
    // ControlAllocator 将 6-DOF 相机速度分解为底盘 (vx, wz) + 云台 (yaw_rate, pitch_rate)。
    // 分配策略基于 allocation_ratio 在底盘和云台之间分配运动自由度。
    allocator_.configure(
      this->get_parameter("gimbal_yaw_limit").as_double(),
      this->get_parameter("gimbal_pitch_limit").as_double(),
      this->get_parameter("chassis_linear_limit").as_double(),
      this->get_parameter("chassis_angular_limit").as_double(),
      this->get_parameter("allocation_ratio").as_double()
    );

    // ═══════════════════════════════════════════════════════════════════════
    // 3. 订阅者 — 接收上游数据
    // ═══════════════════════════════════════════════════════════════════════

    // 感知管线输出的 3D 目标位姿及边界框。
    // 消息包含 targets 数组（多目标）和 tracking_id（跟踪器指定的当前目标）。
    // QoS 使用 Best-effort：控制回路丢一帧比等一帧更重要。
    target_sub_ = this->create_subscription<vision_servo_msgs::msg::TargetArray>(
      "/perception/targets_3d", qos::control_cmd(),
      std::bind(&ServoManagerNode::target_callback, this, std::placeholders::_1));

    // 平台状态（底盘线速度/角速度、云台当前 yaw/pitch 角度）。
    // ControlAllocator 需要这些信息来避免指令跳变和利用当前运动状态。
    platform_sub_ = this->create_subscription<vision_servo_msgs::msg::PlatformState>(
      "/platform/state", qos::platform_state(),
      std::bind(&ServoManagerNode::platform_callback, this, std::placeholders::_1));

    // 相机内参标定信息。
    // TRANSIENT_LOCAL QoS：发布者只需发布一次，所有迟加入的订阅者均可获取
    // 最新的标定消息，无需等待发布者周期性重发。这对离线标定或启动顺序不
    // 固定的多节点系统至关重要。
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ServoManagerNode::camera_info_callback, this, std::placeholders::_1));

    // ═══════════════════════════════════════════════════════════════════════
    // 4. 发布者 — 向下游执行器发送指令和状态
    // ═══════════════════════════════════════════════════════════════════════

    // 底盘速度指令：TwistStamped（带时间戳和 frame_id=base_link），
    // 由底盘驱动节点（如 DiffDriveController）消费。
    chassis_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", qos::control_cmd());

    // 兼容发布：某些仿真插件（如 Gazebo Twist 插件）需要无时间戳的 Twist。
    // 仅当 publish_unstamped_cmd_vel=true 时启用，默认关闭以节省带宽。
    if (publish_unstamped_cmd_vel_) {
      chassis_cmd_unstamped_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel_unstamped", qos::control_cmd());
    }

    // 云台速率指令：GimbalCmd 包含 yaw_rate/pitch_rate 和 hold 标志位，
    // 由云台驱动节点消费。hold 位用于在目标丢失时维持当前位置。
    gimbal_cmd_pub_ = this->create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      "/cmd_gimbal", qos::control_cmd());

    // 伺服状态监控：ServoState 包含误差范数、条件数、状态机阶段等。
    // QoS 使用 Reliable：监控信息不应丢帧，以便调试和性能分析。
    servo_state_pub_ = this->create_publisher<vision_servo_msgs::msg::ServoState>(
      "/servo/state", qos::servo_state());

    // ═══════════════════════════════════════════════════════════════════════
    // 5. 服务 — 运行时热切换控制器
    // ═══════════════════════════════════════════════════════════════════════
    servo_mode_srv_ = this->create_service<vision_servo_msgs::srv::SetServoMode>(
      "/servo/set_mode",
      std::bind(&ServoManagerNode::set_mode_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ═══════════════════════════════════════════════════════════════════════
    // 6. 动作服务器 — 长周期伺服任务
    // ═══════════════════════════════════════════════════════════════════════
    //
    // VisualServo action 允许客户端提交带有超时和容差参数的伺服任务，
    // 持续监控收敛进度，并支持中途取消。三个回调分别处理：
    //   handle_goal   — 校验并接受/拒绝新目标
    //   handle_cancel — 接受取消请求（总是接受）
    //   handle_accepted — 启动独立线程执行任务
    servo_action_ = rclcpp_action::create_server<vision_servo_msgs::action::VisualServo>(
      this, "/servo/visual_servo",
      std::bind(&ServoManagerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ServoManagerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ServoManagerNode::handle_accepted, this, std::placeholders::_1));

    // ═══════════════════════════════════════════════════════════════════════
    // 7. 控制回路定时器 — 50 Hz 主循环
    // ═══════════════════════════════════════════════════════════════════════
    //
    // 使用 wall timer（而非 ROS timer）：确保稳定的 20ms 周期，
    // 不受仿真时钟暂停/加速的影响。对于真实硬件的实时控制，应考虑
    // 改用硬件定时器或 RT 内核。
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ServoManagerNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "伺服管理器已启动 (controller=%s, 50Hz loop)",
                plugin_name.c_str());
  }

  /**
   * @brief 析构 — 安全关闭伺服并等待 action 线程退出。
   *
   * 先置 servo_active_=false（通知 execute_servo 线程退出其主循环），
   * 再 join 等待线程完全结束。这确保析构返回时所有资源已安全释放。
   */
  ~ServoManagerNode() override {
    servo_active_.store(false);
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
  }

private:
  /**
   * @brief 相机内参标定回调。
   *
   * 收到 CameraInfo 消息后，提取针孔模型内参并初始化控制器。
   * 利用 TRANSIENT_LOCAL QoS 特性，无论节点何时启动都能获取标定数据。
   *
   * 内参矩阵 K (3×3) 在 CameraInfo::k 中的布局：
   *   K = [k[0], k[1], k[2];   = [fx,  0, cx;
   *        k[3], k[4], k[5];       0, fy, cy;
   *        k[6], k[7], k[8]]       0,  0,  1]
   *
   * 仅初始化一次（calibrated_ 标志锁定），后续的 CameraInfo 消息被忽略。
   * 若需运行时更新标定，应通过 SetServoMode 切换控制器实例并重新标定。
   */
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!calibrated_) {
      camera_fx_ = msg->k[0];
      camera_fy_ = msg->k[4];
      camera_cx_ = msg->k[2];
      camera_cy_ = msg->k[5];
      camera_width_ = static_cast<int>(msg->width);
      camera_height_ = static_cast<int>(msg->height);
      controller_->initialize(msg->k[0], msg->k[4], msg->k[2], msg->k[5],
                              msg->width, msg->height);
      calibrated_ = true;
    }
  }

  /**
   * @brief 感知目标回调 — 缓存最新目标数组并选择当前伺服目标。
   *
   * 目标选择策略（按优先级）：
   *   1. tracking_id ≥ 0：搜索 targets 数组中 id 匹配的目标
   *      - 找到 → 锁定该目标（跨帧跟踪一致性由感知管线的 tracker 保证）
   *      - 未找到（tracker 丢帧、遮挡）→ 回退到 targets[0]
   *   2. tracking_id < 0：直接取 targets[0]（感知管线已按置信度降序排列）
   *
   * 此回调仅做缓存和选择，不执行控制。实际的伺服计算在 control_loop() 中
   * 以固定频率进行，实现感知与控制的解耦。
   *
   * @note 线程安全：修改 last_target_、active_target_、last_target_time_ 均受 state_mutex_ 保护。
   */
  void target_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg) {
    if (!msg->targets.empty()) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_target_ = msg;                       // 保持 shared_ptr，避免拷贝整个数组
      last_target_time_ = this->now();          // 记录接收时间，用于 target_stale 判定

      if (msg->tracking_id >= 0) {
        bool found = false;
        for (auto& t : msg->targets) {
          if (t.id == msg->tracking_id) {
            active_target_ = t;
            found = true;
            break;
          }
        }
        if (!found) {
          // 跟踪 ID 未在当前帧找到：可能遮挡或 tracker 重初始化
          active_target_ = msg->targets[0];
        }
      } else {
        // 无跟踪 ID → 选择置信度最高的目标（targets[0]）
        active_target_ = msg->targets[0];
      }
    }
    // 注意：当 msg->targets 为空时，不更新 last_target_，保留上一帧数据。
    // 连续空帧会触发 target_timeout_ 超时 → control_loop 发布零速度。
  }

  /**
   * @brief 平台状态回调 — 缓存最新的底盘/云台运动状态。
   *
   * PlatformState 包含：
   *   - chassis_twist：当前底盘线速度 (vx) 和角速度 (wz)
   *   - gimbal_angles：云台当前 yaw/pitch 角度
   *
   * ControlAllocator::allocate() 利用这些信息进行：
   *   - 平滑过渡：避免指令跳变（基于当前状态的增量式控制）
   *   - 限位保护：当云台接近机械限位时，将运动分配转移到底盘
   *
   * @note 采用值拷贝（*msg）而非 shared_ptr，因为 PlatformState 是轻量消息。
   */
  void platform_callback(const vision_servo_msgs::msg::PlatformState::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_platform_state_ = *msg;
  }

  /**
   * @brief 伺服模式切换服务回调 — 运行时热切换控制器插件。
   *
   * 运行时替换活跃的 controller_ 实例。与构造函数中的加载逻辑类似，
   * 但额外处理了状态迁移：
   *   1. 创建新控制器实例（pluginlib::ClassLoader::createSharedInstance）
   *   2. 同步控制器参数（configureFromNode）
   *   3. 若相机已标定，重新初始化新控制器（传递缓存的 K 矩阵）
   *   4. 若当前有活跃伺服任务，立即为新控制器设置伺服目标
   *      （从 active_target_ 重新生成，保持跟踪连续性）
   *
   * 切换失败（插件不存在、类名错误等）→ 返回 success=false，
   * 旧控制器实例保持不变，伺服任务不受影响。
   *
   * plugin_map 将 mode ID (0-4) 映射到插件类名：
   *   mode 0 → IBVSController（唯一完整实现）
   *   mode 1 → PBVSController
   *   mode 2-4 → IBVSController（HYBRID/MPC/RL 占位，待实现）
   * 当前仅有 mode 0 和 mode 1 具备独立实现。
   */
  void set_mode_callback(
      const std::shared_ptr<vision_servo_msgs::srv::SetServoMode::Request> req,
      std::shared_ptr<vision_servo_msgs::srv::SetServoMode::Response> resp) {
    // 控制器插件映射表（mode index → 插件全限定类名）
    static const char* plugin_map[] = {
      "servo_control_pkg::IBVSController",  // 0: IBVS — 基于图像的视觉伺服
      "servo_control_pkg::PBVSController",  // 1: PBVS — 基于位置的视觉伺服
      "servo_control_pkg::IBVSController",  // 2: HYBRID（占位 — 待实现）
      "servo_control_pkg::IBVSController",  // 3: MPC（占位 — 待集成）
      "servo_control_pkg::IBVSController",  // 4: RL（占位 — 待训练）
    };
    if (req->mode >= sizeof(plugin_map) / sizeof(plugin_map[0])) {
      resp->success = false;
      resp->message = "未知伺服模式: " + std::to_string(req->mode);
      return;
    }

    try {
      std::lock_guard<std::mutex> lock(state_mutex_);
      // 通过 ClassLoader 创建新控制器实例（旧实例被 shared_ptr 释放）
      controller_ = controller_loader_->createSharedInstance(plugin_map[req->mode]);
      controller_->configureFromNode(*this);

      // 恢复相机标定状态到新控制器
      if (calibrated_) {
        controller_->initialize(camera_fx_, camera_fy_, camera_cx_, camera_cy_,
                                camera_width_, camera_height_);
      }

      // 若当前有活跃伺服任务，立即为新控制器设置目标
      if (servo_active_.load() && last_target_) {
        controller_->setGoalFromTarget(
          active_target_,
          this->get_parameter("desired_depth").as_double(),
          this->get_parameter("feature_tolerance").as_double());
      }

      resp->success = true;
      resp->message = "已切换到 " + std::string(plugin_map[req->mode]);
      RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
    } catch (const std::exception& e) {
      resp->success = false;
      resp->message = e.what();
    }
  }

  /**
   * @brief 动作目标校验 — 接受或拒绝新的 VisualServo 请求。
   *
   * 拒绝条件：
   *   - 已有活跃伺服任务（servo_active_ = true）：同一时刻只允许一个 action
   *   - feature_tolerance < 0：容差为负数无意义
   *
   * 通过校验后立即返回 ACCEPT_AND_EXECUTE，handle_accepted() 将在独立
   * 线程中启动实际执行，不阻塞 action 服务器的接收循环。
   */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const VisualServo::Goal> goal) {
    if (servo_active_.load()) {
      RCLCPP_WARN(get_logger(), "已有 VisualServo 任务在执行，拒绝新目标");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->feature_tolerance < 0.0f) {
      RCLCPP_WARN(get_logger(), "feature_tolerance 不能为负数");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief 取消回调 — 总是接受取消请求。
   *
   * 实际取消处理在 execute_servo() 的轮询循环中：每轮迭代开头检查
   * is_canceling()，若为 true 则调用 finish_action() 并 return。
   * 取消响应延迟 ≤ 100ms（execute_servo 的轮询周期）。
   */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleVisualServo>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief 目标接受回调 — 在独立线程中启动伺服执行。
   *
   * 每次只保留一个 action 线程：若上一线程尚未结束，先 join 等待其退出。
   * 这保证了 action_thread_ 的生命周期安全和 servo_active_ 互斥。
   */
  void handle_accepted(const std::shared_ptr<GoalHandleVisualServo> goal_handle) {
    if (action_thread_.joinable()) {
      action_thread_.join();  // 等待上一线程完全退出
    }
    action_thread_ = std::thread(&ServoManagerNode::execute_servo, this, goal_handle);
  }

  /**
   * @brief 伺服动作执行器 — 长周期任务的主循环。
   *
   * 执行分为两个阶段：
   *
   *   ┌──────────────────────────────────────────────────────────────────┐
   *   │ 阶段 1：等待就绪（阻塞轮询）                                       │
   *   │                                                                  │
   *   │   条件：相机已标定 (calibrated_) ∧ 有目标 (last_target_) ∧        │
   *   │         可在目标列表中匹配到指定 target_id/class_name 的目标       │
   *   │                                                                  │
   *   │   → 就绪：调用 setGoalFromTarget() 设置伺服目标，进入阶段 2       │
   *   │   → 超时：abort（阶段 1 超时也算任务失败）                         │
   *   │   → 取消：cancel（用户主动取消）                                  │
   *   └──────────────────────────────────────────────────────────────────┘
   *                              │
   *                              ▼
   *   ┌──────────────────────────────────────────────────────────────────┐
   *   │ 阶段 2：持续跟随（轮询 control_loop 输出的伺服状态）               │
   *   │                                                                  │
   *   │   每 100ms 检查一次：                                             │
   *   │     - TRACKING → 继续跟随并发布反馈                                │
   *   │     - 目标丢失（target_stale）→ 状态标记为 LOST，继续等待          │
   *   │     - 到达 timeout → 若仍在 TRACKING 则 succeed，否则 abort         │
   *   │     - 取消 → cancel（用户主动取消）                               │
   *   │                                                                  │
   *   │   每轮迭代均发布反馈（current_error, progress, servo_state）。     │
   *   └──────────────────────────────────────────────────────────────────┘
   *
   * @note execute_servo 运行在独立线程中。它不直接调用 computeVelocity()
   * （由 control_loop 定时器线程负责），而是通过 controller_->getServoState()
   * 观测控制回路的输出。这种设计将"控制执行"（50 Hz 实时）与"任务监控"
   * （10 Hz 轮询）分离，互不干扰。
   *
   * @note 超时时间：优先用 action goal 中指定的 timeout 字段，未指定则默认 30s。
   */
  void execute_servo(const std::shared_ptr<GoalHandleVisualServo> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<VisualServo::Result>();
    const rclcpp::Time start_time = this->now();
    const double timeout_sec = goal_timeout_seconds(*goal);

    servo_active_.store(true);  // 抢占标志，防止 auto_start 重复触发

    // ── 阶段 1：等待目标出现和相机标定 ──────────────────────────────
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        finish_action(goal_handle, result, false, "伺服任务已取消", true, start_time);
        return;
      }

      bool configured = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (calibrated_ && last_target_) {
          vision_servo_msgs::msg::Target selected;
          // 按 target_id / class_name 在目标数组中搜索匹配目标
          if (select_target_locked(goal->target_id, goal->class_name, selected)) {
            active_target_ = selected;
            // setGoalFromTarget 内部计算期望 bbox（基于 desired_depth 缩放，
            // 期望中心位于主点），并调用 setDesiredFeatures → 控制回路开始工作
            configured = controller_->setGoalFromTarget(
              active_target_, goal->desired_depth, goal->feature_tolerance);
          }
        }
      }

      if (configured) {
        break;  // 就绪，进入阶段 2
      }

      // 尚未就绪 → 发布 LOST 反馈并等待
      publish_action_feedback(goal_handle, 0.0f, vision_servo_msgs::msg::ServoState::LOST);
      if ((this->now() - start_time).seconds() > timeout_sec) {
        finish_action(goal_handle, result, false, "等待目标或相机标定超时", false, start_time);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10 Hz 轮询
    }

    // ── 阶段 2：持续跟随，直到取消或达到目标持续时长 ─────────────────
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        finish_action(goal_handle, result, false, "伺服任务已取消", true, start_time);
        return;
      }

      // 从控制器获取当前伺服状态（含误差范数、状态机阶段）
      auto state = vision_servo_msgs::msg::ServoState();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = controller_->getServoState();
        // 如果目标数据超时，覆盖状态为 LOST（控制回路也会检测并输出零速度）
        if (!last_target_ || target_stale_locked(this->now())) {
          state.state = vision_servo_msgs::msg::ServoState::LOST;
        }
      }

      publish_action_feedback(goal_handle, state.norm_error, state.state);

      if ((this->now() - start_time).seconds() > timeout_sec) {
        const bool completed_while_tracking =
          state.state == vision_servo_msgs::msg::ServoState::TRACKING;
        finish_action(
          goal_handle, result, completed_while_tracking,
          completed_while_tracking ? "伺服任务达到持续跟随时长" : "伺服任务超时",
          false, start_time);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10 Hz 轮询
    }

    // ROS 已关闭（rclcpp::ok() = false）
    finish_action(goal_handle, result, false, "ROS2 已关闭", false, start_time);
  }

  /**
   * @brief 主控制回路（50 Hz）— 每 20ms 执行一次，是伺服系统的核心。
   *
   * 这是 servo_manager 的最关键方法，负责串联整个感知→控制→执行的流水线。
   *
   * ── 执行流程 ────────────────────────────────────────────────────────
   *
   *   控制回路按以下顺序执行，任何一步失败即提前返回 (early return)：
   *
   *   1. PREFLIGHT 检查
   *      - 相机未标定 → 跳过（等待 camera_info_callback）
   *      - 无目标或目标超时 → 发布零速度 + LOST 状态 → 返回
   *      - servo_active_=false 且 auto_start_=false → 空闲等待 → 返回
   *
   *   2. AUTO-START（如启用）
   *      - auto_start_=true 且 controller_ 未设定目标时，
   *        自动对 active_target_ 调用 setGoalFromTarget()
   *      - 成功后置 servo_active_=true
   *
   *   3. 计算 dt
   *      - dt = 当前时间 - 上一帧时间（首帧默认 20ms）
   *      - dt 用于 ControlAllocator 中的积分/平滑逻辑
   *
   *   4. computeVelocity()  — 控制器计算 6-DOF 相机速度
   *      - 返回 nullopt → 未就绪 → 发布零速度
   *
   *   5. allocate()          — 分配器分解为底盘 + 云台指令
   *      - 输入：相机速度 + 平台状态 + dt
   *      - 输出：Allocation{chassis_twist, gimbal_yaw_rate, gimbal_pitch_rate,
   *                        hold_yaw, hold_pitch}
   *
   *   6. 发布指令
   *      - /cmd_vel (TwistStamped, frame_id=base_link)
   *      - /cmd_gimbal (GimbalCmd with yaw/pitch rates)
   *
   *   7. 发布状态
   *      - /servo/state (ServoState: error, condition number, FSM state)
   *
   *   8. 调试打印（速率受 debug_print_rate 参数控制）
   *
   * ── 线程安全 ────────────────────────────────────────────────────────
   *
   *   整个 control_loop 在 state_mutex_ 保护下运行，确保与 target_callback、
   *   set_mode_callback、execute_servo 互斥。锁粒度覆盖整帧（粗粒度），
   *   避免了逐变量加锁的复杂性。控制回路的计算时间 (<1ms) 远小于帧间隙
   *   (20ms)，粗粒度锁不会造成显著的锁竞争。
   *
   * ── 目标超时处理 ────────────────────────────────────────────────────
   *
   *   若最后收到目标的时间超过 target_timeout_（默认 0.5s），判定为
   *   目标丢失。此时：
   *     - 发布零底盘速度（停止移动）
   *     - 云台保持当前位置（hold_yaw/hold_pitch = true）
   *     - 发布 LOST 伺服状态
   *   短暂的目标丢失（如检测器漏检 1 帧）会被 target_timeout_ 容错窗口
   *   吸收，避免不必要的零速度跳变。
   */
  void control_loop() {
    auto now = this->now();
    std::lock_guard<std::mutex> lock(state_mutex_);

    // ── 前置检查 1：相机标定 ──────────────────────────────────────────
    if (!calibrated_) return;

    // ── 前置检查 2：目标数据可用性 ────────────────────────────────────
    // 目标丢失或超时 → 安全停止：底盘零速，云台保持当前位置
    if (!last_target_ || target_stale_locked(now)) {
      if (servo_active_.load()) {
        publish_zero_command(now);
        publish_lost_state(now);
      }
      return;
    }

    // ── 自动启动逻辑 ──────────────────────────────────────────────────
    // auto_start_=true 时，第一个有效目标到达后自动设定伺服目标并激活控制。
    // auto_start_=false 时，需通过 VisualServo action 显式启动。
    if (!servo_active_.load()) {
      if (!auto_start_) return;  // 非自动模式且无 action → 空闲
      if (!controller_->hasGoal()) {
        const double desired_depth = this->get_parameter("desired_depth").as_double();
        const double tolerance = this->get_parameter("feature_tolerance").as_double();
        if (!controller_->setGoalFromTarget(active_target_, desired_depth, tolerance)) {
          publish_zero_command(now);
          return;  // 目标无效（bbox 过小、深度非法等），跳过本帧
        }
      }
      servo_active_.store(true);
      RCLCPP_INFO(get_logger(),
        "自动启动伺服跟踪 | 目标位置(相机系): x=%.2f y=%.2f z=%.2fm | "
        "期望深度=%.2fm",
        active_target_.position.size() > 0 ? active_target_.position[0] : 0.0,
        active_target_.position.size() > 1 ? active_target_.position[1] : 0.0,
        active_target_.position.size() > 2 ? active_target_.position[2] : 0.0,
        this->get_parameter("desired_depth").as_double());
    }

    // ── 计算时间步长 dt ──────────────────────────────────────────────
    // 首帧（last_control_time_ 未初始化）：默认 20ms
    // 后续帧：实际时间差（用于分配器中的积分和平滑逻辑）
    double dt = (last_control_time_.nanoseconds() > 0)
      ? (now - last_control_time_).seconds() : 0.02;
    last_control_time_ = now;

    // ═══ 步骤 1：计算相机期望速度 ═════════════════════════════════════
    // controller_->computeVelocity() 是策略模式的核心调用点。
    // 不同控制器（IBVS/PBVS）在此返回语义相同（6-DOF 相机速度），
    // 但内部算法完全不同（图像空间 vs 笛卡尔空间）。
    auto cam_vel = controller_->computeVelocity(active_target_, dt);
    if (!cam_vel) {
      // nullopt 含义：控制器未初始化、未设目标、或已收敛 → 零速度
      publish_zero_command(now);
      return;
    }

    // ═══ 步骤 2：控制分配（相机速度 → 底盘 + 云台指令）═══════════════
    // ControlAllocator 将单一的相机速度分解为两个执行器的协同指令。
    // 分配策略基于 allocation_ratio 参数和云台限位状态。
    auto allocation = allocator_.allocate(*cam_vel, last_platform_state_, dt);

    // ═══ 步骤 3：发布底盘速度指令 ═════════════════════════════════════
    auto twist_stamped = geometry_msgs::msg::TwistStamped();
    twist_stamped.header.stamp = now;
    twist_stamped.header.frame_id = "base_link";
    twist_stamped.twist = allocation.chassis_twist;
    chassis_cmd_pub_->publish(twist_stamped);
    publish_unstamped_chassis_command(allocation.chassis_twist);

    // ═══ 步骤 4：发布云台指令 ═════════════════════════════════════════
    vision_servo_msgs::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header.stamp = now;
    gimbal_cmd.yaw_rate = allocation.gimbal_yaw_rate;      // 偏航角速率 (rad/s)
    gimbal_cmd.pitch_rate = allocation.gimbal_pitch_rate;  // 俯仰角速率 (rad/s)
    gimbal_cmd.hold_yaw = allocation.hold_yaw;              // 保持偏航角（目标丢失时）
    gimbal_cmd.hold_pitch = allocation.hold_pitch;          // 保持俯仰角（目标丢失时）
    gimbal_cmd_pub_->publish(gimbal_cmd);

    // ═══ 步骤 5：发布伺服状态（监控 + 调试）═══════════════════════════
    publish_control_state(now, *cam_vel, allocation);

    // ═══ 步骤 6：周期性调试打印 ═══════════════════════════════════════
    // 打印速率由 debug_print_rate 参数控制（Hz），0 表示关闭。
    // 打印内容：目标位置（相机系）、相机速度向量、底盘指令、特征误差。
    {
      const double rate = this->get_parameter("debug_print_rate").as_double();
      if (rate > 0.0) {
        if (last_debug_print_time_.nanoseconds() == 0) {
          last_debug_print_time_ = now;  // 首帧跳过打印，仅初始化时间戳
        } else if ((now - last_debug_print_time_).seconds() >= 1.0 / rate) {
          last_debug_print_time_ = now;
          auto state = controller_->getServoState();
          double target_x = active_target_.position.size() > 0
            ? active_target_.position[0] : 0.0;
          double target_y = active_target_.position.size() > 1
            ? active_target_.position[1] : 0.0;
          double target_z = active_target_.position.size() > 2
            ? active_target_.position[2] : 0.0;
          RCLCPP_INFO(get_logger(),
            "跟踪中 | 目标(相机系): x=%.2f y=%.2f z=%.2fm | "
            "cam_vel: v=[%.3f %.3f %.3f] w=[%.3f %.3f %.3f] | "
            "cmd: vx=%.3f wz=%.3f | err=%.4f",
            target_x, target_y, target_z,
            (*cam_vel)(0), (*cam_vel)(1), (*cam_vel)(2),  // vx, vy, vz
            (*cam_vel)(3), (*cam_vel)(4), (*cam_vel)(5),  // ωx, ωy, ωz
            allocation.chassis_twist.linear.x,
            allocation.chassis_twist.angular.z,
            state.norm_error);
        }
      }
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // 辅助方法
  // ═══════════════════════════════════════════════════════════════════════════

  /**
   * @brief 从 action goal 的 timeout 字段提取超时秒数。
   *
   * timeout 为 builtin_interfaces/Duration 类型（sec + nanosec），
   * 若两者均为 0（未设置），返回默认值 30s。
   */
  double goal_timeout_seconds(const VisualServo::Goal& goal) const {
    const double timeout =
      static_cast<double>(goal.timeout.sec) +
      static_cast<double>(goal.timeout.nanosec) * 1e-9;
    return timeout > 0.0 ? timeout : 30.0;
  }

  /**
   * @brief 判断目标数据是否已过期（stale）。
   *
   * 判定条件：最后收到目标的时间距今超过 target_timeout_ 秒。
   * 在锁内调用，调用者需持有 state_mutex_。
   */
  bool target_stale_locked(const rclcpp::Time& now) const {
    if (last_target_time_.nanoseconds() == 0) return true;  // 从未收到过目标
    return (now - last_target_time_).seconds() > target_timeout_;
  }

  /**
   * @brief 在目标数组中搜索匹配的目标（在锁内调用）。
   *
   * 匹配规则：
   *   - target_id < 0：匹配所有 ID（不按 ID 过滤）
   *   - class_name 为空：匹配所有类别
   *   两个条件取 AND：同时满足 ID 和类别才算命中
   *
   * @param target_id    期望的目标 ID（负数 = 任意）
   * @param class_name   期望的目标类别（空串 = 任意）
   * @param selected     输出参数：匹配到的目标
   * @return true 找到匹配目标，false 未找到
   */
  bool select_target_locked(
      int32_t target_id,
      const std::string& class_name,
      vision_servo_msgs::msg::Target& selected) const {
    if (!last_target_) return false;

    for (const auto& target : last_target_->targets) {
      const bool id_match = target_id < 0 || target.id == target_id;
      const bool class_match = class_name.empty() || target.class_name == class_name;
      if (id_match && class_match) {
        selected = target;
        return true;
      }
    }
    return false;
  }

  /**
   * @brief 发布零速度指令（目标丢失、未标定或任务终止时调用）。
   *
   * 底盘：发布线速度/角速度均为 0 的 TwistStamped。
   * 云台：设置 hold_yaw/hold_pitch = true，指示云台保持当前位置。
   */
  void publish_zero_command(const rclcpp::Time& stamp) {
    geometry_msgs::msg::TwistStamped twist_stamped;
    twist_stamped.header.stamp = stamp;
    twist_stamped.header.frame_id = "base_link";
    chassis_cmd_pub_->publish(twist_stamped);  // 默认全部字段为 0
    publish_unstamped_chassis_command(twist_stamped.twist);

    vision_servo_msgs::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header.stamp = stamp;
    gimbal_cmd.hold_yaw = true;    // 保持偏航角当前位置
    gimbal_cmd.hold_pitch = true;  // 保持俯仰角当前位置
    gimbal_cmd_pub_->publish(gimbal_cmd);
  }

  /// 发布无时间戳的 Twist（供 Gazebo 等仿真插件使用）
  void publish_unstamped_chassis_command(const geometry_msgs::msg::Twist& twist) {
    if (chassis_cmd_unstamped_pub_) {
      chassis_cmd_unstamped_pub_->publish(twist);
    }
  }

  /**
   * @brief 发布 LOST 伺服状态。
   *
   * 在目标丢失时调用。保留控制器内部的误差和条件数，仅覆盖 state = LOST，
   * 便于监控端区分"控制器正在收敛（但误差大）"和"目标丢失无数据"。
   */
  void publish_lost_state(const rclcpp::Time& stamp) {
    auto servo_state = controller_->getServoState();
    servo_state.header.stamp = stamp;
    servo_state.last_update = time_to_msg(stamp);
    servo_state.state = vision_servo_msgs::msg::ServoState::LOST;
    servo_state_pub_->publish(servo_state);
  }

  /**
   * @brief 发布完整的控制状态快照。
   *
   * 在 controller_->getServoState() 基础上补充：
   *   - camera_velocity：本次控制律输出的 6-DOF 相机速度（覆盖控制器的缓存值）
   *   - gimbal_velocity[2]：分配后的云台 yaw/pitch 速率
   *   - chassis_velocity[3]：分配后的底盘 vx/vy/wz
   */
  void publish_control_state(
      const rclcpp::Time& stamp,
      const Eigen::Matrix<double, 6, 1>& cam_vel,
      const ControlAllocation& allocation) {
    auto servo_state = controller_->getServoState();
    servo_state.header.stamp = stamp;
    servo_state.last_update = time_to_msg(stamp);
    // 覆盖相机速度（以实际输出为准，而非控制器缓存）
    for (size_t i = 0; i < 6; ++i) {
      servo_state.camera_velocity[i] = static_cast<float>(cam_vel(i));
    }
    servo_state.gimbal_velocity[0] = static_cast<float>(allocation.gimbal_yaw_rate);
    servo_state.gimbal_velocity[1] = static_cast<float>(allocation.gimbal_pitch_rate);
    servo_state.chassis_velocity[0] = static_cast<float>(allocation.chassis_twist.linear.x);
    servo_state.chassis_velocity[1] = static_cast<float>(allocation.chassis_twist.linear.y);
    servo_state.chassis_velocity[2] = static_cast<float>(allocation.chassis_twist.angular.z);
    servo_state_pub_->publish(servo_state);
  }

  /// 将 rclcpp::Time 转换为 builtin_interfaces/Time（ROS2 时间消息）
  builtin_interfaces::msg::Time time_to_msg(const rclcpp::Time& stamp) const {
    builtin_interfaces::msg::Time msg;
    const int64_t ns = stamp.nanoseconds();
    msg.sec = static_cast<int32_t>(ns / 1'000'000'000LL);
    msg.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
    return msg;
  }

  /**
   * @brief 发布 VisualServo 动作的周期性反馈。
   *
   * 反馈包含：
   *   - current_error：当前特征误差范数
   *   - servo_state：控制器状态机阶段（LOST / CONVERGING / TRACKING）
   *   - progress：归一化收敛进度 [0, 1]
   *       - progress = 1.0 当 current_error ≤ tolerance
   *       - progress = tolerance / current_error 当 current_error > tolerance
   *       - 不直接用 linear error scaling，因为视觉伺服误差收敛是非线性的
   *   - camera_velocity：当前 6-DOF 相机速度（来自控制器缓存）
   */
  void publish_action_feedback(
      const std::shared_ptr<GoalHandleVisualServo>& goal_handle,
      float current_error,
      uint8_t state) {
    auto feedback = std::make_shared<VisualServo::Feedback>();
    const double tolerance = this->get_parameter("feature_tolerance").as_double();
    feedback->current_error = current_error;
    feedback->servo_state = state;
    // 进度计算：用 tolerance / max(error, tolerance) 做归一化
    //   error=2*tolerance  → progress=0.5
    //   error=tolerance    → progress=1.0 (已达标)
    //   error=0            → progress=1.0
    feedback->progress = current_error <= tolerance
      ? 1.0f
      : static_cast<float>(std::clamp(tolerance / std::max<double>(current_error, tolerance),
                                      0.0, 1.0));
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto servo_state = controller_->getServoState();
      feedback->camera_velocity = servo_state.camera_velocity;
    }
    goal_handle->publish_feedback(feedback);
  }

  /**
   * @brief 终止 VisualServo 动作并发布最终结果。
   *
   * 统一处理三种终止路径：
   *   - canceled=true  → goal_handle->canceled(result)
   *   - success=true   → goal_handle->succeed(result)
   *   - 其他           → goal_handle->abort(result)（超时、目标丢失等）
   *
   * 副作用：置 servo_active_=false，发布零速度指令，安全停车。
   */
  void finish_action(
      const std::shared_ptr<GoalHandleVisualServo>& goal_handle,
      const std::shared_ptr<VisualServo::Result>& result,
      bool success,
      const std::string& message,
      bool canceled,
      const rclcpp::Time& start_time) {
    auto now = this->now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      result->final_error = controller_ ? controller_->getCurrentErrorNorm() : 0.0f;
    }
    result->success = success;
    result->message = message;
    result->elapsed_time = static_cast<float>((now - start_time).seconds());
    servo_active_.store(false);
    publish_zero_command(now);  // 安全停车：底盘零速，云台保持
    if (canceled) {
      goal_handle->canceled(result);
    } else if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // 成员变量
  // ═══════════════════════════════════════════════════════════════════════════

  // ── 控制器与分配器 ─────────────────────────────────────────────────
  //
  // controller_loader_: pluginlib 类加载器，负责从共享库 (.so) 中实例化控制器。
  //   同一 loader 可多次调用 createSharedInstance() 切换不同控制器。
  // controller_: 当前活跃的控制器实例（shared_ptr 管理生命周期，
  //   切换时旧实例自动释放）。
  // allocator_: 控制分配器（值类型），将 6-DOF 相机速度分解为底盘+云台指令。
  std::unique_ptr<pluginlib::ClassLoader<ServoControllerBase>> controller_loader_;
  std::shared_ptr<ServoControllerBase> controller_;
  ControlAllocator allocator_;

  // ── 核心状态标志 ──────────────────────────────────────────────────
  //
  // calibrated_: 相机标定是否完成。由 camera_info_callback 置 true，
  //   之后所有帧不再检查 CameraInfo。
  // servo_active_: 是否有活跃的伺服任务。atomic_bool 保证
  //   execute_servo 线程的写入对 control_loop 线程的读取可见，无需加锁。
  // auto_start_: 收到第一个目标后自动设置伺服目标并开始控制。
  //   用于仿真和一键启动场景，与 VisualServo action 互斥（二选一）。
  bool calibrated_ = false;
  std::atomic_bool servo_active_{false};
  bool auto_start_ = false;

  // ── 调度参数 ────────────────────────────────────────────────────────
  double target_timeout_ = 0.5;           ///< 目标丢失判定超时 (s)
  bool publish_unstamped_cmd_vel_ = false; ///< 仿真兼容：额外发布无时间戳 Twist

  // ── 线程同步 ────────────────────────────────────────────────────────
  //
  // state_mutex_ 保护所有共享状态。被以下线程竞争：
  //   1. control_loop（定时器线程, 50 Hz）
  //   2. target_callback（订阅者线程, 变频率 ~10-30 Hz）
  //   3. platform_callback（订阅者线程, ~50 Hz）
  //   4. camera_info_callback（TRANSIENT_LOCAL, 一次性）
  //   5. set_mode_callback（服务线程, 低频）
  //   6. execute_servo（action 线程, 10 Hz 轮询）
  //
  // 当前采用粗粒度锁（整帧持锁），控制回路计算时间 (<1ms) ≪ 帧间隙 (20ms)，
  // 锁竞争概率低。若未来需降低延迟，可拆分为细粒度锁或使用 lock-free 结构。
  std::mutex state_mutex_;

  // ── 订阅者（按对应话题排序）─────────────────────────────────────────
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr target_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::PlatformState>::SharedPtr platform_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // ── 发布者 ─────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_unstamped_pub_;  ///< 可空
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::ServoState>::SharedPtr servo_state_pub_;

  // ── 服务与动作 ──────────────────────────────────────────────────────
  rclcpp::Service<vision_servo_msgs::srv::SetServoMode>::SharedPtr servo_mode_srv_;
  rclcpp_action::Server<vision_servo_msgs::action::VisualServo>::SharedPtr servo_action_;
  std::thread action_thread_;  ///< 执行 VisualServo action 的独立线程

  // ── 定时器 ──────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr control_timer_;  ///< 50 Hz wall timer

  // ── 数据缓存（均受 state_mutex_ 保护，除非特别说明）───────────────
  //
  // last_target_: 最近一次收到的目标数组（shared_ptr 避免拷贝整个数组）。
  // active_target_: 当前伺服跟踪的目标（由 target_callback 从数组中选出）。
  // last_platform_state_: 值拷贝（PlatformState 是轻量消息）。
  vision_servo_msgs::msg::TargetArray::ConstSharedPtr last_target_;
  vision_servo_msgs::msg::Target active_target_;
  vision_servo_msgs::msg::PlatformState last_platform_state_;

  // ── 时间戳缓存 ──────────────────────────────────────────────────────
  rclcpp::Time last_control_time_;      ///< 上一帧控制时间（用于计算 dt）
  rclcpp::Time last_target_time_;       ///< 最后一次收到目标的时间（用于超时判断）
  rclcpp::Time last_debug_print_time_;  ///< 上次调试打印时间（用于 debug_print_rate 限速）

  // ── 相机内参缓存 ────────────────────────────────────────────────────
  //
  // 缓存 CameraInfo 的 K 矩阵参数，用于在控制器热切换时重新初始化。
  // 对于大多数控制器（IBVS/PBVS），这四个参数 + 分辨率足够。
  double camera_fx_ = 0.0, camera_fy_ = 0.0, camera_cx_ = 0.0, camera_cy_ = 0.0;
  int camera_width_ = 0, camera_height_ = 0;
};

}  // namespace servo_control_pkg

// ═══════════════════════════════════════════════════════════════════════════════
// 入口
// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // spin 阻塞主线程直到 SIGINT/SIGTERM 或节点退出。
  // 所有伺服逻辑（控制回路、订阅回调、服务/动作）均在 spin 内部的事件循环中执行。
  rclcpp::spin(std::make_shared<servo_control_pkg::ServoManagerNode>());
  rclcpp::shutdown();
  return 0;
}
