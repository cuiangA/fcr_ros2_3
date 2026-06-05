/**
 * @file servo_manager_node.cpp
 * @brief 伺服管理节点 — 主控制回路：加载控制器插件 → 运行控制 → 分配 → 发布。
 *
 * 架构说明：
 *   该节点是伺服控制系统的核心调度器。它通过 pluginlib 动态加载
 *   具体的控制器实现（IBVS/PBVS/MPC/RL），以 50 Hz 频率运行控制回路。
 *
 * 每帧控制回路（control_loop）：
 *   1. 读取当前跟踪目标（来自感知管线的 3D 目标位姿）
 *   2. 调用控制器的 computeVelocity() 计算相机期望速度
 *   3. 调用 ControlAllocator::allocate() 分解为底盘 + 云台指令
 *   4. 发布 /cmd_vel（底盘）和 /cmd_gimbal（云台）话题
 *   5. 发布 /servo/state（伺服状态监控）
 *
 * 特性：
 *   - 运行时热切换控制器：通过 SetServoMode 服务动态更换插件
 *   - 动作服务器：VisualServo 动作支持长周期伺服任务（可取消、可反馈）
 *   - 自动选择跟踪目标：优先使用 tracking_id，否则选择置信度最高的目标
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
  using GoalHandleVisualServo = rclcpp_action::ServerGoalHandle<VisualServo>;

  explicit ServoManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("servo_manager", options)
  {
    // ── 1. 声明参数并加载控制器插件 ───────────────────────────────
    this->declare_parameter("controller_plugin", "servo_control_pkg::IBVSController");
    this->declare_parameter("allocation_ratio", 0.5);
    this->declare_parameter("gimbal_yaw_limit", M_PI);
    this->declare_parameter("gimbal_pitch_limit", M_PI_2);
    this->declare_parameter("chassis_linear_limit", 1.0);
    this->declare_parameter("chassis_angular_limit", 2.0);
    this->declare_parameter("auto_start", false);
    this->declare_parameter("target_timeout", 0.5);
    this->declare_parameter("publish_unstamped_cmd_vel", false);

    // 控制器公共参数。插件由 servo_manager 下发参数，避免 pluginlib Node
    // 名称与 YAML 节点名不一致导致调参失效。
    this->declare_parameter("lambda_gain", 0.5);
    this->declare_parameter("max_linear_velocity", 1.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("feature_tolerance", 0.01);
    this->declare_parameter("desired_depth", 2.0);
    this->declare_parameter("gain_max", 1.0);
    this->declare_parameter("gain_min", 0.1);
    this->declare_parameter("error_threshold_slow", 0.05);
    this->declare_parameter("use_adaptive_gain", true);
    this->declare_parameter("translational_gain", 0.5);
    this->declare_parameter("rotational_gain", 0.3);

    std::string plugin_name = this->get_parameter("controller_plugin").as_string();
    auto_start_ = this->get_parameter("auto_start").as_bool();
    target_timeout_ = this->get_parameter("target_timeout").as_double();
    publish_unstamped_cmd_vel_ =
      this->get_parameter("publish_unstamped_cmd_vel").as_bool();

    // 使用 pluginlib::ClassLoader 动态加载控制器共享库
    try {
      controller_loader_ = std::make_unique<pluginlib::ClassLoader<ServoControllerBase>>(
        "servo_control_pkg", "servo_control_pkg::ServoControllerBase");
      controller_ = controller_loader_->createSharedInstance(plugin_name);
      controller_->configureFromNode(*this);
      RCLCPP_INFO(get_logger(), "已加载控制器插件: %s", plugin_name.c_str());
    } catch (const pluginlib::PluginlibException& e) {
      RCLCPP_ERROR(get_logger(), "控制器插件加载失败: %s", e.what());
      throw;  // 控制器加载失败是致命错误，终止节点
    }

    // ── 2. 配置控制分配器 ─────────────────────────────────────────
    allocator_.configure(
      this->get_parameter("gimbal_yaw_limit").as_double(),
      this->get_parameter("gimbal_pitch_limit").as_double(),
      this->get_parameter("chassis_linear_limit").as_double(),
      this->get_parameter("chassis_angular_limit").as_double(),
      this->get_parameter("allocation_ratio").as_double()
    );

    // ── 3. 订阅者 ─────────────────────────────────────────────────
    // 3D 目标位姿（来自感知管线）
    target_sub_ = this->create_subscription<vision_servo_msgs::msg::TargetArray>(
      "/perception/targets_3d", qos::control_cmd(),
      std::bind(&ServoManagerNode::target_callback, this, std::placeholders::_1));

    // 平台状态（来自平台管理器）
    platform_sub_ = this->create_subscription<vision_servo_msgs::msg::PlatformState>(
      "/platform/state", qos::platform_state(),
      std::bind(&ServoManagerNode::platform_callback, this, std::placeholders::_1));

    // 相机内参（TRANSIENT_LOCAL：一次发布，所有迟加入节点均可获取）
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ServoManagerNode::camera_info_callback, this, std::placeholders::_1));

    // ── 4. 发布者 ─────────────────────────────────────────────────
    chassis_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", qos::control_cmd());
    if (publish_unstamped_cmd_vel_) {
      chassis_cmd_unstamped_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel_unstamped", qos::control_cmd());
    }

    gimbal_cmd_pub_ = this->create_publisher<vision_servo_msgs::msg::GimbalCmd>(
      "/cmd_gimbal", qos::control_cmd());

    servo_state_pub_ = this->create_publisher<vision_servo_msgs::msg::ServoState>(
      "/servo/state", qos::servo_state());

    // ── 5. 服务 ───────────────────────────────────────────────────
    servo_mode_srv_ = this->create_service<vision_servo_msgs::srv::SetServoMode>(
      "/servo/set_mode",
      std::bind(&ServoManagerNode::set_mode_callback, this, std::placeholders::_1, std::placeholders::_2));

    // ── 6. 动作服务器 ─────────────────────────────────────────────
    servo_action_ = rclcpp_action::create_server<vision_servo_msgs::action::VisualServo>(
      this, "/servo/visual_servo",
      std::bind(&ServoManagerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ServoManagerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ServoManagerNode::handle_accepted, this, std::placeholders::_1));

    // ── 7. 控制回路定时器（50 Hz = 20ms 周期） ────────────────────
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ServoManagerNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "伺服管理器已启动 (controller=%s, 50Hz loop)",
                plugin_name.c_str());
  }

  ~ServoManagerNode() override {
    servo_active_.store(false);
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
  }

private:
  /// 相机内参回调 — 收到标定信息后初始化控制器
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!calibrated_) {
      // 提取针孔模型内参矩阵 K 的参数
      // K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
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

  /// 目标回调 — 缓存最新目标，优先使用跟踪 ID 匹配的目标
  void target_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg) {
    if (!msg->targets.empty()) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_target_ = msg;
      last_target_time_ = this->now();
      // 优先使用跟踪器指定的目标，否则使用置信度最高的检测
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
          active_target_ = msg->targets[0];
        }
      } else {
        active_target_ = msg->targets[0];  // 取第一个（通常已按置信度排序）
      }
    }
  }

  /// 平台状态回调 — 缓存最新平台状态（用于控制分配）
  void platform_callback(const vision_servo_msgs::msg::PlatformState::ConstSharedPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_platform_state_ = *msg;
  }

  /**
   * @brief 伺服模式切换服务回调 — 运行时热切换控制器插件。
   *
   * 当前 mode 0-4 均映射到 IBVS（MPC/RL 为占位）。
   * 完整实现中应通过 plugin_map 将 mode 映射到不同的插件类名。
   */
  void set_mode_callback(
      const std::shared_ptr<vision_servo_msgs::srv::SetServoMode::Request> req,
      std::shared_ptr<vision_servo_msgs::srv::SetServoMode::Response> resp) {
    // 控制器插件映射表（index → 插件类名）
    static const char* plugin_map[] = {
      "servo_control_pkg::IBVSController",  // 0: IBVS
      "servo_control_pkg::PBVSController",  // 1: PBVS
      "servo_control_pkg::IBVSController",  // 2: HYBRID（占位）
      "servo_control_pkg::IBVSController",  // 3: MPC（占位）
      "servo_control_pkg::IBVSController",  // 4: RL（占位）
    };
    if (req->mode >= 5) {
      resp->success = false;
      resp->message = "未知伺服模式: " + std::to_string(req->mode);
      return;
    }

    try {
      std::lock_guard<std::mutex> lock(state_mutex_);
      controller_ = controller_loader_->createSharedInstance(plugin_map[req->mode]);
      controller_->configureFromNode(*this);
      if (calibrated_) {
        controller_->initialize(camera_fx_, camera_fy_, camera_cx_, camera_cy_,
                                camera_width_, camera_height_);
      }
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

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleVisualServo>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleVisualServo> goal_handle) {
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
    action_thread_ = std::thread(&ServoManagerNode::execute_servo, this, goal_handle);
  }

  /// 伺服动作执行器（长周期任务，支持取消和反馈）
  void execute_servo(const std::shared_ptr<GoalHandleVisualServo> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<VisualServo::Result>();
    const rclcpp::Time start_time = this->now();
    const double timeout_sec = goal_timeout_seconds(*goal);

    servo_active_.store(true);

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
          if (select_target_locked(goal->target_id, goal->class_name, selected)) {
            active_target_ = selected;
            configured = controller_->setGoalFromTarget(
              active_target_, goal->desired_depth, goal->feature_tolerance);
          }
        }
      }

      if (configured) {
        break;
      }

      publish_action_feedback(goal_handle, 0.0f, vision_servo_msgs::msg::ServoState::LOST);
      if ((this->now() - start_time).seconds() > timeout_sec) {
        finish_action(goal_handle, result, false, "等待目标或相机标定超时", false, start_time);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        finish_action(goal_handle, result, false, "伺服任务已取消", true, start_time);
        return;
      }

      auto state = vision_servo_msgs::msg::ServoState();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = controller_->getServoState();
        if (!last_target_ || target_stale_locked(this->now())) {
          state.state = vision_servo_msgs::msg::ServoState::LOST;
        }
      }

      publish_action_feedback(goal_handle, state.norm_error, state.state);
      if (state.state == vision_servo_msgs::msg::ServoState::TRACKING) {
        result->success = true;
        result->message = "伺服误差已收敛";
        result->final_error = state.norm_error;
        result->elapsed_time = static_cast<float>((this->now() - start_time).seconds());
        servo_active_.store(false);
        publish_zero_command(this->now());
        goal_handle->succeed(result);
        return;
      }

      if ((this->now() - start_time).seconds() > timeout_sec) {
        finish_action(goal_handle, result, false, "伺服任务超时", false, start_time);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    finish_action(goal_handle, result, false, "ROS2 已关闭", false, start_time);
  }

  /**
   * @brief 主控制回路（50 Hz）— 每帧执行。
   *
   * 流程：目标 → computeVelocity() → allocate() → 发布指令
   */
  void control_loop() {
    auto now = this->now();
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!calibrated_) return;  // 未标定 → 跳过
    if (!last_target_ || target_stale_locked(now)) {
      if (servo_active_.load()) {
        publish_zero_command(now);
        publish_lost_state(now);
      }
      return;
    }

    if (!servo_active_.load()) {
      if (!auto_start_) return;
      if (!controller_->hasGoal()) {
        const double desired_depth = this->get_parameter("desired_depth").as_double();
        const double tolerance = this->get_parameter("feature_tolerance").as_double();
        if (!controller_->setGoalFromTarget(active_target_, desired_depth, tolerance)) {
          publish_zero_command(now);
          return;
        }
      }
      servo_active_.store(true);
    }

    double dt = (last_control_time_.nanoseconds() > 0)
      ? (now - last_control_time_).seconds() : 0.02;  // 默认 20ms
    last_control_time_ = now;

    // 步骤 1：计算相机期望速度
    auto cam_vel = controller_->computeVelocity(active_target_, dt);
    if (!cam_vel) {
      publish_zero_command(now);
      return;  // 未初始化或未设置目标
    }

    // 步骤 2：控制分配（相机速度 → 底盘 + 云台）
    auto allocation = allocator_.allocate(*cam_vel, last_platform_state_, dt);

    // 步骤 3：发布底盘速度指令
    auto twist_stamped = geometry_msgs::msg::TwistStamped();
    twist_stamped.header.stamp = now;
    twist_stamped.header.frame_id = "base_link";
    twist_stamped.twist = allocation.chassis_twist;
    chassis_cmd_pub_->publish(twist_stamped);
    publish_unstamped_chassis_command(allocation.chassis_twist);

    // 步骤 4：发布云台指令
    vision_servo_msgs::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header.stamp = now;
    gimbal_cmd.yaw_rate = allocation.gimbal_yaw_rate;
    gimbal_cmd.pitch_rate = allocation.gimbal_pitch_rate;
    gimbal_cmd.hold_yaw = allocation.hold_yaw;
    gimbal_cmd.hold_pitch = allocation.hold_pitch;
    gimbal_cmd_pub_->publish(gimbal_cmd);

    // 步骤 5：发布伺服状态（用于监控和调试）
    publish_control_state(now, *cam_vel, allocation);
  }

  double goal_timeout_seconds(const VisualServo::Goal& goal) const {
    const double timeout =
      static_cast<double>(goal.timeout.sec) +
      static_cast<double>(goal.timeout.nanosec) * 1e-9;
    return timeout > 0.0 ? timeout : 30.0;
  }

  bool target_stale_locked(const rclcpp::Time& now) const {
    if (last_target_time_.nanoseconds() == 0) return true;
    return (now - last_target_time_).seconds() > target_timeout_;
  }

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

  void publish_zero_command(const rclcpp::Time& stamp) {
    geometry_msgs::msg::TwistStamped twist_stamped;
    twist_stamped.header.stamp = stamp;
    twist_stamped.header.frame_id = "base_link";
    chassis_cmd_pub_->publish(twist_stamped);
    publish_unstamped_chassis_command(twist_stamped.twist);

    vision_servo_msgs::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header.stamp = stamp;
    gimbal_cmd.hold_yaw = true;
    gimbal_cmd.hold_pitch = true;
    gimbal_cmd_pub_->publish(gimbal_cmd);
  }

  void publish_unstamped_chassis_command(const geometry_msgs::msg::Twist& twist) {
    if (chassis_cmd_unstamped_pub_) {
      chassis_cmd_unstamped_pub_->publish(twist);
    }
  }

  void publish_lost_state(const rclcpp::Time& stamp) {
    auto servo_state = controller_->getServoState();
    servo_state.header.stamp = stamp;
    servo_state.last_update = time_to_msg(stamp);
    servo_state.state = vision_servo_msgs::msg::ServoState::LOST;
    servo_state_pub_->publish(servo_state);
  }

  void publish_control_state(
      const rclcpp::Time& stamp,
      const Eigen::Matrix<double, 6, 1>& cam_vel,
      const ControlAllocation& allocation) {
    auto servo_state = controller_->getServoState();
    servo_state.header.stamp = stamp;
    servo_state.last_update = time_to_msg(stamp);
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

  builtin_interfaces::msg::Time time_to_msg(const rclcpp::Time& stamp) const {
    builtin_interfaces::msg::Time msg;
    const int64_t ns = stamp.nanoseconds();
    msg.sec = static_cast<int32_t>(ns / 1000000000LL);
    msg.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return msg;
  }

  void publish_action_feedback(
      const std::shared_ptr<GoalHandleVisualServo>& goal_handle,
      float current_error,
      uint8_t state) {
    auto feedback = std::make_shared<VisualServo::Feedback>();
    const double tolerance = this->get_parameter("feature_tolerance").as_double();
    feedback->current_error = current_error;
    feedback->servo_state = state;
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
    publish_zero_command(now);
    if (canceled) {
      goal_handle->canceled(result);
    } else if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  // ── 控制器 ──────────────────────────────────────────────────────
  std::unique_ptr<pluginlib::ClassLoader<ServoControllerBase>> controller_loader_;
  std::shared_ptr<ServoControllerBase> controller_;  ///< 当前活跃的控制器实例
  ControlAllocator allocator_;                        ///< 控制分配器
  bool calibrated_ = false;                           ///< 相机标定是否完成
  std::atomic_bool servo_active_{false};              ///< 伺服动作是否活跃
  bool auto_start_ = false;                           ///< 有目标时自动启动控制
  double target_timeout_ = 0.5;                       ///< 目标消息超时时间 (s)
  bool publish_unstamped_cmd_vel_ = false;            ///< 仿真插件使用的 Twist 输出开关
  std::mutex state_mutex_;                            ///< 保护目标、平台和控制器状态

  // ── 订阅者 ──────────────────────────────────────────────────────
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr target_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::PlatformState>::SharedPtr platform_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // ── 发布者 ──────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_unstamped_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::ServoState>::SharedPtr servo_state_pub_;

  // ── 服务与动作 ──────────────────────────────────────────────────
  rclcpp::Service<vision_servo_msgs::srv::SetServoMode>::SharedPtr servo_mode_srv_;
  rclcpp_action::Server<vision_servo_msgs::action::VisualServo>::SharedPtr servo_action_;
  std::thread action_thread_;

  // ── 定时器 ──────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr control_timer_;  ///< 50 Hz 控制回路定时器

  // ── 状态缓存 ────────────────────────────────────────────────────
  vision_servo_msgs::msg::TargetArray::ConstSharedPtr last_target_;
  vision_servo_msgs::msg::Target active_target_;           ///< 当前伺服跟踪的目标
  vision_servo_msgs::msg::PlatformState last_platform_state_;
  rclcpp::Time last_control_time_;
  rclcpp::Time last_target_time_;

  double camera_fx_ = 0.0, camera_fy_ = 0.0, camera_cx_ = 0.0, camera_cy_ = 0.0;
  int camera_width_ = 0, camera_height_ = 0;
};

}  // namespace servo_control_pkg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<servo_control_pkg::ServoManagerNode>());
  rclcpp::shutdown();
  return 0;
}
