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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <string>

namespace servo_control_pkg {

class ServoManagerNode : public rclcpp::Node {
public:
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

    std::string plugin_name = this->get_parameter("controller_plugin").as_string();

    // 使用 pluginlib::ClassLoader 动态加载控制器共享库
    try {
      controller_loader_ = std::make_unique<pluginlib::ClassLoader<ServoControllerBase>>(
        "servo_control_pkg", "servo_control_pkg::ServoControllerBase");
      controller_ = controller_loader_->createSharedInstance(plugin_name);
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
      [this](const auto&, auto goal) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
      std::bind(&ServoManagerNode::execute_servo, this, std::placeholders::_1));

    // ── 7. 控制回路定时器（50 Hz = 20ms 周期） ────────────────────
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ServoManagerNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "伺服管理器已启动 (controller=%s, 50Hz loop)",
                plugin_name.c_str());
  }

private:
  /// 相机内参回调 — 收到标定信息后初始化控制器
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
    if (!calibrated_) {
      // 提取针孔模型内参矩阵 K 的参数
      // K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
      controller_->initialize(msg->k[0], msg->k[4], msg->k[2], msg->k[5],
                              msg->width, msg->height);
      calibrated_ = true;
    }
  }

  /// 目标回调 — 缓存最新目标，优先使用跟踪 ID 匹配的目标
  void target_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg) {
    if (!msg->targets.empty()) {
      last_target_ = msg;
      // 优先使用跟踪器指定的目标，否则使用置信度最高的检测
      if (msg->tracking_id >= 0) {
        for (auto& t : msg->targets) {
          if (t.id == msg->tracking_id) { active_target_ = t; break; }
        }
      } else {
        active_target_ = msg->targets[0];  // 取第一个（通常已按置信度排序）
      }
    }
  }

  /// 平台状态回调 — 缓存最新平台状态（用于控制分配）
  void platform_callback(const vision_servo_msgs::msg::PlatformState::ConstSharedPtr& msg) {
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
    if (req->mode < 5) {
      try {
        controller_ = controller_loader_->createSharedInstance(plugin_map[req->mode]);
        resp->success = true;
        resp->message = "已切换到 " + std::string(plugin_map[req->mode]);
        RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
      } catch (const std::exception& e) {
        resp->success = false;
        resp->message = e.what();
      }
    }
  }

  /// 伺服动作执行器（长周期任务，支持取消和反馈）
  void execute_servo(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      vision_servo_msgs::action::VisualServo>> goal_handle) {
    servo_active_ = true;
    auto result = std::make_shared<vision_servo_msgs::action::VisualServo::Result>();
    // TODO：动作执行逻辑（目标位姿设置 → 误差收敛检测 → 完成）
    servo_active_ = false;
    goal_handle->succeed(result);
  }

  /**
   * @brief 主控制回路（50 Hz）— 每帧执行。
   *
   * 流程：目标 → computeVelocity() → allocate() → 发布指令
   */
  void control_loop() {
    if (!calibrated_ || !last_target_) return;  // 未标定或无目标 → 跳过

    auto now = this->now();
    double dt = (last_control_time_.nanoseconds() > 0)
      ? (now - last_control_time_).seconds() : 0.02;  // 默认 20ms
    last_control_time_ = now;

    // 步骤 1：计算相机期望速度
    auto cam_vel = controller_->computeVelocity(active_target_, dt);
    if (!cam_vel) return;  // 未收敛或未初始化

    // 步骤 2：控制分配（相机速度 → 底盘 + 云台）
    auto allocation = allocator_.allocate(*cam_vel, last_platform_state_, dt);

    // 步骤 3：发布底盘速度指令
    auto twist_stamped = geometry_msgs::msg::TwistStamped();
    twist_stamped.header.stamp = now;
    twist_stamped.header.frame_id = "base_link";
    twist_stamped.twist = allocation.chassis_twist;
    chassis_cmd_pub_->publish(twist_stamped);

    // 步骤 4：发布云台指令
    vision_servo_msgs::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header.stamp = now;
    gimbal_cmd.yaw_rate = allocation.gimbal_yaw_rate;
    gimbal_cmd.pitch_rate = allocation.gimbal_pitch_rate;
    gimbal_cmd.hold_yaw = allocation.hold_yaw;
    gimbal_cmd.hold_pitch = allocation.hold_pitch;
    gimbal_cmd_pub_->publish(gimbal_cmd);

    // 步骤 5：发布伺服状态（用于监控和调试）
    auto servo_state = controller_->getServoState();
    servo_state.header.stamp = now;
    servo_state_pub_->publish(servo_state);
  }

  // ── 控制器 ──────────────────────────────────────────────────────
  std::unique_ptr<pluginlib::ClassLoader<ServoControllerBase>> controller_loader_;
  std::shared_ptr<ServoControllerBase> controller_;  ///< 当前活跃的控制器实例
  ControlAllocator allocator_;                        ///< 控制分配器
  bool calibrated_ = false;                           ///< 相机标定是否完成
  bool servo_active_ = false;                         ///< 伺服动作是否活跃

  // ── 订阅者 ──────────────────────────────────────────────────────
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr target_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::PlatformState>::SharedPtr platform_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // ── 发布者 ──────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_cmd_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::ServoState>::SharedPtr servo_state_pub_;

  // ── 服务与动作 ──────────────────────────────────────────────────
  rclcpp::Service<vision_servo_msgs::srv::SetServoMode>::SharedPtr servo_mode_srv_;
  rclcpp_action::Server<vision_servo_msgs::action::VisualServo>::SharedPtr servo_action_;

  // ── 定时器 ──────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr control_timer_;  ///< 50 Hz 控制回路定时器

  // ── 状态缓存 ────────────────────────────────────────────────────
  vision_servo_msgs::msg::TargetArray::ConstSharedPtr last_target_;
  vision_servo_msgs::msg::Target active_target_;           ///< 当前伺服跟踪的目标
  vision_servo_msgs::msg::PlatformState last_platform_state_;
  rclcpp::Time last_control_time_;
};

}  // namespace servo_control_pkg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<servo_control_pkg::ServoManagerNode>());
  rclcpp::shutdown();
  return 0;
}
