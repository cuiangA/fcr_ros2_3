#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_servo_msgs/action/cinematic_shot.hpp>
#include <vision_servo_msgs/msg/platform_state.hpp>
#include <vision_servo_msgs/msg/shot_reference.hpp>
#include <vision_servo_msgs/msg/target_state.hpp>

#include "servo_control_pkg/qos.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double smoothstep(double value)
{
  const double x = clamp(value, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

}  // namespace

class ShotReferenceGeneratorNode : public rclcpp::Node
{
public:
  using CinematicShot = vision_servo_msgs::action::CinematicShot;
  using GoalHandleShot = rclcpp_action::ServerGoalHandle<CinematicShot>;
  using ShotReference = vision_servo_msgs::msg::ShotReference;

  ShotReferenceGeneratorNode()
  : Node("shot_reference_generator_node")
  {
    target_state_topic_ = declare_parameter<std::string>("target_state_topic", "/target/state");
    platform_state_topic_ = declare_parameter<std::string>("platform_state_topic", "/platform/state");
    output_topic_ = declare_parameter<std::string>("output_topic", "/shot/reference");
    action_name_ = declare_parameter<std::string>("action_name", "/shot/execute");
    output_frame_ = declare_parameter<std::string>("output_frame", "odom");
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 30.0);
    auto_start_ = declare_parameter<bool>("auto_start", true);
    loop_auto_start_shot_ = declare_parameter<bool>("loop_auto_start_shot", false);
    default_shot_type_ = parse_shot_type(
      declare_parameter<std::string>("default_shot_name", "follow"),
      declare_parameter<int>("default_shot_type", ShotReference::SHOT_FOLLOW));
    default_duration_ = declare_parameter<double>("default_duration", 12.0);
    default_radius_ = declare_parameter<double>("default_radius", 2.0);
    default_angle_deg_ = declare_parameter<double>("default_angle_deg", 180.0);
    default_start_distance_ = declare_parameter<double>("default_start_distance", 2.5);
    default_end_distance_ = declare_parameter<double>("default_end_distance", 1.5);
    default_lateral_offset_ = declare_parameter<double>("default_lateral_offset", 1.5);
    default_longitudinal_offset_ = declare_parameter<double>("default_longitudinal_offset", -2.0);
    desired_composition_u_ = declare_parameter<double>("desired_composition_u", 0.5);
    desired_composition_v_ = declare_parameter<double>("desired_composition_v", 0.5);
    max_base_speed_ = declare_parameter<double>("max_base_speed", 0.8);
    max_base_yaw_rate_ = declare_parameter<double>("max_base_yaw_rate", 1.2);
    max_gimbal_yaw_rate_ = declare_parameter<double>("max_gimbal_yaw_rate", 1.5);
    max_gimbal_pitch_rate_ = declare_parameter<double>("max_gimbal_pitch_rate", 1.0);
    tracking_error_gain_ = declare_parameter<double>("tracking_error_gain", 0.8);
    speed_limit_threshold_ = declare_parameter<double>("speed_limit_threshold", 0.85);
    hold_speed_scale_threshold_ = declare_parameter<double>("hold_speed_scale_threshold", 0.2);
    target_speed_fallback_threshold_ = declare_parameter<double>("target_speed_fallback_threshold", 1.2);
    allow_follow_fallback_ = declare_parameter<bool>("allow_follow_fallback", true);
    target_stale_timeout_ = declare_parameter<double>("target_stale_timeout", 0.5);
    platform_stale_timeout_ = declare_parameter<double>("platform_stale_timeout", 0.5);

    update_rate_hz_ = std::max(update_rate_hz_, 1.0);
    default_duration_ = std::max(default_duration_, 0.1);
    default_radius_ = std::max(default_radius_, 0.1);
    max_base_speed_ = std::max(max_base_speed_, 0.01);

    reference_pub_ = create_publisher<ShotReference>(output_topic_, rclcpp::QoS(10).reliable());
    target_state_sub_ = create_subscription<vision_servo_msgs::msg::TargetState>(
      target_state_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&ShotReferenceGeneratorNode::target_callback, this, std::placeholders::_1));
    platform_state_sub_ = create_subscription<vision_servo_msgs::msg::PlatformState>(
      platform_state_topic_, servo_control_pkg::qos::platform_state(),
      std::bind(&ShotReferenceGeneratorNode::platform_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<CinematicShot>(
      this,
      action_name_,
      std::bind(&ShotReferenceGeneratorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ShotReferenceGeneratorNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ShotReferenceGeneratorNode::handle_accepted, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&ShotReferenceGeneratorNode::update, this));

    RCLCPP_INFO(get_logger(), "Shot reference generator started: %s", output_topic_.c_str());
  }

private:
  struct ShotParams
  {
    uint8_t shot_type{ShotReference::SHOT_FOLLOW};
    double duration{12.0};
    double radius{2.0};
    double angle_rad{kPi};
    bool clockwise{false};
    double start_distance{2.5};
    double end_distance{1.5};
    double longitudinal_offset{-2.0};
    double lateral_offset{1.5};
    double composition_start_u{0.5};
    double composition_start_v{0.5};
    double composition_end_u{0.5};
    double composition_end_v{0.5};
    double max_base_speed{0.8};
    double max_base_yaw_rate{1.2};
    double max_gimbal_yaw_rate{1.5};
    double max_gimbal_pitch_rate{1.0};
    bool return_to_follow{true};
  };

  static uint8_t parse_shot_type(const std::string & name, int fallback)
  {
    const std::string lower = [&]() {
      std::string out = name;
      std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      return out;
    }();
    if (lower == "pan") {
      return ShotReference::SHOT_PAN;
    }
    if (lower == "recenter") {
      return ShotReference::SHOT_RECENTER;
    }
    if (lower == "dolly_in") {
      return ShotReference::SHOT_DOLLY_IN;
    }
    if (lower == "dolly_out") {
      return ShotReference::SHOT_DOLLY_OUT;
    }
    if (lower == "orbit") {
      return ShotReference::SHOT_ORBIT;
    }
    if (lower == "truck") {
      return ShotReference::SHOT_TRUCK;
    }
    if (lower == "arc_in") {
      return ShotReference::SHOT_ARC_IN;
    }
    if (lower == "follow") {
      return ShotReference::SHOT_FOLLOW;
    }
    return static_cast<uint8_t>(std::clamp(fallback, 0, 7));
  }

  void target_callback(const vision_servo_msgs::msg::TargetState::ConstSharedPtr msg)
  {
    latest_target_ = *msg;
    target_received_time_ = now();
    has_target_ = true;
  }

  void platform_callback(const vision_servo_msgs::msg::PlatformState::ConstSharedPtr msg)
  {
    latest_platform_ = *msg;
    platform_received_time_ = now();
    has_platform_ = true;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CinematicShot::Goal> goal)
  {
    if (active_goal_) {
      RCLCPP_WARN(get_logger(), "Rejecting cinematic shot goal: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->duration <= 0.0f) {
      RCLCPP_WARN(get_logger(), "Rejecting cinematic shot goal: duration must be positive");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleShot>)
  {
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleShot> goal_handle)
  {
    active_goal_ = goal_handle;
    active_goal_params_ = params_from_goal(*goal_handle->get_goal());
    shot_progress_ = 0.0;
    progress_speed_scale_ = 1.0;
    shot_started_ = false;
    previous_reference_valid_ = false;
    cancel_requested_ = false;
    shot_state_ = ShotReference::STATE_PRECHECK;
    state_enter_time_ = now();
    RCLCPP_INFO(
      get_logger(), "Accepted cinematic shot goal type=%u",
      static_cast<unsigned int>(active_goal_params_.shot_type));
  }

  ShotParams default_params() const
  {
    ShotParams params;
    params.shot_type = default_shot_type_;
    params.duration = default_duration_;
    params.radius = default_radius_;
    params.angle_rad = default_angle_deg_ * kPi / 180.0;
    params.start_distance = default_start_distance_;
    params.end_distance = default_end_distance_;
    params.longitudinal_offset = default_longitudinal_offset_;
    params.lateral_offset = default_lateral_offset_;
    params.composition_start_u = desired_composition_u_;
    params.composition_start_v = desired_composition_v_;
    params.composition_end_u = desired_composition_u_;
    params.composition_end_v = desired_composition_v_;
    params.max_base_speed = max_base_speed_;
    params.max_base_yaw_rate = max_base_yaw_rate_;
    params.max_gimbal_yaw_rate = max_gimbal_yaw_rate_;
    params.max_gimbal_pitch_rate = max_gimbal_pitch_rate_;
    return params;
  }

  ShotParams params_from_goal(const CinematicShot::Goal & goal) const
  {
    ShotParams params = default_params();
    params.shot_type = goal.shot_type;
    params.duration = std::max(static_cast<double>(goal.duration), 0.1);
    params.radius = goal.radius > 0.0f ? goal.radius : params.radius;
    params.angle_rad = goal.angle != 0.0f ? goal.angle : params.angle_rad;
    params.clockwise = goal.clockwise;
    params.start_distance = goal.start_distance > 0.0f ? goal.start_distance : params.start_distance;
    params.end_distance = goal.end_distance > 0.0f ? goal.end_distance : params.end_distance;
    params.longitudinal_offset = goal.longitudinal_offset;
    params.lateral_offset = goal.lateral_offset;
    params.composition_start_u = goal.composition_start_u != 0.0f ? goal.composition_start_u : desired_composition_u_;
    params.composition_start_v = goal.composition_start_v != 0.0f ? goal.composition_start_v : desired_composition_v_;
    params.composition_end_u = goal.composition_end_u != 0.0f ? goal.composition_end_u : params.composition_start_u;
    params.composition_end_v = goal.composition_end_v != 0.0f ? goal.composition_end_v : params.composition_start_v;
    params.max_base_speed = goal.max_base_speed > 0.0f ? goal.max_base_speed : max_base_speed_;
    params.max_base_yaw_rate = goal.max_base_yaw_rate > 0.0f ? goal.max_base_yaw_rate : max_base_yaw_rate_;
    params.max_gimbal_yaw_rate = goal.max_gimbal_yaw_rate > 0.0f ? goal.max_gimbal_yaw_rate : max_gimbal_yaw_rate_;
    params.max_gimbal_pitch_rate = goal.max_gimbal_pitch_rate > 0.0f ? goal.max_gimbal_pitch_rate : max_gimbal_pitch_rate_;
    params.return_to_follow = goal.return_to_follow;
    return params;
  }

  bool data_ready() const
  {
    const auto t = now();
    const bool target_fresh = has_target_ &&
      (t - target_received_time_).seconds() <= target_stale_timeout_ &&
      latest_target_.valid;
    const bool platform_fresh = has_platform_ &&
      (t - platform_received_time_).seconds() <= platform_stale_timeout_;
    return target_fresh && platform_fresh;
  }

  void update()
  {
    const rclcpp::Time t = now();
    const double dt = last_update_time_.nanoseconds() == 0 ? 0.0 : (t - last_update_time_).seconds();
    last_update_time_ = t;

    if (!data_ready()) {
      publish_invalid_reference(t, active_goal_ ? ShotReference::STATE_RECOVER_TARGET : ShotReference::STATE_IDLE);
      return;
    }

    ShotParams params = active_goal_ ? active_goal_params_ : default_params();
    if (!active_goal_ && !auto_start_) {
      publish_invalid_reference(t, ShotReference::STATE_IDLE);
      return;
    }

    if (cancel_requested_) {
      publish_invalid_reference(t, ShotReference::STATE_CANCELED);
      finish_goal(false, ShotReference::STATE_CANCELED, "shot canceled");
      return;
    }

    if (!shot_started_) {
      capture_start_geometry(params);
      shot_started_ = true;
      shot_progress_ = 0.0;
      state_enter_time_ = t;
      shot_state_ = ShotReference::STATE_ACTIVE;
    }

    auto reference = build_reference(params, t, dt);
    if (latest_target_.speed > target_speed_fallback_threshold_ && allow_follow_fallback_) {
      params.shot_type = ShotReference::SHOT_FOLLOW;
      reference = build_reference(params, t, dt);
      reference.state = ShotReference::STATE_FOLLOW_FALLBACK;
      reference.status = "target too fast; falling back to follow reference";
    }

    reference_pub_->publish(reference);
    publish_feedback(reference);

    if (active_goal_ && shot_progress_ >= 1.0) {
      finish_goal(true, ShotReference::STATE_COMPLETED, "shot completed");
    }
  }

  void capture_start_geometry(const ShotParams & params)
  {
    const double target_x = latest_target_.position.x;
    const double target_y = latest_target_.position.y;
    const double robot_x = latest_platform_.chassis_pose[0];
    const double robot_y = latest_platform_.chassis_pose[1];
    start_theta_ = std::atan2(robot_y - target_y, robot_x - target_x);
    if (!std::isfinite(start_theta_)) {
      start_theta_ = 0.0;
    }
    if (params.shot_type == ShotReference::SHOT_DOLLY_IN ||
      params.shot_type == ShotReference::SHOT_DOLLY_OUT)
    {
      start_radius_ = std::max(params.start_distance, 0.1);
    } else {
      start_radius_ = std::hypot(robot_x - target_x, robot_y - target_y);
      if (start_radius_ < 0.1) {
        start_radius_ = params.radius;
      }
    }
  }

  ShotReference build_reference(const ShotParams & params, const rclcpp::Time & stamp, double dt)
  {
    ShotReference reference;
    reference.header.stamp = stamp;
    reference.header.frame_id = output_frame_;
    reference.mode = params.shot_type == ShotReference::SHOT_FOLLOW ?
      ShotReference::MODE_FOLLOW : ShotReference::MODE_ACTION_SHOT;
    reference.shot_type = params.shot_type;
    reference.target_id = latest_target_.target_id;
    reference.target_position = latest_target_.position;
    reference.target_velocity = latest_target_.velocity;
    reference.target_speed = latest_target_.speed;
    reference.max_base_speed = params.max_base_speed;
    reference.max_base_yaw_rate = params.max_base_yaw_rate;
    reference.max_gimbal_yaw_rate = params.max_gimbal_yaw_rate;
    reference.max_gimbal_pitch_rate = params.max_gimbal_pitch_rate;
    reference.gimbal_tracking_enabled = true;
    reference.base_motion_enabled = true;
    reference.valid = true;

    update_progress(params, reference, dt);

    const double p = smoothstep(shot_progress_);
    reference.progress = static_cast<float>(shot_progress_);
    reference.desired_composition_u =
      static_cast<float>((1.0 - p) * params.composition_start_u + p * params.composition_end_u);
    reference.desired_composition_v =
      static_cast<float>((1.0 - p) * params.composition_start_v + p * params.composition_end_v);

    double radius = params.radius;
    double theta = start_theta_;
    double tx = latest_target_.position.x;
    double ty = latest_target_.position.y;
    const double sign = params.clockwise ? -1.0 : 1.0;

    switch (params.shot_type) {
      case ShotReference::SHOT_DOLLY_IN:
      case ShotReference::SHOT_DOLLY_OUT:
        radius = (1.0 - p) * params.start_distance + p * params.end_distance;
        theta = start_theta_;
        break;
      case ShotReference::SHOT_ORBIT:
        radius = params.radius;
        theta = start_theta_ + sign * params.angle_rad * shot_progress_;
        break;
      case ShotReference::SHOT_TRUCK:
        build_truck_reference(params, tx, ty, theta, radius);
        break;
      case ShotReference::SHOT_ARC_IN:
        radius = (1.0 - p) * params.start_distance + p * params.end_distance;
        theta = start_theta_ + sign * params.angle_rad * shot_progress_;
        break;
      case ShotReference::SHOT_PAN:
      case ShotReference::SHOT_RECENTER:
        reference.base_motion_enabled = false;
        tx = latest_platform_.chassis_pose[0];
        ty = latest_platform_.chassis_pose[1];
        radius = 0.0;
        theta = latest_platform_.chassis_pose[2];
        break;
      case ShotReference::SHOT_FOLLOW:
      default:
        radius = params.radius;
        theta = start_theta_;
        break;
    }

    if (params.shot_type != ShotReference::SHOT_TRUCK &&
      params.shot_type != ShotReference::SHOT_PAN &&
      params.shot_type != ShotReference::SHOT_RECENTER)
    {
      tx = latest_target_.position.x + radius * std::cos(theta);
      ty = latest_target_.position.y + radius * std::sin(theta);
    }

    const double yaw_to_target = std::atan2(
      latest_target_.position.y - ty,
      latest_target_.position.x - tx);
    // TODO: Replace this geometric reference with obstacle-aware MPC/QP when local map data exists.
    reference.virtual_base_pose.position.x = tx;
    reference.virtual_base_pose.position.y = ty;
    reference.virtual_base_pose.position.z = 0.0;
    reference.virtual_base_pose.orientation = yaw_to_quaternion(yaw_to_target);
    reference.desired_distance = static_cast<float>(radius);
    reference.orbit_radius = static_cast<float>(radius);
    reference.orbit_angle = static_cast<float>(theta);

    const double error = std::hypot(
      tx - latest_platform_.chassis_pose[0],
      ty - latest_platform_.chassis_pose[1]);
    const double shot_motion_speed = estimate_reference_speed(reference, dt);
    const double required_speed =
      static_cast<double>(latest_target_.speed) + shot_motion_speed + tracking_error_gain_ * error;
    const double speed_scale = required_speed > params.max_base_speed ?
      params.max_base_speed / required_speed : 1.0;
    reference.required_base_speed = static_cast<float>(required_speed);
    reference.speed_scale = static_cast<float>(clamp(speed_scale, 0.0, 1.0));

    if (!reference.base_motion_enabled) {
      reference.state = ShotReference::STATE_ACTIVE;
      reference.status = "base held; gimbal tracks target";
    } else if (reference.speed_scale < hold_speed_scale_threshold_) {
      reference.state = ShotReference::STATE_HOLD_SHOT;
      reference.status = "speed budget exhausted; holding shot progress";
    } else if (reference.speed_scale < speed_limit_threshold_) {
      reference.state = ShotReference::STATE_SPEED_LIMITED;
      reference.status = "speed limited; shot progress slowed";
    } else {
      reference.state = ShotReference::STATE_ACTIVE;
      reference.status = "shot active";
    }
    shot_state_ = reference.state;
    progress_speed_scale_ = reference.speed_scale;

    if (!active_goal_ && loop_auto_start_shot_ && shot_progress_ >= 1.0) {
      shot_progress_ = 0.0;
      previous_reference_valid_ = false;
    }

    previous_reference_position_ = reference.virtual_base_pose.position;
    previous_reference_valid_ = true;
    return reference;
  }

  void update_progress(const ShotParams & params, ShotReference & reference, double dt)
  {
    if (params.shot_type == ShotReference::SHOT_FOLLOW && !active_goal_) {
      shot_progress_ = 0.0;
      progress_speed_scale_ = 1.0;
      reference.progress = 0.0f;
      return;
    }
    if (dt <= 0.0) {
      return;
    }
    const double progress_scale =
      progress_speed_scale_ < hold_speed_scale_threshold_ ? 0.0 : progress_speed_scale_;
    const double progress_step = dt * progress_scale / std::max(params.duration, 0.1);
    shot_progress_ = clamp(shot_progress_ + progress_step, 0.0, 1.0);
  }

  void build_truck_reference(
    const ShotParams & params,
    double & x,
    double & y,
    double & theta,
    double & radius) const
  {
    double heading = std::atan2(latest_target_.velocity.y, latest_target_.velocity.x);
    if (latest_target_.speed < 0.05f) {
      heading = latest_platform_.chassis_pose[2];
    }
    const double forward_x = std::cos(heading);
    const double forward_y = std::sin(heading);
    const double left_x = -std::sin(heading);
    const double left_y = std::cos(heading);
    x = latest_target_.position.x +
      params.longitudinal_offset * forward_x + params.lateral_offset * left_x;
    y = latest_target_.position.y +
      params.longitudinal_offset * forward_y + params.lateral_offset * left_y;
    theta = std::atan2(y - latest_target_.position.y, x - latest_target_.position.x);
    radius = std::hypot(x - latest_target_.position.x, y - latest_target_.position.y);
  }

  double estimate_reference_speed(const ShotReference & reference, double dt) const
  {
    if (!previous_reference_valid_ || dt <= 1e-4) {
      return 0.0;
    }
    return std::hypot(
      reference.virtual_base_pose.position.x - previous_reference_position_.x,
      reference.virtual_base_pose.position.y - previous_reference_position_.y) / dt;
  }

  void publish_feedback(const ShotReference & reference)
  {
    if (!active_goal_) {
      return;
    }
    auto feedback = std::make_shared<CinematicShot::Feedback>();
    feedback->progress = reference.progress;
    feedback->state = reference.state;
    feedback->speed_scale = reference.speed_scale;
    feedback->target_speed = reference.target_speed;
    feedback->required_base_speed = reference.required_base_speed;
    feedback->status = reference.status;
    active_goal_->publish_feedback(feedback);
  }

  void finish_goal(bool success, uint8_t final_state, const std::string & message)
  {
    if (!active_goal_) {
      return;
    }
    auto result = std::make_shared<CinematicShot::Result>();
    result->success = success;
    result->final_state = final_state;
    result->message = message;

    if (cancel_requested_ || final_state == ShotReference::STATE_CANCELED) {
      active_goal_->canceled(result);
    } else if (success) {
      active_goal_->succeed(result);
    } else {
      active_goal_->abort(result);
    }

    active_goal_.reset();
    shot_started_ = false;
    previous_reference_valid_ = false;
    cancel_requested_ = false;
    if (active_goal_params_.return_to_follow) {
      default_shot_type_ = ShotReference::SHOT_FOLLOW;
    }
  }

  void publish_invalid_reference(const rclcpp::Time & stamp, uint8_t state)
  {
    ShotReference reference;
    reference.header.stamp = stamp;
    reference.header.frame_id = output_frame_;
    reference.mode = ShotReference::MODE_STANDBY;
    reference.shot_type = ShotReference::SHOT_FOLLOW;
    reference.state = state;
    reference.target_id = latest_target_.target_id;
    reference.base_motion_enabled = false;
    reference.gimbal_tracking_enabled = false;
    reference.valid = false;
    reference.status = "waiting for fresh target/platform state";
    reference_pub_->publish(reference);
  }

  std::string target_state_topic_;
  std::string platform_state_topic_;
  std::string output_topic_;
  std::string action_name_;
  std::string output_frame_;
  double update_rate_hz_{30.0};
  bool auto_start_{true};
  bool loop_auto_start_shot_{false};
  uint8_t default_shot_type_{ShotReference::SHOT_FOLLOW};
  double default_duration_{12.0};
  double default_radius_{2.0};
  double default_angle_deg_{180.0};
  double default_start_distance_{2.5};
  double default_end_distance_{1.5};
  double default_lateral_offset_{1.5};
  double default_longitudinal_offset_{-2.0};
  double desired_composition_u_{0.5};
  double desired_composition_v_{0.5};
  double max_base_speed_{0.8};
  double max_base_yaw_rate_{1.2};
  double max_gimbal_yaw_rate_{1.5};
  double max_gimbal_pitch_rate_{1.0};
  double tracking_error_gain_{0.8};
  double speed_limit_threshold_{0.85};
  double hold_speed_scale_threshold_{0.2};
  double target_speed_fallback_threshold_{1.2};
  bool allow_follow_fallback_{true};
  double target_stale_timeout_{0.5};
  double platform_stale_timeout_{0.5};

  rclcpp::Publisher<ShotReference>::SharedPtr reference_pub_;
  rclcpp::Subscription<vision_servo_msgs::msg::TargetState>::SharedPtr target_state_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::PlatformState>::SharedPtr platform_state_sub_;
  rclcpp_action::Server<CinematicShot>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_target_{false};
  bool has_platform_{false};
  vision_servo_msgs::msg::TargetState latest_target_;
  vision_servo_msgs::msg::PlatformState latest_platform_;
  rclcpp::Time target_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time platform_received_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<GoalHandleShot> active_goal_;
  ShotParams active_goal_params_;
  bool cancel_requested_{false};
  bool shot_started_{false};
  uint8_t shot_state_{ShotReference::STATE_IDLE};
  double shot_progress_{0.0};
  double progress_speed_scale_{1.0};
  double start_theta_{0.0};
  double start_radius_{2.0};
  bool previous_reference_valid_{false};
  geometry_msgs::msg::Point previous_reference_position_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShotReferenceGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
