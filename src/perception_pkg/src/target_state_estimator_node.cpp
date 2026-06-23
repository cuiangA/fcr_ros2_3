#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>
#include <vision_servo_msgs/msg/target_state.hpp>

#include "perception_pkg/qos.hpp"

namespace
{

float clamp01(float value)
{
  return std::max(0.0f, std::min(1.0f, value));
}

bool zero_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

}  // namespace

class TargetStateEstimatorNode : public rclcpp::Node
{
public:
  TargetStateEstimatorNode()
  : Node("target_state_estimator_node")
  {
    target_topic_ = declare_parameter<std::string>("target_topic", "/target/current");
    output_topic_ = declare_parameter<std::string>("output_topic", "/target/state");
    output_frame_ = declare_parameter<std::string>("output_frame", "odom");
    fallback_input_frame_ = declare_parameter<std::string>("fallback_input_frame", "camera_optical_link");
    target_id_ = declare_parameter<int>("target_id", -1);
    velocity_filter_alpha_ = declare_parameter<double>("velocity_filter_alpha", 0.35);
    min_dt_ = declare_parameter<double>("min_dt", 0.005);
    max_dt_ = declare_parameter<double>("max_dt", 0.5);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.05);

    velocity_filter_alpha_ = std::clamp(velocity_filter_alpha_, 0.0, 1.0);
    min_dt_ = std::max(min_dt_, 1e-4);
    max_dt_ = std::max(max_dt_, min_dt_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    state_pub_ = create_publisher<vision_servo_msgs::msg::TargetState>(
      output_topic_, rclcpp::QoS(10).reliable());
    target_sub_ = create_subscription<vision_servo_msgs::msg::TargetArray>(
      target_topic_, perception_pkg::qos::detections(),
      std::bind(&TargetStateEstimatorNode::target_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Target state estimator started: %s -> %s in frame %s",
      target_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str());
  }

private:
  void target_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr msg)
  {
    const auto * target = select_target(*msg);
    if (target == nullptr) {
      publish_lost(*msg, "no selected target");
      return;
    }

    auto point_in = geometry_msgs::msg::PointStamped();
    point_in.header = target->header;
    if (point_in.header.frame_id.empty()) {
      point_in.header.frame_id = msg->header.frame_id.empty() ? fallback_input_frame_ : msg->header.frame_id;
    }
    if (zero_stamp(point_in.header.stamp)) {
      point_in.header.stamp = zero_stamp(msg->header.stamp) ? now() : msg->header.stamp;
    }
    point_in.point.x = target->position[0];
    point_in.point.y = target->position[1];
    point_in.point.z = target->position[2];

    // TODO: Load calibrated Sony/Orbbec extrinsics and choose the active optical frame at runtime.
    geometry_msgs::msg::PointStamped point_out;
    try {
      if (point_in.header.frame_id == output_frame_) {
        point_out = point_in;
      } else {
        point_out = tf_buffer_->transform(
          point_in, output_frame_, tf2::durationFromSec(transform_timeout_sec_));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Target transform failed from %s to %s: %s",
        point_in.header.frame_id.c_str(), output_frame_.c_str(), ex.what());
      publish_lost(*msg, "target transform failed");
      return;
    }

    auto state = vision_servo_msgs::msg::TargetState();
    state.header = point_out.header;
    state.header.frame_id = output_frame_;
    state.target_id = target->id;
    state.class_name = target->class_name;
    state.position = point_out.point;
    state.position_confidence = clamp01(target->confidence * target->depth_confidence);
    state.velocity_confidence = 0.0f;
    state.image_center[0] = target->center[0];
    state.image_center[1] = target->center[1];
    state.bbox[0] = target->bbox[0];
    state.bbox[1] = target->bbox[1];
    state.bbox[2] = target->bbox[2];
    state.bbox[3] = target->bbox[3];
    state.tracking_status = vision_servo_msgs::msg::TargetState::STATUS_TRACKING;
    state.valid = true;

    update_velocity(state);
    state_pub_->publish(state);
  }

  const vision_servo_msgs::msg::Target * select_target(
    const vision_servo_msgs::msg::TargetArray & msg) const
  {
    if (msg.targets.empty()) {
      return nullptr;
    }

    const int preferred_id = target_id_ >= 0 ? target_id_ : msg.tracking_id;
    if (preferred_id >= 0) {
      for (const auto & target : msg.targets) {
        if (target.id == preferred_id) {
          return &target;
        }
      }
    }
    return &msg.targets.front();
  }

  void update_velocity(vision_servo_msgs::msg::TargetState & state)
  {
    const rclcpp::Time stamp(state.header.stamp);
    if (!has_previous_ || previous_target_id_ != state.target_id) {
      reset_velocity_state(state, stamp);
      return;
    }

    const double dt = (stamp - previous_stamp_).seconds();
    if (dt < min_dt_ || dt > max_dt_) {
      reset_velocity_state(state, stamp);
      return;
    }

    const double vx = (state.position.x - previous_position_.x) / dt;
    const double vy = (state.position.y - previous_position_.y) / dt;
    const double vz = (state.position.z - previous_position_.z) / dt;

    filtered_velocity_[0] =
      velocity_filter_alpha_ * vx + (1.0 - velocity_filter_alpha_) * filtered_velocity_[0];
    filtered_velocity_[1] =
      velocity_filter_alpha_ * vy + (1.0 - velocity_filter_alpha_) * filtered_velocity_[1];
    filtered_velocity_[2] =
      velocity_filter_alpha_ * vz + (1.0 - velocity_filter_alpha_) * filtered_velocity_[2];

    state.velocity.x = filtered_velocity_[0];
    state.velocity.y = filtered_velocity_[1];
    state.velocity.z = filtered_velocity_[2];
    state.speed = static_cast<float>(std::hypot(filtered_velocity_[0], filtered_velocity_[1]));
    state.velocity_confidence = state.position_confidence;

    previous_position_ = state.position;
    previous_stamp_ = stamp;
    previous_target_id_ = state.target_id;
  }

  void reset_velocity_state(
    vision_servo_msgs::msg::TargetState & state,
    const rclcpp::Time & stamp)
  {
    filtered_velocity_ = {0.0, 0.0, 0.0};
    state.velocity.x = 0.0;
    state.velocity.y = 0.0;
    state.velocity.z = 0.0;
    state.speed = 0.0f;
    state.velocity_confidence = 0.0f;
    previous_position_ = state.position;
    previous_stamp_ = stamp;
    previous_target_id_ = state.target_id;
    has_previous_ = true;
  }

  void publish_lost(
    const vision_servo_msgs::msg::TargetArray & source,
    const std::string & reason)
  {
    auto state = vision_servo_msgs::msg::TargetState();
    state.header.stamp = zero_stamp(source.header.stamp) ? now() : source.header.stamp;
    state.header.frame_id = output_frame_;
    state.target_id = target_id_;
    state.tracking_status = vision_servo_msgs::msg::TargetState::STATUS_LOST;
    state.valid = false;
    state.position_confidence = 0.0f;
    state.velocity_confidence = 0.0f;
    state.speed = 0.0f;
    state_pub_->publish(state);

    has_previous_ = false;
    RCLCPP_DEBUG(get_logger(), "Target state lost: %s", reason.c_str());
  }

  std::string target_topic_;
  std::string output_topic_;
  std::string output_frame_;
  std::string fallback_input_frame_;
  int target_id_{-1};
  double velocity_filter_alpha_{0.35};
  double min_dt_{0.005};
  double max_dt_{0.5};
  double transform_timeout_sec_{0.05};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr target_sub_;
  rclcpp::Publisher<vision_servo_msgs::msg::TargetState>::SharedPtr state_pub_;

  bool has_previous_{false};
  int previous_target_id_{-1};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Point previous_position_;
  std::array<double, 3> filtered_velocity_{0.0, 0.0, 0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetStateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
