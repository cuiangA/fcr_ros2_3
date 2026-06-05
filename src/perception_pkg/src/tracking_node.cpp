/**
 * @file tracking_node.cpp
 * @brief 多目标跟踪节点实现；MultiObjectTracker 核心逻辑在 multi_object_tracker.cpp 中共享。
 */

#include "perception_pkg/tracking_node.hpp"
#include "perception_pkg/qos.hpp"

namespace perception_pkg {

TrackingNode::TrackingNode(const rclcpp::NodeOptions& options)
  : Node("tracking_node", options),
    tracker_(/*max_age=*/30, /*min_hits=*/3, /*iou_threshold=*/0.3),
    active_tracking_id_(-1),
    tracking_enabled_(true)
{
  this->declare_parameter("max_age", 30);
  this->declare_parameter("min_hits", 3);
  this->declare_parameter("iou_threshold", 0.3);
  this->declare_parameter("camera_frame", "camera_link");

  camera_frame_ = this->get_parameter("camera_frame").as_string();
  const int max_age = this->get_parameter("max_age").as_int();
  const int min_hits = this->get_parameter("min_hits").as_int();
  const float iou_threshold =
      static_cast<float>(this->get_parameter("iou_threshold").as_double());
  tracker_ = MultiObjectTracker(max_age, min_hits, iou_threshold);

  det_sub_ = this->create_subscription<vision_servo_msgs::msg::TargetArray>(
    "/perception/detections", qos::detections(),
    std::bind(&TrackingNode::detection_callback, this, std::placeholders::_1));

  track_pub_ = this->create_publisher<vision_servo_msgs::msg::TargetArray>(
    "/perception/tracks", qos::detections());

  tracking_srv_ = this->create_service<vision_servo_msgs::srv::SetTrackingTarget>(
    "~/set_tracking_target",
    [this](
      const std::shared_ptr<vision_servo_msgs::srv::SetTrackingTarget::Request> req,
      std::shared_ptr<vision_servo_msgs::srv::SetTrackingTarget::Response> resp) {
      set_tracking_target(req->target_id, req->class_name, req->enable);
      resp->success = true;
      resp->message = "OK";
      resp->assigned_id = active_tracking_id_;
    });

  RCLCPP_INFO(get_logger(), "跟踪节点已启动 (max_age=%d, min_hits=%d)",
              max_age, min_hits);
}

void TrackingNode::detection_callback(
    const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg)
{
  tracker_.update(*msg);
  auto tracks = tracker_.get_tracks(
      static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(msg->header.stamp.nanosec),
      camera_frame_);

  if (tracking_enabled_ && active_tracking_id_ < 0 && !tracks.targets.empty()) {
    active_tracking_id_ = tracks.targets[0].id;
  }

  tracks.tracking_id = active_tracking_id_;
  tracks.header = msg->header;
  track_pub_->publish(tracks);
}

void TrackingNode::set_tracking_target(
    int target_id, const std::string& class_name, bool enable)
{
  tracking_enabled_ = enable;
  active_tracking_id_ = target_id;
  target_class_filter_ = class_name;
  RCLCPP_INFO(get_logger(), "设置跟踪目标: id=%d, class=%s, enable=%d",
              target_id, class_name.c_str(), enable);
}

}  // namespace perception_pkg

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(perception_pkg::TrackingNode)

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::TrackingNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
