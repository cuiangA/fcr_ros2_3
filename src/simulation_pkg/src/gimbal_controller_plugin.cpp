/**
 * @file gimbal_controller_plugin.cpp
 * @brief Gazebo ModelPlugin — subscribes to /cmd_gimbal and drives
 *        gimbal_yaw_joint / gimbal_pitch_joint via SetVelocity().
 *
 * Works alongside the existing gimbal_driver node:
 *   - This plugin handles 3D visualization (joints actually move)
 *   - gimbal_driver node handles the platform state feedback chain
 *
 * Joint names are configured via SDF parameters (defaults match the URDF).
 */

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>

#include <mutex>
#include <string>

namespace simulation_pkg {

class GimbalControllerPlugin : public gazebo::ModelPlugin {
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;

    // ── Read joint names from SDF parameters ─────────────────────────
    if (sdf->HasElement("yaw_joint")) {
      yaw_joint_name_ = sdf->Get<std::string>("yaw_joint");
    }
    if (sdf->HasElement("pitch_joint")) {
      pitch_joint_name_ = sdf->Get<std::string>("pitch_joint");
    }

    yaw_joint_ = model_->GetJoint(yaw_joint_name_);
    if (!yaw_joint_) {
      RCLCPP_ERROR(rclcpp::get_logger("gimbal_controller_plugin"),
                   "Joint '%s' not found in model '%s'",
                   yaw_joint_name_.c_str(), model_->GetName().c_str());
      return;
    }

    pitch_joint_ = model_->GetJoint(pitch_joint_name_);
    if (!pitch_joint_) {
      RCLCPP_ERROR(rclcpp::get_logger("gimbal_controller_plugin"),
                   "Joint '%s' not found in model '%s'",
                   pitch_joint_name_.c_str(), model_->GetName().c_str());
      return;
    }

    // ── Create ROS2 node via gazebo_ros bridge ───────────────────────
    ros_node_ = gazebo_ros::Node::Get(sdf);

    auto reliable_qos = rclcpp::QoS(10).reliable();
    cmd_sub_ = ros_node_->create_subscription<vision_servo_msgs::msg::GimbalCmd>(
      "/cmd_gimbal", reliable_qos,
      [this](const vision_servo_msgs::msg::GimbalCmd::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_cmd_ = msg;
      });

    // ── Register Gazebo update callback ──────────────────────────────
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GimbalControllerPlugin::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(),
                "GimbalControllerPlugin loaded: yaw=%s, pitch=%s",
                yaw_joint_name_.c_str(), pitch_joint_name_.c_str());
  }

private:
  void OnUpdate() {
    vision_servo_msgs::msg::GimbalCmd::SharedPtr cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd = latest_cmd_;
    }

    if (!cmd) return;

    const double yaw_rate = cmd->hold_yaw ? 0.0 : cmd->yaw_rate;
    const double pitch_rate = cmd->hold_pitch ? 0.0 : cmd->pitch_rate;

    yaw_joint_->SetVelocity(0, yaw_rate);
    pitch_joint_->SetVelocity(0, pitch_rate);
  }

  // ── Gazebo objects ─────────────────────────────────────────────────
  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr yaw_joint_;
  gazebo::physics::JointPtr pitch_joint_;
  gazebo::event::ConnectionPtr update_connection_;

  // ── ROS2 objects ───────────────────────────────────────────────────
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalCmd>::SharedPtr cmd_sub_;

  // ── Thread-safe command buffer ─────────────────────────────────────
  std::mutex cmd_mutex_;
  vision_servo_msgs::msg::GimbalCmd::SharedPtr latest_cmd_;

  // ── Configuration ──────────────────────────────────────────────────
  std::string yaw_joint_name_{"gimbal_yaw_joint"};
  std::string pitch_joint_name_{"gimbal_pitch_joint"};
};

GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)

}  // namespace simulation_pkg
