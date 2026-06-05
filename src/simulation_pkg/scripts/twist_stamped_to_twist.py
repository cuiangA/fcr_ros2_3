#!/usr/bin/env python3
"""
Convert geometry_msgs/TwistStamped commands to geometry_msgs/Twist for Gazebo plugins.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped


class TwistStampedToTwist(Node):
    def __init__(self):
        super().__init__("twist_stamped_to_twist")
        self.declare_parameter("input_topic", "/cmd_vel_stamped")
        self.declare_parameter("output_topic", "/cmd_vel")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(Twist, output_topic, qos)
        self.sub = self.create_subscription(TwistStamped, input_topic, self.callback, qos)
        self.get_logger().info(
            f"TwistStamped->Twist 适配器已启动: {input_topic} -> {output_topic}"
        )

    def callback(self, msg: TwistStamped):
        self.pub.publish(msg.twist)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
