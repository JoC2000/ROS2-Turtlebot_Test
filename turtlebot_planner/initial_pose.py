#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPose(Node):
    def __init__(self):
        super().__init__("initial_pose_node")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)

        self.x = self.get_parameter("x").get_parameter_value().double_value
        self.y = self.get_parameter("y").get_parameter_value().double_value
        self.yaw = self.get_parameter("yaw").get_parameter_value().double_value

        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0685, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0685, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]

        self.publisher_.publish(msg)
        rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)
    node = InitialPose()
    rclpy.spin(node)

if __name__ == '__main__':
    main()