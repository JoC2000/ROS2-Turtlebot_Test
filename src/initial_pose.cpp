#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <thread>
#include <chrono>
#include <cmath>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("initial_pose_node");
    auto publisher = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    node->declare_parameter("x", 0.0);
    node->declare_parameter("y", 0.0);
    node->declare_parameter("yaw", 0.0);

    double x, y, yaw;
    node->get_parameter("x", x);
    node->get_parameter("y", y);
    node->get_parameter("yaw", yaw);

    RCLCPP_INFO(node->get_logger(), "Publishing initial pose...");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();

    msg.header.stamp = node->get_clock()->now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    msg.pose.covariance = {
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0685, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0685, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
    };

    publisher->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Initial pose published. Node shutting down.");
    rclcpp::shutdown();
    return 0;
}