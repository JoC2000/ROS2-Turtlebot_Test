#ifndef MAZE_SOLVER_HPP
#define MAZE_SOLVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <turtlebot_test/solver_utils.hpp>

using namespace std::chrono_literals;
using ActionNeighborsType = std::vector<std::pair<std::string, std::pair<int,int>>>;
using PathType = std::vector<std::pair<int, int>>;

// Define Maze Solver node class
class MazeSolver : public rclcpp::Node {
    public:
        explicit MazeSolver();

    private:
        // Map callback function will receive a pointer to OccupancyGrid of Map
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        // Return a pair of coordinates of the robot cell
        std::pair<int, int> get_robot_cell();
        // Get neighbors is a vector of pairs containing a string and a pair of coordinates
        // Function will receive a referente to the state and grid of map
        // Will return a vector of pairs containing the action and the coordinates of the neighbor
        ActionNeighborsType get_neighbors(
            const std::pair<int, int> &state,
            const std::vector<std::vector<int>> &grid
        );
        // A vector of pairs containing the already walked path by the robot
        PathType reconstruct_path(Agent* node);
        // Will publish velocity commands to turtlebot to run to the goal pose
        void publish_cmd_vel(const std::pair<int, int> &cell);

        void run_solver();

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        nav_msgs::msg::OccupancyGrid current_map_;
        bool map_received_ = false;
};

#endif