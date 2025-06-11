#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <turtlebot_test/solver_utils.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace std::chrono_literals;
using ActionNeighborsType = std::vector<std::pair<std::string, std::pair<int, int>>>;
using PathType = std::vector<std::pair<int, int>>;

// Define Maze Solver node class
class MazeSolver : public rclcpp::Node {
public:
    explicit MazeSolver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
private:
    /**
     * @brief Receives map data and starts solver.
     * @param msg Shared Ptr to OccupancyGrid type msg.
    */
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    /**
     * @brief Get the cell coordinates of the robot.
     * @return A pair of integers representing the robot position in cell coordinates.
    */
    std::pair<int, int> get_robot_cell();
    /**
     * @brief Get the neighbors of the 
     * @param state A pair of integers that represents the state of the agent in map cell coordinates x and y.
     * @param grid A vector of vectors of integers, the 2-D representation of the map.
     * @return The neighbors coordinates and the action needed to reach each neighbor.
    */
    ActionNeighborsType get_neighbors(
        const std::pair<int, int> &state,
        const std::vector<std::vector<int>> &grid
    );
    /** 
     * @brief Reconstruct the path to reach the goal.
     * @return A vector of pairs of integers representing the path.
    */
    PathType reconstruct_path(Agent* node);
    /** 
     * @brief Convert from world coordinates (m) to cell coordinates.
     * @param x X coordinate in meters.
     * @param y Y coordinates in meters.
     * @return A pair of integers representing te position in cell coordinates.
    */
    std::pair<int, int> world_to_grid(double x, double y);
    /** 
     * @brief Convert from cell coordinates to world coordinates (m).
     * @return A pair of doubles representing te position in world coordinates (m).
    */
    std::pair<double, double> grid_to_world(const std::pair<int, int> &cell);

    /** 
     * @brief Publish created path as Path msg
     * @param path Vector of pair of integers representing the path created in cell coordinates.
    */
    void publish_path(const PathType &path);

    /** 
     * @brief Run the algorithm depending on the used class. Frontier approach.
    */
    void run_solver();

    bool map_received_ = false;
    double last_distance_e = 0.0;
    double last_angle_e = 0.0;
    double dt = 0.1;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    nav_msgs::msg::OccupancyGrid current_map_;
};