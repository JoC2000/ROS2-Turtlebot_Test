#include "turtlebot_test/maze_solver.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <set>
#include <chrono>

MazeSolver::MazeSolver(const rclcpp::NodeOptions &options) : Node("maze_solver", options) {
    // Subscribe to map topic in order to get map info like origin, grid, resolution, etc
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos, std::bind(&MazeSolver::map_callback, this, std::placeholders::_1)
    );
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", qos);
    // Publisher to the velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // tf buffer and listener to get actual robot pose relative to the map frame
    // Using shared pointer in case another node needs to access the same buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
    RCLCPP_INFO(this->get_logger(), "MazeSolver node initialized");
}

/**
*  Receives a pointer to OccupancyGrid message
*  Sets the current_map_ to the received pointer and sets the map_received_ flag to true
*  Calls the run_solver function
*/
void MazeSolver::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Map received: resolution=%.6f, origin=(%.2f, %.2f), width=%d, height=%d",
    msg->info.resolution,
    msg->info.origin.position.x,
    msg->info.origin.position.y,
    msg->info.width,
    msg->info.height);
    current_map_ = *msg;
    map_received_ = true;
    run_solver();
}

std::pair<int, int> MazeSolver::get_robot_cell() {
    try {
        // Transform from the robot frame to the map frame
        auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        // Position of the robot relative to the map frame
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;

        // Get the map resolution and origin
        double resolution = current_map_.info.resolution;
        double origin_x = current_map_.info.origin.position.x;
        double origin_y = current_map_.info.origin.position.y;

        int i = static_cast<int>((y - origin_y) / resolution);
        int j = static_cast<int>((x - origin_x) / resolution);
        RCLCPP_INFO(this->get_logger(), "Robot cell: (%d, %d)", i, j);
        return {i, j};

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        return {-1, -1};
    }
}

ActionNeighborsType MazeSolver::get_neighbors(
    const std::pair<int, int> &state,
    const std::vector<std::vector<int>> &grid
) {
    ActionNeighborsType result;
    int i = state.first;
    int j = state.second;

    ActionNeighborsType directions = {
        {"UP", {i -1, j}}, {"DOWN", {i + 1, j}},
        {"LEFT", {i, j - 1}}, {"RIGHT", {i, j + 1}}
    };

    for (const auto& [action, neighbor] : directions) {
        auto [ni, nj] = neighbor;
        if (ni >= 0 && ni < static_cast<int>(grid.size()) &&
            nj >= 0 && nj < static_cast<int>(grid[0].size()) &&
            grid[ni][nj] == 0) 
            {
                result.push_back({action, {ni, nj}});
            }
    }
    return result;
}

PathType MazeSolver::reconstruct_path(Agent* node) {
    PathType path;
    while (node) {
        path.push_back(node->state);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::pair<double, double> MazeSolver::grid_to_world(const std::pair<int, int> &cell) {
    double resolution = current_map_.info.resolution;
    double origin_x = current_map_.info.origin.position.x;
    double origin_y = current_map_.info.origin.position.y;

    int i = cell.first;
    int j = cell.second;

    double world_x = origin_x + (j) * resolution;
    double world_y = origin_y + (i) * resolution;

    return std::make_pair(world_x, world_y);
}

std::pair<int, int> MazeSolver::world_to_grid(double x, double y) {
    double resolution = current_map_.info.resolution;
    double origin_x = current_map_.info.origin.position.x;
    double origin_y = current_map_.info.origin.position.y;

    int i = static_cast<int>((y - origin_y) / resolution);
    int j = static_cast<int>((x - origin_x) / resolution);

    return {i, j};
}

void MazeSolver::path_follow(const PathType &path) {
    rclcpp::Rate rate(10); // 10 Hz

    for(const auto &cell : path) {
        auto [target_x, target_y] = grid_to_world(cell);

        bool target_reached = false;

        double target_theta = 0.0;

        {
            geometry_msgs::msg::TransformStamped tf;

            try {
                tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
                continue;
            }

            double robot_x = tf.transform.translation.x;
            double robot_y = tf.transform.translation.y;
            
            double dx = target_x - robot_x;
            double dy = target_y - robot_y;

            target_theta = std::atan2(dy, dx);
            target_theta = std::atan2(std::sin(target_theta), std::cos(target_theta));
        }

    
        while (rclcpp::ok() && !target_reached) {
            geometry_msgs::msg::TransformStamped tf;
            try {
                tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
                continue;
            }
        
            double robot_x = tf.transform.translation.x;
            double robot_y = tf.transform.translation.y;

            double dx = target_x - robot_x;
            double dy = target_y - robot_y;
            double distance_error = std::hypot(dx, dy);

            tf2::Quaternion q;
            tf2::fromMsg(tf.transform.rotation, q);
            double robot_theta = tf2::getYaw(q);

            double angle_error = target_theta - robot_theta;
            angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));
            
            double d_distance = (distance_error - last_distance_e) / dt;
            double d_angle = (angle_error - last_angle_e) / dt;
            last_distance_e = distance_error;
            last_angle_e = angle_error;

            geometry_msgs::msg::Twist cmd;

            double Kp_lin = 0.4, Kd_lin = 0.03;
            double Kp_ang = 0.098, Kd_ang = 0.01;

            double linear_vel = Kp_lin * distance_error + Kd_lin * d_distance;
            double angular_vel = Kp_ang * angle_error + Kd_ang * d_angle;
            RCLCPP_INFO(this->get_logger(), "Robot theta: %.2f, Target theta: %.2f, Angle error: %.2f",
                        robot_theta, target_theta, angle_error);
            if (std::abs(angle_error) > 0.25) {
                cmd.angular.z = std::clamp(angular_vel, -0.2, 0.2);
                cmd.linear.x = 0.0;
            } else if (distance_error > 0.05) {
                cmd.linear.x = std::clamp(linear_vel, 0.0, 0.2);
                cmd.angular.z = std::clamp(angular_vel, -0.1, 0.1);
            } else {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                target_reached = true;
                RCLCPP_INFO(this->get_logger(), "Target cell reached: (%.2f, %.2f)", target_x, target_y);
            }
            cmd_vel_pub_->publish(cmd);
            rate.sleep();
        }
    }

    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
}

void MazeSolver::publish_path(const PathType &path) {
    nav_msgs::msg::Path ros_path;
    ros_path.header.stamp = this->now();
    ros_path.header.frame_id = "map";  // Match the map frame

    for (const auto &cell : path) {
        auto [x, y] = grid_to_world(cell);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = ros_path.header.stamp;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;  // Neutral rotation

        ros_path.poses.push_back(pose);
    }

    path_pub_->publish(ros_path);
    RCLCPP_INFO(this->get_logger(), "Published path with %lu points", ros_path.poses.size());
}

void MazeSolver::run_solver() {
    if (!map_received_) return;
    RCLCPP_INFO(this->get_logger(), "Running solver...");

    std::vector<Agent *> allocated_agents;
    auto [start_i, start_j] = get_robot_cell();
    std::pair<int, int> start = {start_i, start_j};

    double goal_x_meters = 2.0;
    double goal_y_meters = 3.0;
    std::pair<int, int> goal = world_to_grid(goal_x_meters, goal_y_meters);
    RCLCPP_INFO(this->get_logger(), "Start cell: (%d, %d), Goal cell: (%d, %d)", start_i, start_j, goal.first, goal.second);
    int width = current_map_.info.width;
    int height = current_map_.info.height;

    std::vector<std::vector<int>> grid(height, std::vector<int>(width, 1));

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            grid[i][j] = (current_map_.data[i * width + j] == 0 ? 0 : 1);
        }
    }

    // Create a queue frontier for breath-first search
    QueueFrontier frontier;
    Agent* start_agent = new Agent(start);
    allocated_agents.push_back(start_agent);
    frontier.add(start_agent);
    std::set<std::pair<int, int>> explored;
    Agent* solution = nullptr;
    int count = 0;
    while(!frontier.empty()) {
        count++;
        Agent* current = frontier.remove();
        if(current->state == goal) {
            solution = current;
            break;
        }

        explored.insert(current->state);
        for (const auto &[action, neighbor] : get_neighbors(current->state, grid)) {
            if(!frontier.contains_state(neighbor) && explored.count(neighbor) == 0) {
                Agent* new_agent = new Agent(neighbor, current, action);
                allocated_agents.push_back(new_agent);
                frontier.add(new_agent);
            }
        }
    }

    if (solution) {
        RCLCPP_INFO(this->get_logger(), "Solution Found!");
        RCLCPP_INFO(this->get_logger(), "Total iterations: %d", count);
        PathType path = reconstruct_path(solution);
        publish_path(path);
        RCLCPP_INFO(this->get_logger(), "Reached goal!");
    } else {
        RCLCPP_WARN(this->get_logger(), "No solution found");
    }

    for(Agent* agent : allocated_agents) {
        delete agent;
    }
}