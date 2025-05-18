#include "turtlebot_test/maze_solver.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <set>
#include <chrono>

MazeSolver::MazeSolver() : Node("maze_solver") {
    // Subscribe to map topic in order to get map info like origin, grid, resolution, etc
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos, std::bind(&MazeSolver::map_callback, this, std::placeholders::_1)
    );

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
    RCLCPP_INFO(this->get_logger(), "Received map, width: %d, height: %d", msg->info.width, msg->info.height);
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

    double world_x = origin_x + j * resolution;
    double world_y = origin_y + i * resolution;

    return std::make_pair(world_x, world_y);
}

void MazeSolver::publish_cmd_vel(const std::pair<int, int> & target_cell) {
    auto [goal_x, goal_y] = grid_to_world(target_cell);

    geometry_msgs::msg::TransformStamped tf;

    try {
        tf = tf_buffer_-> lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        return;
    }

    double robot_x = tf.transform.translation.x;
    double robot_y = tf.transform.translation.y;

    double dx = goal_x - robot_x;
    double dy = goal_y - robot_y;

    double distance = std::hypot(dx, dy);   // Distance to the goal
    double angle = std::atan2(dy, dx);      // Angle to the goal

    geometry_msgs::msg::Twist cmd;

    double p_linear = 0.5; // Proportional gain
    double p_angular = 1.0; // Proportional gain

    cmd.linear.x = std::min(0.3, p_linear * distance); // Max speed
    cmd.angular.z = std::min(0.3 ,p_angular * angle);                  // Rotate towards the goal

    cmd_vel_pub_->publish(cmd);
}

void MazeSolver::run_solver() {
    if (!map_received_) return;
    RCLCPP_INFO(this->get_logger(), "Running solver...");

    auto [start_i, start_j] = get_robot_cell();
    std::pair<int, int> start = {start_i, start_j};
    std::pair<int, int> goal = {8.0, 9.0};

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
    frontier.add(new Agent(start));
    std::set<std::pair<int, int>> explored;
    Agent* solution = nullptr;

    while(!frontier.empty()) {
        Agent* current = frontier.remove();
        if(current->state == goal) {
            solution = current;
            break;
        }

        explored.insert(current->state);
        for (const auto &[action, neighbor] : get_neighbors(current->state, grid)) {
            if(!frontier.contains_state(neighbor) && explored.count(neighbor)) {
                frontier.add(new Agent(neighbor, current, action));
            }
        }
    }

    if (solution) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        auto path = reconstruct_path(solution);
        for (const auto &cell : path) {
            publish_cmd_vel(cell);
            rclcpp::sleep_for(500ms);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "No solution found");
    }
}