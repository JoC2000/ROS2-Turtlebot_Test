#include "turtlebot_test/maze_solver.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <set>
#include <chrono>

using namespace std::chrono_literals;

MazeSolver::MazeSolver() : Node("maze_solver") {
    // Subscribe to map topic in order to get map info like origin, grid, resolution, etc
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&MazeSolver::map_callback, this, std::placeholders::_1)
    );

    // Publisher to the velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // tf buffer and listener to get actual robot pose relative to the map frame
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "MazeSolver node initialized");
}

/**
*  Receives a pointer to OccupancyGrid message
*  Sets the current_map_ to the received pointer and sets the map_received_ flag to true
*  Calls the run_solver function
*/
void MazeSolver::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    map_received_ = true;
    run_solver();
}

std::pair<int, int> MazeSolver::get_robot_cell() {
    try {
        auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;

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

void MazeSolver::run_solver() {
    if (!map_received_) return;

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

std::vector<std::pair<std::string, std::pair<int,int>>> MazeSolver::get_neighbors(
    const std::pair<int, int> &state,
    const std::vector<std::vector<int>> &grid
) {
    std::vector<std::pair<std::string, std::pair<int, int>>> result;
    int i = state.first;
    int j = state.second;

    std::vector<std::pair<std::string, std::pair<int, int>>> directions = {
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

std::vector<std::pair<int, int>> MazeSolver::reconstruct_path(Agent* node) {
    std::vector<std::pair<int, int>> path;
    while (node) {
        path.push_back(node->state);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void MazeSolver::publish_cmd_vel(const std::pair<int, int> &) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.1;
    msg.angular.z = 0.0;
    cmd_vel_pub_->publish(msg);
}