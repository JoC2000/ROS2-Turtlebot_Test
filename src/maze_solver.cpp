#include "turtlebot_test/maze_solver.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <set>
#include <chrono>

MazeSolver::MazeSolver(const rclcpp::NodeOptions &options) : Node("maze_solver", options) {
    // Set qos policy to transient_local
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    // Use sim_time to avoid issues with tf transform listener
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Subscribe with qos
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos, std::bind(&MazeSolver::map_callback, this, std::placeholders::_1)
    );

    // Path publisher with qos
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", qos);
    
    // tf buffer and listener to get actual robot pose relative to the map frame
    // Using shared pointer in case another node needs to access the same buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
    RCLCPP_INFO(this->get_logger(), "MazeSolver node initialized");
}

void MazeSolver::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
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

std::pair<int, int> MazeSolver::get_robot_cell()
{
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
    const std::vector<std::vector<int>> &grid)
{
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

PathType MazeSolver::reconstruct_path(Agent* node)
{
    PathType path;
    while (node) {
        path.push_back(node->state);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::pair<double, double> MazeSolver::grid_to_world(const std::pair<int, int> &cell) 
{
    double resolution = current_map_.info.resolution;
    double origin_x = current_map_.info.origin.position.x;
    double origin_y = current_map_.info.origin.position.y;

    int i = cell.first;
    int j = cell.second;

    double world_x = origin_x + (j) * resolution;
    double world_y = origin_y + (i) * resolution;

    return std::make_pair(world_x, world_y);
}

std::pair<int, int> MazeSolver::world_to_grid(double x, double y) 
{
    double resolution = current_map_.info.resolution;
    double origin_x = current_map_.info.origin.position.x;
    double origin_y = current_map_.info.origin.position.y;

    int i = static_cast<int>((y - origin_y) / resolution);
    int j = static_cast<int>((x - origin_x) / resolution);

    return {i, j};
}

void MazeSolver::publish_path(const PathType &path) 
{
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

void MazeSolver::run_solver() 
{
    if (!map_received_) return;
    RCLCPP_INFO(this->get_logger(), "Running solver...");

    // Initiate vector of used pointers to free memory when program ends
    std::vector<Agent *> allocated_agents;

    // Initial robot position
    auto [start_i, start_j] = get_robot_cell();
    std::pair<int, int> start = {start_i, start_j};

    // Goal - TODO: make it a parameter for user input
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
    } else {
        RCLCPP_WARN(this->get_logger(), "No solution found");
    }

    // Free pointers used for path construction
    for(Agent* agent : allocated_agents) {
        delete agent;
    }
}