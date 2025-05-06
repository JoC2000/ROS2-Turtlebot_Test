#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MapSub : public rclcpp::Node {
public:
    MapSub() : Node("map_subscriber") {
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapSub::map_callback, this, std::placeholders::_1)
        );
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Node started");
        RCLCPP_INFO(this->get_logger(), "map resolution: %f", msg->info.resolution);
        // RCLCPP_INFO(this->get_logger(), "Received map: width = %d, height = %d", msg->info.width, msg->info.height);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSub>());
    rclcpp::shutdown();
    return 0;
}
