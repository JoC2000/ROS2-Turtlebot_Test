#include "rclcpp/rclcpp.hpp"
#include "turtlebot_test/maze_solver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true);
    rclcpp::spin(std::make_shared<MazeSolver>(options));
    rclcpp::shutdown();
    return 0;
}

