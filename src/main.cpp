#include "rclcpp/rclcpp.hpp"
#include "turtlebot_test/maze_solver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeSolver>());
    rclcpp::shutdown();
    return 0;
}

