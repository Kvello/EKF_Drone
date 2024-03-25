#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ee4308_drone/behavior.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    double initial_x = std::stod(argv[1]);
    double initial_y = std::stod(argv[2]);
    double initial_z = std::stod(argv[3]);

    // mapper node
    auto node = std::make_shared<ee4308::drone::ROSNodeBehavior>(
        initial_x, initial_y, initial_z,
        "behavior" // node name
    );

    exe.add_node(node);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}