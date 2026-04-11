#include "bt/WarehouseManager.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WarehouseManagerNode>());
    rclcpp::shutdown();
    return 0;
}
