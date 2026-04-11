#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

// TODO: Vytvořte třídu BTServer odvozenou z BT::TreeExecutionServer.
// Konstruktor přijímá const rclcpp::NodeOptions& a předá je rodičovské třídě.
// Volitelně přepište metodu onTreeCreated() pro přidání loggeru (BT::StdCoutLogger).

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // TODO: Vytvořte instanci BTServer s výchozími NodeOptions.
    // Vytvořte MultiThreadedExecutor, přidejte do něj server->node() a zavolejte spin().

    rclcpp::shutdown();
    return 0;
}
