#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"

class BTServer : public BT::TreeExecutionServer {
public:
    BTServer(const rclcpp::NodeOptions& options)
        : TreeExecutionServer(options) {}

    void onTreeCreated(BT::Tree& /*tree*/) override {
        // Volitelné zalogování, že se strom úspěšně vytvořil a načetl
        RCLCPP_INFO(node()->get_logger(), "Behavior Tree loaded and created successfully.");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    
    auto server = std::make_shared<BTServer>(options);
    
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(server->node());
    exec.spin();
    
    rclcpp::shutdown();
    return 0;
}
