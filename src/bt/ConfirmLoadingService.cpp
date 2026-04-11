#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "std_srvs/srv/trigger.hpp"

using Trigger = std_srvs::srv::Trigger;

class ConfirmLoadingService : public BT::RosServiceNode<Trigger> {
public:
    ConfirmLoadingService(const std::string& name, const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params)
        : RosServiceNode<Trigger>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    bool setRequest(Request::SharedPtr& request) override
    {
        // Trigger nemá žádné pole v requestu
        (void)request;
        return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
        // TODO: Zkontrolujte response->success. Pokud je false, vraťte FAILURE.
        // Tento node nemá žádný output port – stačí vrátit SUCCESS.
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "ConfirmLoadingService failed: %d", static_cast<int>(error));
        return BT::NodeStatus::FAILURE;
    }
};

CreateRosNodePlugin(ConfirmLoadingService, "ConfirmLoadingService");
