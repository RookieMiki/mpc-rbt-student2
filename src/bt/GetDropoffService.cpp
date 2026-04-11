#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "std_srvs/srv/trigger.hpp"

using Trigger = std_srvs::srv::Trigger;

class GetDropoffService : public BT::RosServiceNode<Trigger> {
public:
    GetDropoffService(const std::string& name, const BT::NodeConfig& conf,
                      const BT::RosNodeParams& params)
        : RosServiceNode<Trigger>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::OutputPort<std::string>("storage_id", "Assigned storage location (A1-D2)")
        });
    }

    bool setRequest(Request::SharedPtr& request) override
    {
        // Trigger nemá žádné pole v requestu
        (void)request;
        return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
        // Kontrola success
        if (!response->success) {
            return BT::NodeStatus::FAILURE;
        }

        // Zápis do output portu
        setOutput("storage_id", response->message);
        
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "GetDropoffService failed: %d", static_cast<int>(error));
        return BT::NodeStatus::FAILURE;
    }
};

CreateRosNodePlugin(GetDropoffService, "GetDropoffService");
