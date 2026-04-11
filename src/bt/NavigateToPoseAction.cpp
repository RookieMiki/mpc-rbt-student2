#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavigateToPoseAction : public BT::RosActionNode<NavigateToPose> {
public:
    NavigateToPoseAction(const std::string& name, const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params)
        : RosActionNode<NavigateToPose>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<double>("x", "Target position X"),
            BT::InputPort<double>("y", "Target position Y")
        });
    }

    bool setGoal(Goal& goal) override
    {
        // TODO: Načtěte souřadnice x a y z input portů (getInput<double>).
        // Naplňte goal.pose: nastavte header.frame_id na "map",
        // pozici na načtené souřadnice a orientaci (w=1.0).
        // Vraťte false pokud porty nejsou dostupné, jinak true.
        return false;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override
    {
        // TODO: Zkontrolujte wr.code. Pokud je SUCCEEDED, vraťte SUCCESS, jinak FAILURE.
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
    {
        // TODO: Zalogujte chybu a vraťte FAILURE.
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*feedback*/) override
    {
        // TODO: Vraťte RUNNING (akce stále probíhá).
        return BT::NodeStatus::RUNNING;
    }
};

CreateRosNodePlugin(NavigateToPoseAction, "NavigateToPoseAction");
