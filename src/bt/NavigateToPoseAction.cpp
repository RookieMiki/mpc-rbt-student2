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
        // Načtení souřadnic x a y z input portů
        auto x_res = getInput<double>("x");
        auto y_res = getInput<double>("y");

        // Vrátíme false, pokud porty nejsou dostupné
        if (!x_res || !y_res) {
            RCLCPP_ERROR(logger(), "Missing 'x' or 'y' in input ports");
            return false;
        }

        // Naplnění goal.pose
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node_.lock()->now();
        goal.pose.pose.position.x = x_res.value();
        goal.pose.pose.position.y = y_res.value();
        goal.pose.pose.orientation.w = 1.0;


        RCLCPP_INFO(logger(), "NavigateToPoseAction: Sending goal [%.2f, %.2f]",
                    x_res.value(), y_res.value());
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override
    {
        // Kontrola výsledku akce
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
    {
        // Zalogování chyby a vrácení FAILURE
        RCLCPP_ERROR(logger(), "NavigateToPoseAction failed with error code: %d", static_cast<int>(error));
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*feedback*/) override
    {
        // Akce stále probíhá
        return BT::NodeStatus::RUNNING;
    }
};

CreateRosNodePlugin(NavigateToPoseAction, "NavigateToPoseAction");
