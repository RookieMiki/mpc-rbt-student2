#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <string>
#include <unordered_map>

// Jednoduchá struktura pro uchování souřadnic
struct Pose2D {
    double x;
    double y;
};

class LookupPose : public BT::SyncActionNode {
public:
    LookupPose(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        // Tabulka souřadnic podle zadání
        pose_table_ = {
            {"1",  {4.5,  1.5}},
            {"2",  {4.5, -0.5}},
            {"3",  {4.5, -2.5}},
            {"A1", {1.5,  0.5}},  // 1.5
            {"A2", {1.5, -1.5}},
            {"B1", {-0.5, 0.5}},
            {"B2", {-0.5, -1.5}},
            {"C1", {-2.5, 0.5}},
            {"C2", {-2.5, -1.5}},
            {"D1", {-4.5, 0.5}},
            {"D2", {-4.5, -1.5}}
        };
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("location_id", "Manipulator or Storage ID"),
            BT::OutputPort<double>("x", "Target X coordinate"),
            BT::OutputPort<double>("y", "Target Y coordinate")
        };
    }

    BT::NodeStatus tick() override
    {
        auto id = getInput<std::string>("location_id");
        if (!id) {
            return BT::NodeStatus::FAILURE;
        }

        auto it = pose_table_.find(id.value());
        if (it == pose_table_.end()) {
            // ID nebylo v tabulce nalezeno
            return BT::NodeStatus::FAILURE;
        }

        // Zápis souřadnic do output portů
        setOutput("x", it->second.x);
        setOutput("y", it->second.y);

        return BT::NodeStatus::SUCCESS;
    }

private:
    std::unordered_map<std::string, Pose2D> pose_table_;
};

// Registrace (jiný způsob než u ROS uzlů)
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<LookupPose>("LookupPose");
}
