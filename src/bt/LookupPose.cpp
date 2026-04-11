#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"

#include <map>
#include <string>
#include <stdexcept>

struct Pose2D {
    double x;
    double y;
};

class LookupPose : public BT::SyncActionNode {
public:
    LookupPose(const std::string& name, const BT::NodeConfig& config)
        : SyncActionNode(name, config)
    {
        // TODO: Naplňte tabulku souřadnic pose_table_.
        // Manipulátory: ID "1", "2", "3" (viz tabulka v zadání).
        // Sklady: ID "A1", "A2", "B1", "B2", "C1", "C2", "D1", "D2" (viz tabulka v zadání).
        // Příklad: pose_table_["1"] = { 4.5, 1.5 };
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("location_id", "Location ID to look up (e.g. '1', 'A1')"),
            BT::OutputPort<double>("x", "Looked up X coordinate"),
            BT::OutputPort<double>("y", "Looked up Y coordinate")
        };
    }

    BT::NodeStatus tick() override
    {
        // TODO: Načtěte location_id z input portu pomocí getInput<std::string>.
        // Vyhledejte ID v pose_table_. Pokud neexistuje, vraťte FAILURE.
        // Zapište souřadnice do output portů "x" a "y" pomocí setOutput().
        // Vraťte SUCCESS.
        return BT::NodeStatus::FAILURE;
    }

private:
    std::map<std::string, Pose2D> pose_table_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<LookupPose>("LookupPose");
}
