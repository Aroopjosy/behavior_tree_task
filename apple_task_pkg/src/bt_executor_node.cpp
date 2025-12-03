#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h> // Correct header for ConditionNode
#include <iostream>
#include <stdexcept>

using namespace BT;

// --- Base Class for Simple Actions (always succeeds) ---
class BaseDummyAction : public SyncActionNode
{
public:
    // FIX: Use NodeConfig for V4 compatibility and pass name and config to parent
    BaseDummyAction(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config) {}

    NodeStatus tick() override
    {
        RCLCPP_INFO(rclcpp::get_logger("BT_Executor"), "ACTION Executing: %s", name().c_str());
        return NodeStatus::SUCCESS; // Action succeeds
    }
    static PortsList providedPorts() { return {}; }
};

// --- Base Class for Condition Checks (SUCCESS or FAILURE) ---
// FIX: Inherit from ConditionNode, not SyncActionNode. ConditionNode is a SyncActionNode 
// that is structurally simpler and better suited for conditions.
class BaseDummyCondition : public ConditionNode
{
protected:
    // This value will simulate the condition (true = SUCCESS, false = FAILURE)
    bool condition_result_; 

public:
    // FIX: Use NodeConfig for V4 compatibility and pass name and config to parent
    BaseDummyCondition(const std::string& name, const NodeConfig& config, bool result_value)
        : ConditionNode(name, config), condition_result_(result_value) {}

    NodeStatus tick() override
    {
        RCLCPP_INFO(rclcpp::get_logger("BT_Executor"), "CONDITION Checking: %s", name().c_str());
        if (condition_result_) {
            RCLCPP_INFO(rclcpp::get_logger("BT_Executor"), "CONDITION Result: SUCCESS (Is open/true)");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("BT_Executor"), "CONDITION Result: FAILURE (Is closed/false)");
            return NodeStatus::FAILURE;
        }
    }
    static PortsList providedPorts() { return {}; }
};

// --- Custom Action Implementations ---
// FIX: Derived constructors must pass name and config to the base class constructor.
class MoveToRoomDoor : public BaseDummyAction { public: MoveToRoomDoor(const std::string& name, const NodeConfig& config) : BaseDummyAction(name, config) {} };
class OpenDoorAction : public BaseDummyAction { public: OpenDoorAction(const std::string& name, const NodeConfig& config) : BaseDummyAction(name, config) {} };
class EnterRoomAction : public BaseDummyAction { public: EnterRoomAction(const std::string& name, const NodeConfig& config) : BaseDummyAction(name, config) {} };
class MoveToFridgeDoor : public BaseDummyAction { public: MoveToFridgeDoor(const std::string& name, const NodeConfig& config) : BaseDummyAction(name, config) {} };
class OpenFridgeDoorAction : public BaseDummyAction { public: OpenFridgeDoorAction(const std::string& name, const NodeConfig& config) : BaseDummyAction(name, config) {} };
class FindAppleAction : public BaseDummyAction { public: FindAppleAction(const std::string& name, const NodeConfig& config) : BaseDummyAction(name, config) {} };
class PickAppleAction : public BaseDummyAction { public: PickAppleAction(const std::string& name, const NodeConfig& config) : BaseDummyAction(name, config) {} };
class CloseFridgeDoorAction : public BaseDummyAction { public: CloseFridgeDoorAction(const std::string& name, const NodeConfig& config) : BaseDummyAction(name, config) {} };
class MoveToRoomDoorForExit : public BaseDummyAction { public: MoveToRoomDoorForExit(const std::string& name, const NodeConfig& config) : BaseDummyAction(name, config) {} };
class ExitRoomAction : public BaseDummyAction { public: ExitRoomAction(const std::string& name, const NodeConfig& config) : BaseDummyAction(name, config) {} };


// --- Custom Condition Implementations (Simulate state) ---

// FIX: Derived constructors must pass name, config, AND the condition result to the base class.
// SIMULATION: Assume room door is initially CLOSED (returns FAILURE) -> pass false
class CheckDoorOpen : public BaseDummyCondition
{ 
public: CheckDoorOpen(const std::string& name, const NodeConfig& config) : BaseDummyCondition(name, config, false) {}
};

// SIMULATION: Assume fridge door is initially OPEN (returns SUCCESS) -> pass true
class CheckFridgeDoorOpen : public BaseDummyCondition
{ 
public: CheckFridgeDoorOpen(const std::string& name, const NodeConfig& config) : BaseDummyCondition(name, config, true) {}
};


// --- Main Execution Loop ---

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_executor");
    
    // Get the path to the installed XML file
    std::string xml_file;
    // NOTE: This assumes your package is named "apple_task_pkg"
    try {
        xml_file = ament_index_cpp::get_package_share_directory("apple_task_pkg") + 
                            "/bt_xml/apple_grab_task.xml";
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error finding package share directory: %s", e.what());
        RCLCPP_ERROR(node->get_logger(), "Ensure package 'apple_task_pkg' is built and sourced.");
        rclcpp::shutdown();
        return 1;
    }

    try {
        // 1. Create a factory and register all custom nodes
        BehaviorTreeFactory factory;
        
        factory.registerNodeType<MoveToRoomDoor>("MoveToRoomDoor");
        factory.registerNodeType<CheckDoorOpen>("CheckDoorOpen");
        factory.registerNodeType<OpenDoorAction>("OpenDoorAction");
        factory.registerNodeType<EnterRoomAction>("EnterRoomAction");
        
        factory.registerNodeType<MoveToFridgeDoor>("MoveToFridgeDoor");
        factory.registerNodeType<CheckFridgeDoorOpen>("CheckFridgeDoorOpen");
        factory.registerNodeType<OpenFridgeDoorAction>("OpenFridgeDoorAction");
        factory.registerNodeType<FindAppleAction>("FindAppleAction");

        factory.registerNodeType<PickAppleAction>("PickAppleAction");
        factory.registerNodeType<CloseFridgeDoorAction>("CloseFridgeDoorAction");
        factory.registerNodeType<MoveToRoomDoorForExit>("MoveToRoomDoorForExit");
        factory.registerNodeType<ExitRoomAction>("ExitRoomAction");

        RCLCPP_INFO(node->get_logger(), "Behavior Tree Factory initialized and nodes registered.");
        
        // 2. Instantiate the tree
        auto tree = factory.createTreeFromFile(xml_file);
        RCLCPP_INFO(node->get_logger(), "Behavior Tree loaded from XML: %s", xml_file.c_str());

        // 3. Tick the tree (using tickOnce for synchronous nodes)
        RCLCPP_INFO(node->get_logger(), "\n--- Starting Behavior Tree Execution ---\n");

        // FIX: Use tickOnce() for synchronous execution
        NodeStatus status = tree.tickOnce();

        RCLCPP_INFO(node->get_logger(), "\n--- Behavior Tree Execution Finished ---\n");
        RCLCPP_INFO(node->get_logger(), "Final Status: %s", BT::toStr(status).c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception during BT execution: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}