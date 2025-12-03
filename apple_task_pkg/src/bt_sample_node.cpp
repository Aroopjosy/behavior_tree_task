#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
// REMOVED: #include <ament_index_cpp/exceptions.hpp> (This file is often missing in ROS 2 installs)
#include <behaviortree_cpp/bt_factory.h> 
#include <behaviortree_cpp/action_node.h> 
#include <iostream>
#include <stdexcept> // ADDED: To catch potential runtime_error

using namespace BT;

// --- Base Class for Simple Actions (always succeeds) ---
class BaseDummyAction : public SyncActionNode
{
public:
    // FIX: The base class constructor MUST accept the name and pass it 
    // to the parent SyncActionNode constructor.
    BaseDummyAction(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config) {}

    NodeStatus tick() override
    {
        // Use the friendly name from the XML for logging
        RCLCPP_INFO(rclcpp::get_logger("BT_Executor"), "ACTION Executing: %s", name().c_str());
        return NodeStatus::SUCCESS;
    }
    static PortsList providedPorts() { return {}; }
};

// --- Custom Action Implementations ---
// FIX: Derived constructors must accept name and config and pass them up to the base class. 
// Since they do nothing else, they are simple pass-throughs.

class OpenDoorAction : public BaseDummyAction { 
public: 
    OpenDoorAction(const std::string& name, const NodeConfig& config) 
        : BaseDummyAction(name, config) {}
};

class PickAppleAction : public BaseDummyAction { 
public: 
    PickAppleAction(const std::string& name, const NodeConfig& config) 
        : BaseDummyAction(name, config) {}
};

class CloseDoorAction : public BaseDummyAction { 
public: 
    CloseDoorAction(const std::string& name, const NodeConfig& config) 
        : BaseDummyAction(name, config) {}
};


// --- Main Execution Loop (using BT.v4 API) ---

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_executor");
    
    // IMPORTANT: Point to the new simple XML file
    // NOTE: Using "apple_fetch_bt" as the package name, adjust if you renamed your package.
    std::string xml_file;
    try {
        xml_file = ament_index_cpp::get_package_share_directory("apple_task_pkg") + 
                   "/bt_xml/sample_task.xml";
    // FIX: Catching std::runtime_error which is the base for PackageNotFoundError, 
    // or std::exception, which is a catch-all.
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error finding package share directory: %s", e.what());
        RCLCPP_ERROR(node->get_logger(), "Ensure package 'apple_task_pkg' is built and sourced.");
        rclcpp::shutdown();
        return 1;
    }


    try {
        // 1. Create factory
        BehaviorTreeFactory factory;
        
        // 2. Register only the nodes used in the simple tree
        factory.registerNodeType<OpenDoorAction>("OpenDoorAction");
        factory.registerNodeType<PickAppleAction>("PickAppleAction");
        factory.registerNodeType<CloseDoorAction>("CloseDoorAction");

        RCLCPP_INFO(node->get_logger(), "Behavior Tree Factory initialized and nodes registered.");
        
        // 3. Instantiate the tree
        auto tree = factory.createTreeFromFile(xml_file);
        RCLCPP_INFO(node->get_logger(), "Behavior Tree loaded from XML: %s", xml_file.c_str());

        RCLCPP_INFO(node->get_logger(), "\n--- Starting Simple Behavior Tree Execution ---\n");

        // 4. Tick the tree
        NodeStatus status = tree.tickOnce();

        RCLCPP_INFO(node->get_logger(), "\n--- Behavior Tree Execution Finished ---\n");
        RCLCPP_INFO(node->get_logger(), "Final Status: %s", BT::toStr(status).c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception during BT execution: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}