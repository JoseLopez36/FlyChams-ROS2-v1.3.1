#include "rclcpp/rclcpp.hpp"

// Control includes
#include "flychams_control/head_control/head_controller.hpp"

// Core includes
#include "flychams_core/base/discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::control;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Head control node for controlling the different heads
 * in the simulation
 *
 * @details
 * This class implements the head control node for controlling the
 * different heads in the simulation. It uses the discoverer node to
 * discover the different heads and then creates a controller for each
 * head discovered.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class HeadControlNode : public DiscovererNode
{
public: // Constructor/Destructor
    HeadControlNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : DiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize head controllers
        head_controllers_.clear();
    }

    void onShutdown() override
    {
        // Destroy head controllers
        head_controllers_.clear();
    }

private: // Element management
    void onAddAgent(const ID& agent_id) override
    {
        // Create head controller
        auto controller = std::make_shared<HeadController>(agent_id, node_, config_tools_, ext_tools_, topic_tools_, tf_tools_);
        head_controllers_.insert(std::make_pair(agent_id, controller));

        RCLCPP_INFO(node_->get_logger(), "Head controller created for agent %s", agent_id.c_str());
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Destroy head controller
        head_controllers_.erase(agent_id);
    }

private: // Components
    // Head controllers
    std::unordered_map<ID, HeadController::SharedPtr> head_controllers_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create and initialize node
    auto node = std::make_shared<HeadControlNode>("head_control_node", options);
    node->init();
    // Create executor and add node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // Spin node
    executor.spin();
    // Shutdown
    rclcpp::shutdown();
    return 0;
}