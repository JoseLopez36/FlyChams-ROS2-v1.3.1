#include "rclcpp/rclcpp.hpp"

// Core includes
#include "flychams_core/base/registrator_node.hpp"

using namespace flychams::core;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Agent registration node for registering the different agents
 * in the simulation
 *
 * @details
 * This class implements the agent registration node for registering the
 * different agents in the simulation. It uses the registrator node to
 * register the different agents, so that they can be discovered by the
 * different nodes.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class AgentRegistrationNode : public RegistratorNode
{
public: // Constructor/Destructor
    AgentRegistrationNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : RegistratorNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Cycle through all agents in the config and register them
        for (const auto& [id, _] : config_tools_->getAgents())
        {
            // Register agent
            registerAgent(id);
        }
    }

    void onShutdown() override
    {
        // Nothing to do
    }
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
    auto node = std::make_shared<AgentRegistrationNode>("agent_registration_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}