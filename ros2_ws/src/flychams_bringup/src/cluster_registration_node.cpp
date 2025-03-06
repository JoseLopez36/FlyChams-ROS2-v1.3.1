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
class ClusterRegistrationNode : public RegistratorNode
{
public: // Constructor/Destructor
    ClusterRegistrationNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : RegistratorNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Cycle through all agents in the config
        for (const auto& [_, config_ptr] : config_tools_->getAgents())
        {
            // Extract macimum number of assignments per agent
            const int max_assign = config_ptr->max_assignments;

            // Cycle through all assignments and register clusters
            for (int i = 0; i < max_assign; i++)
            {
                // Generate cluster ID
                std::stringstream ss;
                ss << "CLUSTER" << std::setw(2) << std::setfill('0') << i;
                const ID cluster_id = ss.str();

                // Register cluster
                registerCluster(cluster_id);
            }
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
    auto node = std::make_shared<ClusterRegistrationNode>("cluster_registration_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}