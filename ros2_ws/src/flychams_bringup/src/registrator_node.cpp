#include "rclcpp/rclcpp.hpp"

// Standard includes
#include <iostream>
#include <cstdlib>

// Core includes
#include "flychams_core/base/base_registrator_node.hpp"

using namespace flychams::core;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Registrator node for registering the different elements
 * in the simulation
 *
 * @details
 * This class implements the registrator node for registering the
 * different elements in the simulation. It bases on the registrator node
 * to register the different elements, so that they can be discovered by
 * the different nodes.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class RegistratorNode : public BaseRegistratorNode
{
public: // Constructor/Destructor
    RegistratorNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : BaseRegistratorNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Register agents
        for (const auto& [agent_id, agent_config] : config_tools_->getAgents())
        {
            registerElement(agent_id, ElementType::Agent);
        }

        // Register targets
        for (const auto& [group_id, group_config] : config_tools_->getGroups())
        {
            // Extract number of targets in group
            const int num_targets = group_config->target_count;

            // Cycle through all targets in group and register them
            for (int i = 0; i < num_targets; i++)
            {
                // Generate target ID
                std::stringstream ss;
                ss << group_id << "TARGET" << std::setw(2) << std::setfill('0') << i;
                const ID target_id = ss.str();

                // Register target
                registerElement(target_id, ElementType::Target);
            }
        }

        // Register clusters
        for (const auto& [agent_id, agent_config] : config_tools_->getAgents())
        {
            // Extract maximum number of assignments per agent
            const int max_assign = agent_config->max_assignments;

            // Cycle through all assignments and register clusters
            for (int i = 0; i < max_assign; i++)
            {
                // Generate cluster ID
                std::stringstream ss;
                ss << "CLUSTER" << std::setw(2) << std::setfill('0') << i;
                const ID cluster_id = ss.str();

                // Register cluster
                registerElement(cluster_id, ElementType::Cluster);
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
    auto node = std::make_shared<RegistratorNode>("registrator_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}