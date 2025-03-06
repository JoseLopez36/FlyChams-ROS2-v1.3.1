#pragma once

// Standard includes
#include <unordered_set>

// Tools includes
#include "flychams_core/tools/config_tools.hpp"
#include "flychams_core/tools/external_tools.hpp"
#include "flychams_core/tools/topic_tools.hpp"
#include "flychams_core/tools/tf_tools.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Registrator node for registering the different elements
     * in the simulation
     *
     * @details
     * This class implements the registrator node for registering agents,
     * targets, and clusters with the help of the various tools. It serves
     * as a base class for the different nodes that need to register elements
     * dynamically in the simulation.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class RegistratorNode : public rclcpp::Node
    {
    public: // Constructor/Destructor
        RegistratorNode(const std::string& node_name, const rclcpp::NodeOptions& options);
        void init();
        virtual ~RegistratorNode();
        void shutdown();

    public: // Types
        using SharedPtr = std::shared_ptr<RegistratorNode>;

    protected: // Overridable methods
        virtual void onInit() {};
        virtual void onShutdown() {};

    protected: // Registration methods
        // Agent registration
        void onAgentChange();
        void registerAgent(const ID& agent_id);
        void unregisterAgent(const ID& agent_id);
        // Target registration
        void onTargetChange();
        void registerTarget(const ID& target_id);
        void unregisterTarget(const ID& target_id);
        // Cluster registration
        void onClusterChange();
        void registerCluster(const ID& cluster_id);
        void unregisterCluster(const ID& cluster_id);

    protected: // Components
        // Node
        NodePtr node_;
        const std::string node_name_;
        // Executor
        ExecutorPtr executor_;
        // Tools
        ConfigTools::SharedPtr config_tools_;
        ExternalTools::SharedPtr ext_tools_;
        TopicTools::SharedPtr topic_tools_;
        TfTools::SharedPtr tf_tools_;
        // Existing elements
        std::unordered_set<ID> registered_agents_;
        std::unordered_set<ID> registered_targets_;
        std::unordered_set<ID> registered_clusters_;
        // Registration publishers
        PublisherPtr<RegistrationMsg> agent_registration_pub_;
        PublisherPtr<RegistrationMsg> target_registration_pub_;
        PublisherPtr<RegistrationMsg> cluster_registration_pub_;
    };

} // namespace flychams::core