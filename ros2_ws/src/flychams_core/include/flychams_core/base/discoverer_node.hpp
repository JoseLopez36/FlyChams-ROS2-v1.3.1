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
     * @brief Discoverer node for discovering the different elements
     * in the simulation
     *
     * @details
     * This class implements the discoverer node for discovering agents,
     * targets, and clusters with the help of the various tools. It serves
     * as a base class for the different nodes that need to discover elements
     * dynamically in the simulation.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class DiscovererNode : public rclcpp::Node
    {
    public: // Constructor/Destructor
        DiscovererNode(const std::string& node_name, const rclcpp::NodeOptions& options);
        void init();
        virtual ~DiscovererNode();
        void shutdown();

    public: // Types
        using SharedPtr = std::shared_ptr<DiscovererNode>;

    protected: // Overridable methods
        virtual void onInit() {};
        virtual void onShutdown() {};
        virtual void onAddAgent(const ID& agent_id) {};
        virtual void onRemoveAgent(const ID& agent_id) {};
        virtual void onAddTarget(const ID& target_id) {};
        virtual void onRemoveTarget(const ID& target_id) {};
        virtual void onAddCluster(const ID& cluster_id) {};
        virtual void onRemoveCluster(const ID& cluster_id) {};

    private: // Discovery callbacks
        void onAgentDiscovery(const RegistrationMsg::SharedPtr msg);
        void onTargetDiscovery(const RegistrationMsg::SharedPtr msg);
        void onClusterDiscovery(const RegistrationMsg::SharedPtr msg);

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
        std::unordered_set<ID> discovered_agents_;
        std::unordered_set<ID> discovered_targets_;
        std::unordered_set<ID> discovered_clusters_;
        // Discovery subscribers
        SubscriberPtr<RegistrationMsg> agent_discovery_sub_;
        SubscriberPtr<RegistrationMsg> target_discovery_sub_;
        SubscriberPtr<RegistrationMsg> cluster_discovery_sub_;
    };

} // namespace flychams::core