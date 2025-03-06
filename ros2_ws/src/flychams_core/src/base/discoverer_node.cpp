#include "flychams_core/base/discoverer_node.hpp"

namespace flychams::core
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    DiscovererNode::DiscovererNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : Node(node_name, options), node_name_(node_name)
    {
        // Nothing to do
    }

    void DiscovererNode::init()
    {
        // Get node pointer
        node_ = this->shared_from_this();
        RCLCPP_INFO(node_->get_logger(), "Starting %s node...", node_name_.c_str());

        // Create tools
        config_tools_ = std::make_shared<ConfigTools>(node_);
        ext_tools_ = externalToolsFactory(node_, Framework::AirSim);
        topic_tools_ = std::make_shared<TopicTools>(node_);
        tf_tools_ = std::make_shared<TfTools>(node_);

        // Initialize registration subscribers
        // The subscribers activate a callback that adds or removes unique elements
        // from the sets of existing elements.
        // They also activate an overridable callback onAddElement and onRemoveElement,
        // that can be used to react to the addition or removal of an element.
        // Agent discovery
        discovered_agents_.clear();
        agent_discovery_sub_ = topic_tools_->createAgentRegistrationSubscriber(
            std::bind(&DiscovererNode::onAgentDiscovery, this, std::placeholders::_1));
        // Target discovery
        discovered_targets_.clear();
        target_discovery_sub_ = topic_tools_->createTargetRegistrationSubscriber(
            std::bind(&DiscovererNode::onTargetDiscovery, this, std::placeholders::_1));
        // Cluster discovery
        discovered_clusters_.clear();
        cluster_discovery_sub_ = topic_tools_->createClusterRegistrationSubscriber(
            std::bind(&DiscovererNode::onClusterDiscovery, this, std::placeholders::_1));

        // Call on init overridable method
        onInit();
        RCLCPP_INFO(node_->get_logger(), "%s node running", node_name_.c_str());
    }

    DiscovererNode::~DiscovererNode()
    {
        shutdown();
    }

    void DiscovererNode::shutdown()
    {
        RCLCPP_INFO(node_->get_logger(), "Shutting down %s node...", node_name_.c_str());
        // Call on shutdown overridable method
        onShutdown();
        // Destroy existing elements
        discovered_agents_.clear();
        discovered_targets_.clear();
        discovered_clusters_.clear();
        // Destroy discovery subscribers
        agent_discovery_sub_.reset();
        target_discovery_sub_.reset();
        cluster_discovery_sub_.reset();
        // Destroy tools
        config_tools_.reset();
        ext_tools_.reset();
        topic_tools_.reset();
        tf_tools_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // DISCOVERY CALLBACKS: Discovery callbacks for adding or removing elements
    // ════════════════════════════════════════════════════════════════════════════

    void DiscovererNode::onAgentDiscovery(const RegistrationMsg::SharedPtr msg)
    {
        std::unordered_set<ID> new_agents;
        for (const auto& agent_id : msg->element_ids)
        {
            new_agents.insert(agent_id);
        }

        // Find agents to add
        const auto existing_agents = discovered_agents_;
        for (const auto& agent_id : new_agents)
        {
            if (existing_agents.find(agent_id) == existing_agents.end())
            {
                // Add agent to set
                discovered_agents_.insert(agent_id);
                ext_tools_->addVehicle(agent_id);
                RCLCPP_INFO(node_->get_logger(), "Agent %s added", agent_id.c_str());
                onAddAgent(agent_id);
            }
        }

        // Find agents to remove
        for (const auto& agent_id : existing_agents)
        {
            if (new_agents.find(agent_id) == new_agents.end())
            {
                discovered_agents_.erase(agent_id);
                ext_tools_->removeVehicle(agent_id);
                RCLCPP_INFO(node_->get_logger(), "Agent %s removed", agent_id.c_str());
                onRemoveAgent(agent_id);
            }
        }
    }

    void DiscovererNode::onTargetDiscovery(const RegistrationMsg::SharedPtr msg)
    {
        std::unordered_set<ID> new_targets;
        for (const auto& target_id : msg->element_ids)
        {
            new_targets.insert(target_id);
        }

        // Find targets to add
        const auto existing_targets = discovered_targets_;
        for (const auto& target_id : new_targets)
        {
            if (existing_targets.find(target_id) == existing_targets.end())
            {
                // Add target to set
                discovered_targets_.insert(target_id);
                RCLCPP_INFO(node_->get_logger(), "Target %s added", target_id.c_str());
                onAddTarget(target_id);
            }
        }

        // Find targets to remove
        for (const auto& target_id : existing_targets)
        {
            if (new_targets.find(target_id) == new_targets.end())
            {
                discovered_targets_.erase(target_id);
                RCLCPP_INFO(node_->get_logger(), "Target %s removed", target_id.c_str());
                onRemoveTarget(target_id);
            }
        }
    }

    void DiscovererNode::onClusterDiscovery(const RegistrationMsg::SharedPtr msg)
    {
        std::unordered_set<ID> new_clusters;
        for (const auto& cluster_id : msg->element_ids)
        {
            new_clusters.insert(cluster_id);
        }

        // Find clusters to add
        const auto existing_clusters = discovered_clusters_;
        for (const auto& cluster_id : new_clusters)
        {
            if (existing_clusters.find(cluster_id) == existing_clusters.end())
            {
                // Add cluster to set
                discovered_clusters_.insert(cluster_id);
                RCLCPP_INFO(node_->get_logger(), "Cluster %s added", cluster_id.c_str());
                onAddCluster(cluster_id);
            }
        }

        // Find clusters to remove
        for (const auto& cluster_id : existing_clusters)
        {
            if (new_clusters.find(cluster_id) == new_clusters.end())
            {
                discovered_clusters_.erase(cluster_id);
                RCLCPP_INFO(node_->get_logger(), "Cluster %s removed", cluster_id.c_str());
                onRemoveCluster(cluster_id);
            }
        }
    }

} // namespace flychams::core