#include "flychams_core/base/registrator_node.hpp"

namespace flychams::core
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    RegistratorNode::RegistratorNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : Node(node_name, options), node_name_(node_name)
    {
        // Nothing to do
    }

    void RegistratorNode::init()
    {
        // Get node pointer
        node_ = this->shared_from_this();
        RCLCPP_INFO(node_->get_logger(), "Starting %s node...", node_name_.c_str());

        // Create tools
        config_tools_ = std::make_shared<ConfigTools>(node_);
        ext_tools_ = externalToolsFactory(node_, Framework::AirSim);
        topic_tools_ = std::make_shared<TopicTools>(node_);
        tf_tools_ = std::make_shared<TfTools>(node_);

        // Initialize registration publishers
        // Agent registration
        registered_agents_.clear();
        agent_registration_pub_ = topic_tools_->createAgentRegistrationPublisher();
        // Target registration
        registered_targets_.clear();
        target_registration_pub_ = topic_tools_->createTargetRegistrationPublisher();
        // Cluster registration
        registered_clusters_.clear();
        cluster_registration_pub_ = topic_tools_->createClusterRegistrationPublisher();

        // Call on init overridable method
        onInit();
        RCLCPP_INFO(node_->get_logger(), "%s node running", node_name_.c_str());
    }

    RegistratorNode::~RegistratorNode()
    {
        shutdown();
    }

    void RegistratorNode::shutdown()
    {
        RCLCPP_INFO(node_->get_logger(), "Shutting down %s node...", node_name_.c_str());
        // Call on shutdown overridable method
        onShutdown();
        // Destroy existing elements
        registered_agents_.clear();
        registered_targets_.clear();
        registered_clusters_.clear();
        // Destroy registration publishers
        agent_registration_pub_.reset();
        target_registration_pub_.reset();
        cluster_registration_pub_.reset();
        // Destroy tools
        config_tools_.reset();
        ext_tools_.reset();
        topic_tools_.reset();
        tf_tools_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // REGISTRATION METHODS: Registration methods for adding or removing elements
    // ════════════════════════════════════════════════════════════════════════════

    void RegistratorNode::onAgentChange()
    {
        // Create and publish agent registration message
        RegistrationMsg msg;
        for (const auto& agent_id : registered_agents_)
        {
            msg.element_ids.push_back(agent_id);
        }
        agent_registration_pub_->publish(msg);
    }

    void RegistratorNode::registerAgent(const ID& agent_id)
    {
        // Add agent to set
        bool success = RosUtils::addToSet(node_, registered_agents_, agent_id);
        if (!success)
            return;
        ext_tools_->addVehicle(agent_id);

        // Call on agent change
        onAgentChange();
        RCLCPP_WARN(node_->get_logger(), "Agent %s registered", agent_id.c_str());
    }

    void RegistratorNode::unregisterAgent(const ID& agent_id)
    {
        // Remove agent from set
        bool success = RosUtils::removeFromSet(node_, registered_agents_, agent_id);
        if (!success)
            return;
        ext_tools_->removeVehicle(agent_id);

        // Call on agent change
        onAgentChange();
        RCLCPP_WARN(node_->get_logger(), "Agent %s unregistered", agent_id.c_str());
    }

    void RegistratorNode::onTargetChange()
    {
        // Create and publish target registration message
        RegistrationMsg msg;
        for (const auto& target_id : registered_targets_)
        {
            msg.element_ids.push_back(target_id);
        }
        target_registration_pub_->publish(msg);
    }

    void RegistratorNode::registerTarget(const ID& target_id)
    {
        // Add target to set
        bool success = RosUtils::addToSet(node_, registered_targets_, target_id);
        if (!success)
            return;

        // Call on target change
        onTargetChange();
        RCLCPP_WARN(node_->get_logger(), "Target %s registered", target_id.c_str());
    }

    void RegistratorNode::unregisterTarget(const ID& target_id)
    {
        // Remove target from set
        bool success = RosUtils::removeFromSet(node_, registered_targets_, target_id);
        if (!success)
            return;

        // Call on target change
        onTargetChange();
        RCLCPP_WARN(node_->get_logger(), "Target %s unregistered", target_id.c_str());
    }

    void RegistratorNode::onClusterChange()
    {
        // Create and publish cluster registration message
        RegistrationMsg msg;
        for (const auto& cluster_id : registered_clusters_)
        {
            msg.element_ids.push_back(cluster_id);
        }
        cluster_registration_pub_->publish(msg);
    }

    void RegistratorNode::registerCluster(const ID& cluster_id)
    {
        // Add cluster to set
        bool success = RosUtils::addToSet(node_, registered_clusters_, cluster_id);
        if (!success)
            return;

        // Call on cluster change
        onClusterChange();
        RCLCPP_WARN(node_->get_logger(), "Cluster %s registered", cluster_id.c_str());
    }

    void RegistratorNode::unregisterCluster(const ID& cluster_id)
    {
        // Remove cluster from set
        bool success = RosUtils::removeFromSet(node_, registered_clusters_, cluster_id);
        if (!success)
            return;

        // Call on cluster change
        onClusterChange();
        RCLCPP_WARN(node_->get_logger(), "Cluster %s unregistered", cluster_id.c_str());
    }

} // namespace flychams::core