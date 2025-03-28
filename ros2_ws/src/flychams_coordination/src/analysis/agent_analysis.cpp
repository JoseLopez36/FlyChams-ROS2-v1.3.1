#include "flychams_coordination/analysis/agent_analysis.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAnalysis::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        update_rate_ = RosUtils::getParameterOr<float>(node_, "agent_analysis.analysis_rate", 20.0f);

        // Initialize data
        clusters_.clear();
        agents_.clear();

        // Set update timer
        update_timer_ = RosUtils::createTimer(node_, update_rate_,
            std::bind(&AgentAnalysis::update, this), module_cb_group_);
    }

    void AgentAnalysis::onShutdown()
    {
        // Destroy clusters and agents
        clusters_.clear();
        agents_.clear();
        // Destroy update timer
        update_timer_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Public methods for adding/removing clusters and agents
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAnalysis::addAgent(const ID& agent_id)
    {
        // Create and add agent
        agents_.insert({ agent_id, Agent() });

        // Initialize clusters message
        agents_[agent_id].clusters.header = RosUtils::createHeader(node_, transform_tools_->getGlobalFrame());
        agents_[agent_id].clusters.cluster_ids.clear();
        agents_[agent_id].clusters.centers.clear();
        agents_[agent_id].clusters.radii.clear();

        // Create agent status subscriber
        agents_[agent_id].status_sub = topic_tools_->createAgentStatusSubscriber(agent_id,
            [this, agent_id](const AgentStatusMsg::SharedPtr msg)
            {
                this->agentStatusCallback(agent_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create agent clusters publisher
        agents_[agent_id].clusters_pub = topic_tools_->createAgentClustersPublisher(agent_id);
    }

    void AgentAnalysis::removeAgent(const ID& agent_id)
    {
        // Remove agent from map
        agents_.erase(agent_id);
    }

    void AgentAnalysis::addCluster(const ID& cluster_id)
    {
        // Create and add cluster
        clusters_.insert({ cluster_id, Cluster() });

        // Create cluster geometry subscriber
        clusters_[cluster_id].geometry_sub = topic_tools_->createClusterGeometrySubscriber(cluster_id,
            [this, cluster_id](const ClusterGeometryMsg::SharedPtr msg)
            {
                this->clusterGeometryCallback(cluster_id, msg);
            }, sub_options_with_module_cb_group_);

        // Create cluster assignment subscriber
        clusters_[cluster_id].assignment_sub = topic_tools_->createClusterAssignmentSubscriber(cluster_id,
            [this, cluster_id](const StringMsg::SharedPtr msg)
            {
                this->clusterAssignmentCallback(cluster_id, msg);
            }, sub_options_with_module_cb_group_);
    }

    void AgentAnalysis::removeCluster(const ID& cluster_id)
    {
        // Remove cluster from map
        clusters_.erase(cluster_id);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAnalysis::clusterGeometryCallback(const ID& cluster_id, const ClusterGeometryMsg::SharedPtr msg)
    {
        // Update cluster geometry
        clusters_[cluster_id].center = msg->center;
        clusters_[cluster_id].radius = msg->radius;
        clusters_[cluster_id].has_geometry = true;
    }

    void AgentAnalysis::clusterAssignmentCallback(const ID& cluster_id, const StringMsg::SharedPtr msg)
    {
        // Update cluster assignment
        clusters_[cluster_id].assignment = msg->data;
        clusters_[cluster_id].has_assignment = true;
    }

    void AgentAnalysis::agentStatusCallback(const ID& agent_id, const AgentStatusMsg::SharedPtr msg)
    {
        // Update agent status
        agents_[agent_id].status = static_cast<AgentStatus>(msg->status);
        agents_[agent_id].has_status = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update analysis
    // ════════════════════════════════════════════════════════════════════════════

    void AgentAnalysis::update()
    {
        // Check if we have a valid agent status, cluster geometries and assignments
        for (const auto& [agent_id, agent] : agents_)
        {
            if (!agent.has_status)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent analysis: Agent %s has no status", agent_id.c_str());
                return; // Skip updating if we don't have a valid agent status
            }

            // Check if we are in the correct state to analyze
            if (agent.status != AgentStatus::TRACKING)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent analysis: Agent %s is not in the correct state to analyze",
                    agent_id.c_str());
                return;
            }
        }
        for (const auto& [cluster_id, cluster] : clusters_)
        {
            if (!cluster.has_geometry || !cluster.has_assignment)
            {
                RCLCPP_WARN(node_->get_logger(), "Agent analysis: Cluster %s has no geometry or assignment", cluster_id.c_str());
                return; // Skip updating if we don't have a valid cluster geometry or assignment
            }
        }

        // Create a map to associate agents with their clusters
        std::unordered_map<core::ID, std::unordered_set<core::ID>> assigned_clusters;
        for (const auto& [cluster_id, cluster] : clusters_)
        {
            assigned_clusters[cluster.assignment].insert(cluster_id);
        }

        // Process each agent
        for (auto& [agent_id, agent] : agents_)
        {
            // Get clusters for this agent
            const auto& new_clusters = assigned_clusters[agent_id];
            if (new_clusters.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "Agent analysis: Agent %s has no clusters", agent_id.c_str());
                continue;
            }

            // Order new clusters consistently
            agent.clusters.cluster_ids = ensureOrder(agent.clusters.cluster_ids, new_clusters);

            // Set centers and radii size if not already set
            if (agent.clusters.centers.size() != agent.clusters.cluster_ids.size())
            {
                agent.clusters.centers.resize(agent.clusters.cluster_ids.size());
                agent.clusters.radii.resize(agent.clusters.cluster_ids.size());
            }

            // Update cluster geometries
            for (size_t i = 0; i < agent.clusters.cluster_ids.size(); i++)
            {
                const auto& cluster_id = agent.clusters.cluster_ids[i];
                const auto& cluster = clusters_[cluster_id];
                agent.clusters.centers[i] = cluster.center;
                agent.clusters.radii[i] = cluster.radius;
            }

            // Publish agent clusters
            agent.clusters.header.stamp = RosUtils::now(node_);
            agent.clusters_pub->publish(agent.clusters);
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Extra methods
    // ════════════════════════════════════════════════════════════════════════════

    std::vector<core::ID> AgentAnalysis::ensureOrder(const std::vector<core::ID>& previous_clusters, const std::unordered_set<core::ID>& new_clusters)
    {
        // Ensures ID order from one iteration to the next
        // The process is as follows:
        // 1. Keep all previous clusters that are still present
        // 2. Add new clusters in the vacant positions and if there are no vacant positions, push them back

        // Result vector that will maintain the ordering
        std::vector<core::ID> ordered_clusters;
        ordered_clusters.resize(new_clusters.size());

        // Create a set to track which new clusters have been added
        std::unordered_set<core::ID> added_clusters;

        // First, keep all previous clusters that are still present
        std::vector<int> vacant_indices;
        for (size_t i = 0; i < previous_clusters.size(); i++)
        {
            const auto& cluster_id = previous_clusters[i];
            if (new_clusters.find(cluster_id) != new_clusters.end()) {
                ordered_clusters[i] = cluster_id;
                added_clusters.insert(cluster_id);
            }
            else {
                vacant_indices.push_back(i);
            }
        }

        // Second, add new clusters in the vacant positions or if there are no vacant positions, push them back
        for (const auto& cluster_id : new_clusters)
        {
            bool not_added = added_clusters.find(cluster_id) == added_clusters.end();
            bool vacant = !vacant_indices.empty();

            // If the cluster has not been added, add it to the vacant position
            if (not_added && vacant) {
                ordered_clusters[vacant_indices.back()] = cluster_id;
                vacant_indices.pop_back();
            }
            else if (not_added) {
                ordered_clusters.push_back(cluster_id);
            }
        }

        return ordered_clusters;
    }

} // namespace flychams::coordination