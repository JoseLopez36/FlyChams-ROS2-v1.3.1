#pragma once

// Metrics factory include
#include "flychams_dashboard/visualization/metrics_factory.hpp"

// Markers factory include
#include "flychams_dashboard/visualization/markers_factory.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::dashboard
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Factory for creating and updating visualization components
     *
     * @details
     * This class is responsible for creating and updating various
     * visualization components, including RViz markers and standard
     * metrics.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-27
     * ════════════════════════════════════════════════════════════════
     */
    class VisualizationFactory : public core::BaseModule
    {
    public: // Constructor/Destructor
        VisualizationFactory(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::FrameworkTools::SharedPtr framework_tools, core::TopicTools::SharedPtr topic_tools, core::TransformTools::SharedPtr transform_tools, core::CallbackGroupPtr module_cb_group)
            : BaseModule(node, config_tools, framework_tools, topic_tools, transform_tools, module_cb_group)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<VisualizationFactory>;
        struct Agent
        {
            // Position data
            core::PointMsg position;
            bool has_position;
            // Position setpoint data
            core::PointMsg position_setpoint;
            bool has_position_setpoint;
            // Subscribers
            core::SubscriberPtr<core::PointStampedMsg> position_sub;
            core::SubscriberPtr<core::PointStampedMsg> position_setpoint_sub;
            // Publishers
            core::PublisherPtr<core::AgentMetricsMsg> metrics_pub;
            core::PublisherPtr<core::MarkerArrayMsg> markers_pub;
            // Constructor
            Agent()
                : position(), has_position(false), position_setpoint(), has_position_setpoint(false), position_sub(), position_setpoint_sub(), metrics_pub(), markers_pub()
            {
            }
        };
        struct Target
        {
            // Position data
            core::PointMsg position;
            bool has_position;
            // Subscribers
            core::SubscriberPtr<core::PointStampedMsg> position_sub;
            // Publishers
            core::PublisherPtr<core::TargetMetricsMsg> metrics_pub;
            core::PublisherPtr<core::MarkerArrayMsg> markers_pub;
            // Constructor
            Target()
                : position(), has_position(false), position_sub(), metrics_pub(), markers_pub()
            {
            }
        };
        struct Cluster
        {
            // Geometric data
            core::PointMsg center;
            float radius;
            bool has_geometry;
            // Subscriber
            core::SubscriberPtr<core::ClusterGeometryMsg> geometry_sub;
            // Publishers
            core::PublisherPtr<core::ClusterMetricsMsg> metrics_pub;
            core::PublisherPtr<core::MarkerArrayMsg> markers_pub;
            // Constructor
            Cluster()
                : center(), radius(), has_geometry(false), geometry_sub(), metrics_pub(), markers_pub()
            {
            }
        };


    private: // Parameters
        float metrics_update_rate_;
        float markers_update_rate_;

    private: // Data
        // Element metrics
        std::unordered_map<core::ID, core::AgentMetrics> curr_agent_metrics_;
        std::unordered_map<core::ID, core::AgentMetrics> prev_agent_metrics_;
        std::unordered_map<core::ID, core::TargetMetrics> curr_target_metrics_;
        std::unordered_map<core::ID, core::TargetMetrics> prev_target_metrics_;
        std::unordered_map<core::ID, core::ClusterMetrics> curr_cluster_metrics_;
        std::unordered_map<core::ID, core::ClusterMetrics> prev_cluster_metrics_;
        core::GlobalMetrics curr_global_metrics_;
        core::GlobalMetrics prev_global_metrics_;
        // Element markers
        std::unordered_map<core::ID, core::MarkerArrayMsg> agent_markers_;
        std::unordered_map<core::ID, core::MarkerArrayMsg> target_markers_;
        std::unordered_map<core::ID, core::MarkerArrayMsg> cluster_markers_;
        // Element IDs
        std::unordered_set<core::ID> agent_ids_;
        std::unordered_set<core::ID> target_ids_;
        std::unordered_set<core::ID> cluster_ids_;
        // Time data
        core::Time prev_time_;

    public: // Public methods
        void addAgent(const core::ID& agent_id);
        void removeAgent(const core::ID& agent_id);
        void addTarget(const core::ID& target_id);
        void removeTarget(const core::ID& target_id);
        void addCluster(const core::ID& cluster_id);
        void removeCluster(const core::ID& cluster_id);

    private: // Methods
        // Callbacks
        void agentOdomCallback(const core::ID& agent_id, const core::OdometryMsg::SharedPtr msg);
        void agentGoalCallback(const core::ID& agent_id, const core::PositionGoalMsg::SharedPtr msg);
        void targetInfoCallback(const core::ID& target_id, const core::TargetInfoMsg::SharedPtr msg);
        void clusterInfoCallback(const core::ID& cluster_id, const core::ClusterInfoMsg::SharedPtr msg);
        // Update
        void updateMetrics();
        void updateRvizMarkers();

    private:
        // Publishers
        core::PublisherPtr<core::GlobalMetricsMsg> global_metrics_pubs_;
        // Timers
        core::TimerPtr metrics_timer_;
        core::TimerPtr rviz_markers_timer_;
    };

} // namespace flychams::dashboard