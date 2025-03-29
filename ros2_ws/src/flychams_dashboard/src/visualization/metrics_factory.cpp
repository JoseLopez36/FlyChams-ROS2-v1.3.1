#include "flychams_dashboard/metrics/metrics_factory.hpp"

using namespace flychams::core;

namespace flychams::dashboard
{
    // ════════════════════════════════════════════════════════════════════════════
    // DEFAULT METRICS: Create default metrics
    // ════════════════════════════════════════════════════════════════════════════

    AgentMetrics MetricsFactory::createDefaultAgent()
    {
        // Create default agent metrics
        AgentMetrics metrics;
        // Agent data
        metrics.curr_x = 0.0f;
        metrics.curr_y = 0.0f;
        metrics.curr_z = 0.0f;
        metrics.curr_yaw = 0.0f;
        metrics.vel_x = 0.0f;
        metrics.vel_y = 0.0f;
        metrics.vel_z = 0.0f;
        metrics.vel_yaw = 0.0f;
        metrics.goal_x = 0.0f;
        metrics.goal_y = 0.0f;
        metrics.goal_z = 0.0f;
        metrics.goal_yaw = 0.0f;
        // Position and movement metrics
        metrics.total_distance_traveled = 0.0f;
        metrics.current_speed = 0.0f;
        // Goal-related metrics
        metrics.distance_to_goal = 0.0f;
        // Mission metrics
        metrics.time_elapsed = 0.0f;
        // Performance metrics
        metrics.average_speed = 0.0f;

        return metrics;
    }

    TargetMetrics MetricsFactory::createDefaultTarget()
    {
        // Create default target metrics
        TargetMetrics metrics;
        // Target data
        metrics.curr_x = 0.0f;
        metrics.curr_y = 0.0f;
        metrics.curr_z = 0.0f;
        // Position and movement metrics
        metrics.total_distance_traveled = 0.0f;

        return metrics;
    }

    ClusterMetrics MetricsFactory::createDefaultCluster()
    {
        // Create default cluster metrics
        ClusterMetrics metrics;
        // Cluster data
        metrics.curr_center_x = 0.0f;
        metrics.curr_center_y = 0.0f;
        metrics.curr_center_z = 0.0f;
        metrics.curr_radius = 0.0f;
        // Position and movement metrics
        metrics.total_distance_traveled = 0.0f;

        return metrics;
    }

    GlobalMetrics MetricsFactory::createDefaultGlobal()
    {
        // Create default global metrics
        GlobalMetrics metrics;
        // Overall system metrics
        metrics.total_agents = 0;
        metrics.total_targets = 0;
        metrics.total_clusters = 0;
        // Mission metrics
        metrics.mission_time = 0.0f;

        return metrics;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update metrics
    // ════════════════════════════════════════════════════════════════════════════

    void MetricsFactory::updateAgentMetrics(const AgentMetrics& prev_metrics, AgentMetrics& curr_metrics, const float& dt)
    {
        // Get current and previous data as vectors
        const auto& curr_pos = Vector3r(curr_metrics.curr_x, curr_metrics.curr_y, curr_metrics.curr_z);
        const auto& prev_pos = Vector3r(prev_metrics.curr_x, prev_metrics.curr_y, prev_metrics.curr_z);
        const auto& curr_vel = Vector3r(curr_metrics.vel_x, curr_metrics.vel_y, curr_metrics.vel_z);
        const auto& curr_goal_pos = Vector3r(curr_metrics.goal_x, curr_metrics.goal_y, curr_metrics.goal_z);

        // Update position and movement metrics
        curr_metrics.total_distance_traveled += MathUtils::distance(curr_pos, prev_pos);
        curr_metrics.current_speed = curr_vel.norm();

        // Goal-related metrics
        curr_metrics.distance_to_goal = MathUtils::distance(curr_goal_pos, curr_pos);

        // Mission metrics
        curr_metrics.time_elapsed += dt;

        // Performance metrics
        curr_metrics.average_speed = curr_metrics.total_distance_traveled / curr_metrics.time_elapsed;
    }

    void MetricsFactory::updateTargetMetrics(const TargetMetrics& prev_metrics, TargetMetrics& curr_metrics, const float& dt)
    {
        // Get current and previous data as vectors
        const auto& curr_pos = Vector3r(curr_metrics.curr_x, curr_metrics.curr_y, curr_metrics.curr_z);
        const auto& prev_pos = Vector3r(prev_metrics.curr_x, prev_metrics.curr_y, prev_metrics.curr_z);

        // Update position and movement metrics
        curr_metrics.total_distance_traveled += MathUtils::distance(curr_pos, prev_pos);
    }

    void MetricsFactory::updateClusterMetrics(const ClusterMetrics& prev_metrics, ClusterMetrics& curr_metrics, const float& dt)
    {
        // Get current and previous data as vectors
        const auto& curr_center = Vector3r(curr_metrics.curr_center_x, curr_metrics.curr_center_y, curr_metrics.curr_center_z);
        const auto& curr_radius = curr_metrics.curr_radius;
        const auto& prev_center = Vector3r(prev_metrics.curr_center_x, prev_metrics.curr_center_y, prev_metrics.curr_center_z);
        const auto& prev_radius = prev_metrics.curr_radius;

        // Update position and movement metrics
        curr_metrics.total_distance_traveled += MathUtils::distance(curr_center, prev_center);
    }

    void MetricsFactory::updateGlobalMetrics(const GlobalMetrics& prev_metrics, GlobalMetrics& curr_metrics, const float& dt)
    {
        // Mission metrics
        curr_metrics.mission_time += dt;
    }

} // namespace flychams::dashboard