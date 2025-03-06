#pragma once

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/utils/math_utils.hpp"

namespace flychams::dashboard
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Factory for creating and updating simulation metrics
     *
     * @details
     * This class is responsible for creating and updating various
     * metrics used in the simulation, including agent, target,
     * cluster, and global metrics. It provides standardized methods
     * for metrics creation and updates.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-27
     * ════════════════════════════════════════════════════════════════
     */
    class MetricsFactory
    {
    public: // Public methods
        // Default metrics creation
        static core::AgentMetrics createDefaultAgent();
        static core::TargetMetrics createDefaultTarget();
        static core::ClusterMetrics createDefaultCluster();
        static core::GlobalMetrics createDefaultGlobal();
        // Metrics update methods
        static void updateAgentMetrics(const core::AgentMetrics& prev_metrics, core::AgentMetrics& curr_metrics, const float& dt);
        static void updateTargetMetrics(const core::TargetMetrics& prev_metrics, core::TargetMetrics& curr_metrics, const float& dt);
        static void updateClusterMetrics(const core::ClusterMetrics& prev_metrics, core::ClusterMetrics& curr_metrics, const float& dt);
        static void updateGlobalMetrics(const core::GlobalMetrics& prev_metrics, core::GlobalMetrics& curr_metrics, const float& dt);
    };

} // namespace flychams::dashboard