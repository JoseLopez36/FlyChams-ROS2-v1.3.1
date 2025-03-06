#pragma once

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/ros_types.hpp"
#include "flychams_core/utils/math_utils.hpp"

namespace flychams::dashboard
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Factory for creating and updating simulation markers
     *
     * @details
     * This class is responsible for creating and updating various
     * markers used in the simulation, including agent, target,
     * cluster, and global markers. It provides standardized methods
     * for markers creation and updates.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-27
     * ════════════════════════════════════════════════════════════════
     */
    class MarkersFactory
    {
    private: // Parameters
        static constexpr float BASE_MARKER_SIZE = 3.5f;
        static constexpr float BASE_LINE_WIDTH = 0.8f;
        static constexpr float BASE_MARKER_ALPHA = 0.8f;

    public: // Public methods
        // Marker arrays creation
        static core::MarkerArrayMsg createAgentMarkers(const std::string& frame_id, const float& duration);
        static core::MarkerArrayMsg createTargetMarkers(const std::string& frame_id, const float& duration);
        static core::MarkerArrayMsg createClusterMarkers(const std::string& frame_id, const float& duration);

        // Markers creation
        static core::MarkerMsg createAgentPoint(const int& marker_idx, const std::string& frame_id, const float& duration);
        static core::MarkerMsg createAgentGoalPoint(const int& marker_idx, const std::string& frame_id, const float& duration);
        static core::MarkerMsg createTargetPoint(const int& marker_idx, const std::string& frame_id, const float& duration);
        static core::MarkerMsg createClusterCenter(const int& marker_idx, const std::string& frame_id, const float& duration);
        static core::MarkerMsg createClusterBoundary(const int& marker_idx, const std::string& frame_id, const float& duration);

        // Markers update
        static void updateAgentPoint(const float& curr_x, const float& curr_y, const float& curr_z, const core::Time& time, core::MarkerMsg& marker);
        static void updateAgentGoalPoint(const float& goal_x, const float& goal_y, const float& goal_z, const core::Time& time, core::MarkerMsg& marker);
        static void updateTargetPoint(const float& curr_x, const float& curr_y, const float& curr_z, const core::Time& time, core::MarkerMsg& marker);
        static void updateClusterCenter(const float& center_x, const float& center_y, const float& center_z, const core::Time& time, core::MarkerMsg& marker);
        static void updateClusterBoundary(const float& center_x, const float& center_y, const float& center_z, const float& radius, const core::Time& time, core::MarkerMsg& marker);
    };

} // namespace flychams::dashboard