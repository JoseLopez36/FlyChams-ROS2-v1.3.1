#pragma once

// Utilities
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/utils/math_utils.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for agent tracking
     *
     * @details
     * This
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-31
     * ════════════════════════════════════════════════════════════════
     */
    class TrackingSolver
    {
    public: // Types
        using SharedPtr = std::shared_ptr<TrackingSolver>;
        // Modes
        enum class AimingMode
        {
            INITIAL,
            CONTINUOUS
        };
        // Data
        struct Data
        {
            // MultiCamera tracking data
            float focal;
            core::Vector3r rpy;
            bool is_first_update;
            // MultiWindow tracking data
            core::Vector2i size;
            core::Vector2i corner;

            Data()
                : focal(0.0f), rpy(0.0f, 0.0f, 0.0f), size(0, 0), corner(0, 0)
            {
            }
        };

    private: // Data
        // Previous tracking data
        Data data_;

    public: // Public methods
        // Configuration
        void reset();
        // Optimization
        std::tuple<float, core::Vector3r, float> runCamera(const core::Vector3r& z, const float& r, const core::Matrix4r& T, const core::HeadParameters& head_params);
        std::tuple<core::Vector2i, core::Vector2i, float, float> runWindow(const core::Vector3r& z, const float& r, const core::Matrix4r& T, const core::HeadParameters& central_params, const core::WindowParameters& window_params);

    private: // Implementation
        // Camera tracking implementation
        std::pair<float, float> computeCameraFocal(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::HeadParameters& head_params);
        core::Vector3r computeCameraOrientation(const core::Vector3r& z, const core::Vector3r& x, const core::Matrix3r& wRc, const core::Vector3r& prev_rpy, const bool& is_first_update);
        std::pair<float, float> calculateCameraBaseSolution(const core::Vector3r& dir);
        std::pair<float, float> calculateCameraInvertedSolution(const core::Vector3r& dir);
        core::Vector3r calculateCameraContinuousSolution(const float& yaw1, const float& pitch1, const float& yaw2, const float& pitch2, const core::Vector3r& prev_rpy);
        // Window tracking implementation
        std::tuple<core::Vector2i, float, float> computeWindowSize(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::HeadParameters& central_params, const core::WindowParameters& window_params);
        core::Vector2i computeWindowCorner(const core::Vector2r& p, const core::Vector2i& size);
    };

} // namespace flychams::coordination