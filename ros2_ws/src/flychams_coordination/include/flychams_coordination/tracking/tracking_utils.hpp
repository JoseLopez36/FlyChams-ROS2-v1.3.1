#pragma once

// Standard includes
#include <cmath>
#include <algorithm>

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/utils/math_utils.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Tracking utilities implementation
     *
     * @details
     * This class implements tracking utilities, such as focal length
     * or orientation computation.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-27
     * ════════════════════════════════════════════════════════════════
     */
    class TrackingUtils
    {
    private: // Types
        enum class AimingMode
        {
            INITIAL,
            CONTINUOUS
        };

    public: // Public methods
        static float computeFocal(const core::Vector3r& wPt, const float& r, const core::Vector3r& wPc, const float& fMin, const float& fMax, const float& sRef);
        static core::Vector3r computeOrientation(const core::Vector3r& wPt, const core::Vector3r& wPc, const core::Matrix3r& wRc, const core::Vector3r& prev_rpy, const bool& is_first_update);

    private: // Methods
        static std::pair<float, float> calculateBaseSolution(const core::Vector3r& dir);
        static std::pair<float, float> calculateInvertedSolution(const core::Vector3r& dir);
        static core::Vector3r calculateContinuousSolution(const float& yaw1, const float& pitch1, const float& yaw2, const float& pitch2, const core::Vector3r& prev_rpy);
    };

} // namespace flychams::coordination
