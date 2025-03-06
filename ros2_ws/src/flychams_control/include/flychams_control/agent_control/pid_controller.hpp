#pragma once

// Standard includes
#include <cmath>
#include <algorithm>

namespace flychams::control
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief PID controller implementation
     *
     * @details
     * This class implements a PID (Proportional-Integral-Derivative)
     * controller with anti-windup protection and output saturation.
     * It provides methods for tuning gains and updating control values.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-29
     * ════════════════════════════════════════════════════════════════
     */
    class PIDController
    {
    public: // Constructor
        PIDController();

    private: // Parameters
        // Gains
        float kp_; // Proportional gain
        float ki_; // Integral gain
        float kd_; // Derivative gain
        // Limits
        float min_;   // Minimum output
        float max_;   // Maximum output
        float i_min_; // Minimum integral term
        float i_max_; // Maximum integral term

    private: // Data
        // Terms
        float p_; // Proportional term
        float i_; // Integral term
        float d_; // Derivative term
        // Previous data
        float prev_e_; // Previous error

    public: // Methods
        void setGains(const float& kp, const float& ki, const float& kd);
        void setOutputLimits(const float& min, const float& max);
        void setIntegralLimits(const float& i_min, const float& i_max);
        void reset();
        float update(const float& curr_x, const float& target_x, const float& dt);
    };

} // namespace flychams::control
