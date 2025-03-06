#include "flychams_control/agent_control/pid_controller.hpp"

namespace flychams::control
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and initialization
    // ════════════════════════════════════════════════════════════════════════════

    PIDController::PIDController()
        : kp_(0.0f), ki_(0.0f), kd_(0.0f),
        min_(-HUGE_VALF), max_(HUGE_VALF),
        i_min_(-HUGE_VALF), i_max_(HUGE_VALF)
    {
        reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CONFIGURATION: Set controller parameters
    // ════════════════════════════════════════════════════════════════════════════

    void PIDController::setGains(const float& kp, const float& ki, const float& kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void PIDController::setOutputLimits(const float& min, const float& max)
    {
        min_ = min;
        max_ = max;
    }

    void PIDController::setIntegralLimits(const float& i_min, const float& i_max)
    {
        i_min_ = i_min;
        i_max_ = i_max;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CONTROL: Reset and update controller
    // ════════════════════════════════════════════════════════════════════════════

    void PIDController::reset()
    {
        p_ = 0.0f;
        i_ = 0.0f;
        d_ = 0.0f;
        prev_e_ = 0.0f;
    }

    float PIDController::update(const float& curr_x, const float& target_x, const float& dt)
    {
        if (dt <= 1e-6f) return 0.0f;  // Prevent division by zero

        float curr_e = target_x - curr_x; // Compute error

        // Compute P term
        p_ = kp_ * curr_e;

        // Compute I term with anti-windup
        i_ += ki_ * curr_e * dt;
        i_ = std::clamp(i_, i_min_, i_max_);

        // Compute D term
        d_ = kd_ * (curr_e - prev_e_) / dt;
        prev_e_ = curr_e;

        // Compute control signal with clamping
        float u = p_ + i_ + d_;
        return std::clamp(u, min_, max_);
    }

} // namespace flychams::control
