#pragma once

// Non-Linear Optimization Library: https://github.com/stevengj/nlopt
#include <nlopt.hpp>

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for agent positioning
     *
     * @details
     * This class implements optimization algorithms for finding optimal
     * agent positions based on target positions and visibility constraints.
     * It uses non-linear optimization techniques to minimize cost functions
     * that consider camera parameters, target sizes, and height constraints.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-31
     * ════════════════════════════════════════════════════════════════
     */
    class PositionSolver
    {
    public: // Types
        using SharedPtr = std::shared_ptr<PositionSolver>;
        // Modes
        enum class SolverMode
        {
            NLOPT_NELDER_MEAD
        };
        // Parameters
        struct CostParameters
        {
            core::TrackingMode mode;

            // Camera parameters
            float f_min = 0.0f;
            float f_max = 0.0f;
            float f_ref = 0.0f;

            // Window parameters
            float lambda_min = 0.0f;
            float lambda_max = 0.0f;
            float lambda_ref = 0.0f;
            float central_f = 0.0f;

            // Projection parameters
            float s_min = 0.0f;
            float s_max = 0.0f;
            float s_ref = 0.0f;

            // Cost function weights
            // Psi
            float tau0 = 1.0f;
            float tau1 = 2.0f;
            float tau2 = 10.0f;
            // Lambda
            float sigma0 = 1.0f;
            float sigma1 = 2.0f;
            float sigma2 = 10.0f;
            // Gamma
            float mu = 1.0f;
            float nu = 1.0f;
        };
        // Data
        struct CostData
        {
            int n;
            core::Matrix3Xr tab_P;                          // Cluster positions matrix
            core::RowVectorXr tab_r;                        // Cluster radii vector
            core::Vector3r xHat;                            // Estimated optimal position
            CostParameters central_params;                  // Cost parameters for the central head
            std::vector<CostParameters> tracking_params;    // Cost parameters for each tracking camera/window

            CostData(const int& n)
                : n(n), tab_P(3, n), tab_r(n), xHat(), central_params(), tracking_params(n) {
            }
        };

    private: // Parameters
        SolverMode mode_;
        float tol_;
        int max_iter_;
        float eps_;

    private: // Data
        // NLopt optimizer instance
        nlopt_opt opt_;

    public: // Public methods
        // Configuration
        void init();
        void destroy();
        void reset();
        void setMode(const SolverMode& mode);
        void setParameters(const float& tol, const int& max_iter, const float& eps);
        // Optimization
        core::Vector3r run(const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r,
            const core::Vector3r& x0, const float& min_h, const float& max_h,
            const CostParameters& central_params, const std::vector<CostParameters>& tracking_params);

    private: // Implementation
        // Optimization stages
        core::Vector3r preOptimization(const core::Vector3r& x0, CostData& data);
        core::Vector3r iterativeOptimization(const core::Vector3r& x0, CostData& data);
        // Objective functions
        static double funJ1(unsigned n, const double* x, double* grad, void* data);
        static double funJ2(unsigned n, const double* x, double* grad, void* data);
        // Cost function implementations
        static float calculateCameraJ1(const core::Vector3r& z, const float& r, const core::Vector3r& x, const CostParameters& params);
        static float calculateCameraJ2(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::Vector3r& xHat, const CostParameters& params);
        static float calculateWindowJ1(const core::Vector3r& z, const float& r, const core::Vector3r& x, const CostParameters& params);
        static float calculateWindowJ2(const core::Vector3r& z, const float& r, const core::Vector3r& x, const core::Vector3r& xHat, const CostParameters& params);
        // Optimization utility methods
        float optimize(core::Vector3r& xOpt);
    };

} // namespace flychams::coord