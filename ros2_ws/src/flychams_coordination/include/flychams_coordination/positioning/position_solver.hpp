#pragma once

// Standard includes
#include <cmath>
#include <algorithm>

/* Non-Linear Optimization Library: https://github.com/stevengj/nlopt*/
#include <nlopt.hpp>

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for optimal agent positioning
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
    public: // Constructor/Destructor
        PositionSolver();
        ~PositionSolver();

    public: // Types
        using SharedPtr = std::shared_ptr<PositionSolver>;
        // Settings types
        struct SolverParams
        {
            float tol;        // Convergence tolerance
            int max_iter;     // Maximum iterations
            float eps;        // Solution difference threshold
        };
        struct FunctionParams
        {
            int n;                                              // Number of cameras
            std::vector<core::CameraParameters> camera_params_; // Camera parameters
            float hMin;                                         // Minimum height [m]
            float hMax;                                         // Maximum height [m]

            FunctionParams(int n, float h_min, float h_max)
                : n(n), hMin(h_min), hMax(h_max)
            {
                camera_params_.resize(n);
            }
        };
        // Data types
        struct FunctionData
        {
            FunctionParams params;        // Function parameters
            core::Matrix3Xr tab_P;        // Camera positions matrix
            core::RowVectorXr tab_r;      // Target positions vector
            core::Vector3r x_hat;         // Estimated optimal position

            FunctionData(
                const FunctionParams& params,
                const core::Matrix3Xr& tab_P,
                const core::RowVectorXr& tab_r,
                const core::Vector3r& x_hat)
                : params(params), tab_P(tab_P), tab_r(tab_r), x_hat(x_hat) {
            }
        };

    private: // Settings
        SolverParams solver_params_;     // Solver configuration
        FunctionParams params_;          // Cost function parameters

    private: // Data
        nlopt_opt opt_;                  // NLopt optimizer instance

    public: // Public methods
        // Parameter configuration methods
        void setSolverParams(const SolverParams& params);
        void setFunctionParams(const FunctionParams& params);
        // Solver lifecycle methods
        void initSolver();
        void destroySolver();
        // Optimization methods
        core::Vector3r solve(const core::Vector3r& x0, const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r);

    private: // Implementation
        // Optimization stages
        core::Vector3r preOptimization(const core::Vector3r& x0, const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r);
        core::Vector3r iterativeOptimization(const core::Vector3r& x0, const core::Matrix3Xr& tab_P, const core::RowVectorXr& tab_r, const float& eps);
        // Cost function implementations
        static double calculateJNestedIntervals(unsigned n, const double* x, double* grad, void* data);
        static double calculateJNestedIntervalsWithConvexRelaxation(unsigned n, const double* x, double* grad, void* data);
        // Optimization utility methods
        float optimize(core::Vector3r& xOpt);
    };

} // namespace flychams::coord