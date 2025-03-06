#include "flychams_coordination/positioning/position_solver.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    PositionSolver::PositionSolver()
        : params_(FunctionParams(0, 0.0f, 300.0f))
    {
        // Initialize solver parameters
        solver_params_.tol = 1e-6f;
        solver_params_.max_iter = 100;
        solver_params_.eps = 1.0f;
    }

    PositionSolver::~PositionSolver()
    {
        destroySolver();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // METHODS: Configuration and runtime methods
    // ════════════════════════════════════════════════════════════════════════════

    void PositionSolver::setSolverParams(const SolverParams& params)
    {
        if (params.eps <= 0.0f || params.tol <= 0.0f)
            throw std::invalid_argument("Tolerances must be positive");

        solver_params_.tol = params.tol;
        solver_params_.max_iter = params.max_iter;
        solver_params_.eps = params.eps;
    }

    void PositionSolver::setFunctionParams(const FunctionParams& params)
    {
        if (params.n <= 0)
            throw std::invalid_argument("Number of cameras must be positive");
        if (params.hMin >= params.hMax)
            throw std::invalid_argument("hMin must be less than hMax");
        if (params.camera_params_.size() != params.n)
            throw std::invalid_argument("Number of camera parameters must match the number of cameras");

        params_ = params;
    }

    void PositionSolver::initSolver()
    {
        // Create an NLopt optimizer
        int dim = 3; // Dimension of the problem
        opt_ = nlopt_create(NLOPT_LN_NELDERMEAD, dim);
        if (!opt_)
        {
            throw std::runtime_error("Failed to create NLOpt optimizer");
            return;
        }

        // Define the optimization bounds
        const double lb[3] = { -HUGE_VAL, -HUGE_VAL, static_cast<double>(params_.hMin) };
        const double ub[3] = { HUGE_VAL, HUGE_VAL, static_cast<double>(params_.hMax) };
        nlopt_set_lower_bounds(opt_, lb);
        nlopt_set_upper_bounds(opt_, ub);

        // Optimization options
        nlopt_set_xtol_rel(opt_, static_cast<double>(solver_params_.tol)); // Set convergence tolerance
        nlopt_set_maxeval(opt_, solver_params_.max_iter);                  // Maximum number of function evaluations
    }

    void PositionSolver::destroySolver()
    {
        if (opt_)
        {
            nlopt_destroy(opt_);
        }
    }

    Vector3r PositionSolver::solve(const Vector3r& x0, const Matrix3Xr& tab_P, const RowVectorXr& tab_r)
    {
        // Clip height to limits
        Vector3r x0_clipped = x0;
        x0_clipped(2) = std::clamp(x0(2), static_cast<float>(params_.hMin), static_cast<float>(params_.hMax));

        // First optimization (solve for initial position)
        Vector3r xOpt_1 = preOptimization(x0_clipped, tab_P, tab_r);

        // Iterative optimization (solve for optimal position)
        return iterativeOptimization(xOpt_1, tab_P, tab_r, solver_params_.eps);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // IMPLEMENTATION: Optimization methods
    // ════════════════════════════════════════════════════════════════════════════

    Vector3r PositionSolver::preOptimization(const Vector3r& x0, const Matrix3Xr& tab_P, const RowVectorXr& tab_r)
    {
        // Set the objective cost function J1 (with parameters)
        FunctionData dataJ1(params_, tab_P, tab_r, Vector3r::Zero());
        nlopt_set_min_objective(opt_, calculateJNestedIntervals, &dataJ1);

        // Optimize for J1
        Vector3r xOpt = x0;
        optimize(xOpt);

        return xOpt;
    }

    Vector3r PositionSolver::iterativeOptimization(const Vector3r& x0, const Matrix3Xr& tab_P, const RowVectorXr& tab_r, const float& eps)
    {
        // Initialize with the previous solution
        Vector3r xOpt_prev = x0;
        Vector3r xOpt = xOpt_prev;

        // Define the data for the convex relaxation (J2)
        FunctionData dataJ2(params_, tab_P, tab_r, Vector3r::Zero());

        // Iteratively call the optimization algorithm that implements the convex relaxation (J2)
        float xDifNorm = HUGE_VALF;
        while (xDifNorm > eps)
        {
            // Define the xHat parameter of cost function J2
            dataJ2.x_hat << xOpt_prev[0], xOpt_prev[1], xOpt_prev[2];

            // Set the objective cost function J2 (with parameters)
            nlopt_set_min_objective(opt_, calculateJNestedIntervalsWithConvexRelaxation, &dataJ2);

            // Optimize for J2
            optimize(xOpt);

            // Compute the norm of difference
            xDifNorm = (xOpt - xOpt_prev).norm();

            // Update the previous solution
            xOpt_prev = xOpt;
        }

        return xOpt;
    }

    double PositionSolver::calculateJNestedIntervals(unsigned n, const double* x, double* grad, void* data)
    {
        // Extract data
        FunctionData* function_data = reinterpret_cast<FunctionData*>(data);
        Vector3r x_vec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));

        // Compute the value of the optimization index based on nested intervals
        float J = 0.0f;
        for (int i = 0; i < function_data->params.n; i++)
        {
            // Extract the necessary parameters
            const auto& camera_params = function_data->params.camera_params_[i];
            const float& fMin = camera_params.f_min;
            const float& fMax = camera_params.f_max;
            const float& fRef = camera_params.f_ref;
            const float& sMin = camera_params.s_min;
            const float& sMax = camera_params.s_max;
            const float& sRef = camera_params.s_ref;

            // Target position and distance to its camera (approximated by distance to the vehicle)
            const Vector3r z_vec = function_data->tab_P.col(i);
            const float d = (x_vec - z_vec).norm();

            // Equivalent interest radius of the real target
            const float r = function_data->tab_r(i);

            // Calculate the reference distance
            const float dRef = r * fRef / sRef;

            // Calculate what would be the ideal reference position in the case of a single target
            Vector3r pRef_vec = z_vec;
            pRef_vec(2) += dRef;

            // Determine the intervals
            const float L0 = dRef;
            const float U0 = dRef;
            const float L1 = r * fMin / sRef;
            const float U1 = r * fMax / sRef;
            const float L2 = r * fMin / sMax;
            const float U2 = r * fMax / sMin;

            // Calculate the index terms based on intervals
            const float Psi_i = 1.0f * pow((std::max)(0.0f, d - U0), 2) + 2.0f * pow((std::max)(0.0f, d - U1), 2) + 10.0f * pow((std::max)(0.0f, d - U2), 2);
            const float Lambda_i = 0.0f;

            const float mu_i = 1.0f;
            const float nu_i = 1.0f;
            const float Gamma_i = mu_i * (x_vec - pRef_vec).transpose() * (x_vec - pRef_vec) + nu_i * pow((d - (x_vec - z_vec).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Accumulate into the total index
            J += Psi_i + Lambda_i + Gamma_i;
        }

        return static_cast<double>(J);
    }

    double PositionSolver::calculateJNestedIntervalsWithConvexRelaxation(unsigned n, const double* x, double* grad, void* data)
    {
        // Extract data
        FunctionData* function_data = reinterpret_cast<FunctionData*>(data);
        Vector3r x_vec(static_cast<float>(x[0]), static_cast<float>(x[1]), static_cast<float>(x[2]));

        // Distance threshold to consider that xHat coincides with zi
        // Considering that hMin is several meters, it should not be reached unless set very high, since zi are at height 0
        float eps_dist = 0.1f;

        // Compute the value of the optimization index based on nested intervals
        float J = 0.0f;
        for (int i = 0; i < function_data->params.n; i++)
        {
            // Extract the necessary parameters
            const auto& camera_params = function_data->params.camera_params_[i];
            const float& fMin = camera_params.f_min;
            const float& fMax = camera_params.f_max;
            const float& fRef = camera_params.f_ref;
            const float& sMin = camera_params.s_min;
            const float& sMax = camera_params.s_max;
            const float& sRef = camera_params.s_ref;

            // Target position and distance to its camera (approximated by distance to the vehicle)
            const Vector3r z_vec = function_data->tab_P.col(i);
            const float d = (x_vec - z_vec).norm();

            // Vector indicating the direction to project
            const Vector3r v_vec = function_data->x_hat - z_vec;
            const float v_norm = v_vec.norm();
            Vector3r eta_vec = Vector3r::Zero();
            if (v_norm > eps_dist)
                eta_vec = v_vec / v_norm;

            // Calculate the projection as a substitute for distance for the non-convex term
            const float dProj = (x_vec - z_vec).transpose() * eta_vec;

            // Equivalent interest radius of the real target
            const float r = function_data->tab_r(i);

            // Calculate the reference distance
            const float dRef = r * fRef / sRef;

            // Calculate what would be the ideal reference position in the case of a single target
            Vector3r pRef_vec = z_vec;
            pRef_vec(2) += dRef;

            // Determine the intervals
            const float L0 = dRef;
            const float U0 = dRef;
            const float L1 = r * fMin / sRef;
            const float U1 = r * fMax / sRef;
            const float L2 = r * fMin / sMax;
            const float U2 = r * fMax / sMin;

            // Calculate the index terms based on intervals
            const float alpha = 2.0f;
            const float Psi_i = 1.0f * pow((std::max)(0.0f, d - U0), alpha) + 2.0f * pow((std::max)(0.0f, d - U1), alpha) + 10.0f * pow((std::max)(0.0f, d - U2), alpha);
            const float Lambda_i = 1.0f * pow((std::max)(0.0f, L0 - dProj), alpha) + 2.0f * pow((std::max)(0.0f, L1 - dProj), alpha) + 10.0f * pow((std::max)(0.0f, L2 - dProj), alpha);

            const float mu_i = 1.0f;
            const float nu_i = 1.0f;
            const float Gamma_i = mu_i * (x_vec - pRef_vec).transpose() * (x_vec - pRef_vec) + nu_i * pow((d - (x_vec - z_vec).transpose() * Vector3r(0.0f, 0.0f, 1.0f)), 2);

            // Accumulate into the total index
            J += Psi_i + Lambda_i + Gamma_i;
        }

        return static_cast<double>(J);
    }

    float PositionSolver::optimize(Vector3r& xOpt)
    {
        double xOpt_nlopt[3] = { static_cast<double>(xOpt(0)), static_cast<double>(xOpt(1)), static_cast<double>(xOpt(2)) };

        // Call the optimization algorithm
        // J: optimal value of the cost function
        // xOpt: value that minimizes the cost function
        double J;
        nlopt_optimize(opt_, xOpt_nlopt, &J);

        // Update the position
        xOpt << static_cast<float>(xOpt_nlopt[0]), static_cast<float>(xOpt_nlopt[1]), static_cast<float>(xOpt_nlopt[2]);

        // Return the optimal value of the cost function
        return static_cast<float>(J);
    }

} // namespace flychams::coordination