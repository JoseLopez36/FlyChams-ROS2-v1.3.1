#include "flychams_coordination/assignment/assignment_solver.hpp"

using namespace flychams::core;

namespace flychams::coordination
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    AssignmentSolver::AssignmentSolver()
    {
        // Initialize solver parameters
        solver_params_.tol = 1e-6f;
        solver_params_.max_iter = 100;
        solver_params_.eps = 1.0f;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CONFIGURATION: Set solver parameters and function parameters
    // ════════════════════════════════════════════════════════════════════════════

    void AssignmentSolver::setSolverParams(const SolverParams& params)
    {
        if (params.eps <= 0.0f || params.tol <= 0.0f)
            throw std::invalid_argument("Tolerances must be positive");

        solver_params_ = params;
    }

    void AssignmentSolver::setFunctionParams(const FunctionParams& params)
    {
        if (params.w_obs <= 0.0f || params.w_dist <= 0.0f || params.w_switch <= 0.0f)
            throw std::invalid_argument("Weights must be positive");

        params_ = params;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // SOLVER METHODS: Assignment solving methods
    // ════════════════════════════════════════════════════════════════════════════

    AssignmentSolver::Assignments AssignmentSolver::solveGreedy(const Agents& A, const Clusters& C)
    {
        // Initialize empty assignment
        Assignments assignments;

        // If no clusters, return empty assignment
        if (C.empty())
        {
            return assignments;
        }

        // Create a list of unassigned clusters
        std::vector<std::pair<core::ID, core::Vector3r>> U;
        for (const auto& [Cj_id, Cj] : C)
        {
            U.push_back({ Cj_id, Cj.center });
        }

        // Create a map to track how many clusters are assigned to each agent
        std::unordered_map<core::ID, int> n;
        for (const auto& [Ai_id, _] : A)
        {
            n[Ai_id] = 0;
        }

        // Assign clusters to agents greedily based on distance
        while (!U.empty())
        {
            float min_dist = HUGE_VALF;
            core::ID best_A_id = "";
            size_t best_U_idx = 0;

            // Find the best agent-cluster pair based on distance
            for (size_t j = 0; j < U.size(); j++)
            {
                const auto& [Uj_id, Uj] = U[j];

                for (const auto& [Ai_id, Ai] : A)
                {
                    // Skip if agent has reached maximum assignments
                    if (n[Ai_id] >= Ai.max)
                        continue;

                    // Calculate distance
                    float dist = (Ai.pos - Uj).norm();

                    // Update best assignment if this is better
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        best_A_id = Ai_id;
                        best_U_idx = j;
                    }
                }
            }

            // If we found a valid assignment
            if (min_dist < HUGE_VALF)
            {
                // Get the cluster ID
                const auto& [Uj_id, _] = U[best_U_idx];

                // Add to assignments
                assignments[Uj_id] = best_A_id;

                // Increment agent assignment count
                n[best_A_id]++;

                // Remove the assigned cluster from unassigned list
                U.erase(U.begin() + best_U_idx);
            }
            else
            {
                // No valid assignment found for remaining clusters
                break;
            }
        }

        return assignments;
    }

} // namespace flychams::coordination