#pragma once

// Standard includes
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

// Utilities
#include "flychams_core/types/core_types.hpp"

namespace flychams::coordination
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Solver for cluster-agent assignment problems
     *
     * @details
     * This class implements different algorithms for solving the
     * cluster-agent assignment problem. It supports multiple assignment
     * modes including greedy coordination and sub-optimal coordination,
     * with configurable parameters for optimization.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-31
     * ════════════════════════════════════════════════════════════════
     * Assignment coordination modes:
     * - GreedyCoordination: Assigns clusters to agents based on distance
     * - SubOptimalCoordination: Advanced coordination algorithm (TODO)
     * ════════════════════════════════════════════════════════════════
     */
    class AssignmentSolver
    {
    public: // Constructor/Destructor
        AssignmentSolver();

    public: // Types
        using SharedPtr = std::shared_ptr<AssignmentSolver>;
        // Modes
        enum class AssignmentMode
        {
            GreedyCoordination,     // Distance-based greedy assignment. Default
            SubOptimalCoordination  // Advanced coordination (TODO)
        };
        // Settings types
        struct SolverParams
        {
            float tol;      // Convergence tolerance
            int max_iter;   // Maximum iterations
            float eps;      // Numerical stability epsilon
        };
        struct FunctionParams
        {
            float w_obs;    // Weight for observation quality
            float w_dist;   // Weight for distance cost
            float w_switch; // Weight for assignment switching
        };
        // Data types
        struct Agent
        {
            bool valid;             // Whether the agent is valid
            core::Vector3r pos;     // Agent position
            int max;                // Maximum number of assignments

            Agent() : valid(false), pos(0.0f, 0.0f, 0.0f), max(0) {}
            Agent(int max_assignments) : valid(false), pos(0.0f, 0.0f, 0.0f), max(max_assignments) {}
        };
        struct Cluster
        {
            bool valid;             // Whether the cluster is valid
            core::Vector3r center;  // Cluster center
            float radius;           // Cluster radius

            Cluster() : valid(false), center(0.0f, 0.0f, 0.0f), radius(0.0f) {}
        };
        using Agents = std::unordered_map<core::ID, Agent>; // Agent ID -> Agent
        using Clusters = std::unordered_map<core::ID, Cluster>; // Cluster ID -> Cluster
        using Assignments = std::unordered_map<core::ID, core::ID>; // Cluster ID -> Agent ID

    private: // Parameters
        SolverParams solver_params_;     // Solver configuration parameters
        FunctionParams params_;          // Cost function parameters

    private: // Data
        // IDs
        std::unordered_map<core::ID, int> A; // Agent ID -> Max assignments
        std::unordered_set<core::ID> C;      // Cluster IDs

    public: // Public methods
        // Parameter configuration methods
        void setSolverParams(const SolverParams& params);
        void setFunctionParams(const FunctionParams& params);
        // Control
        Assignments solveGreedy(const Agents& agents, const Clusters& clusters);

    private: // Implementation
        // TODO: Implementation of other assignment methods
    };

} // namespace flychams::coordination