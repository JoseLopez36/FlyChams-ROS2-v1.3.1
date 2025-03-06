#pragma once

// Standard includes
#include <unordered_map>
#include <vector>
#include <unordered_set>

// Cluster analysis include
#include "flychams_perception/clustering/cluster_analysis.hpp"

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/utils/math_utils.hpp"
namespace flychams::perception
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief K-Means modified implementation
	 *
	 * @details
	 * This class implements a K-Means modified algorithm.
	 * It provides methods for clustering targets.
	 * The modifications include:
	 * - Inter-iteration cluster consistency.
	 * - Time persistence of clusters.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-01-29
	 * ════════════════════════════════════════════════════════════════
	 * Example of usage:
	 * ════════════════════════════════════════════════════════════════
	 * // Set persistence weights
	 * KmeansMod kmeans;
	 * kmeans.setPersistenceWeights(1.0f, 1.0f, 2.0f, 0.5f);
	 *
	 * // Initialize K-Means (must be called each time points or clusters change)
	 * std::unordered_set<core::ID> point_ids = {"point_1", "point_2", "point_3", "point_4", "point_5"};
	 * std::unordered_set<core::ID> cluster_ids = {"cluster_1", "cluster_2"};
	 * kmeans.initialize(point_ids, cluster_ids);
	 *
	 * // Run clustering
	 * assignments = kmeans.performClustering(points, dt);
	 *
	 * // Results:
	 * // For example, the assignments will be:
	 * // - {"point_1" -> "cluster_1"}
	 * // - {"point_2" -> "cluster_1"}
	 * // - {"point_3" -> "cluster_2"}
	 * // - {"point_4" -> "cluster_2"}
	 * // - {"point_5" -> "cluster_2"}
	 * ════════════════════════════════════════════════════════════════
	 */
	class KmeansMod
	{
	public: // Constructor
		KmeansMod();
		void reset();

	public: // Types
		using SharedPtr = std::shared_ptr<KmeansMod>;
		// Output
		using Assignments = std::unordered_map<core::ID, core::ID>; // Point ID -> Cluster ID

	private: // Types
		// Internal types
		using IndexToIDMap = std::unordered_map<int, core::ID>;
		using IDToIndexMap = std::unordered_map<core::ID, int>;
		struct ClusterData
		{
			core::RowVectorXi assignments;
			core::Matrix3Xr centroids;

			ClusterData() = default;
			ClusterData(int n, int K)
				: assignments(n), centroids(3, K)
			{
				assignments.setZero();
				centroids.setZero();
			}
		};

	private: // Parameters
		// Persistence weights
		float w_opt_ = 1.0f;
		float w_cool_ = 1.0f;
		float w_switch_ = 2.0f;
		float w_bond_ = 0.5f;

	private: // Data
		// Index to ID maps
		IndexToIDMap point_index_to_id_;
		IDToIndexMap point_id_to_index_;
		IndexToIDMap cluster_index_to_id_;
		IDToIndexMap cluster_id_to_index_;

		// Clustering data
		int K_;
		int n_;

		// Previous data
		ClusterData prev_data_;
		ClusterData new_data_;

		// Persistence data
		float cooldown_time_;
		core::RowVectorXr bonding_time_;
		core::RowVectorXi prev_calculated_assignments_;
		// First update flag
		bool first_update_;

	public: // Public methods
		// Configuration
		void setPersistenceWeights(const float& w_opt, const float& w_cool, const float& w_switch, const float& w_bond);
		// Control
		void initialize(const core::UnorderedIDs& point_ids, const core::UnorderedIDs& cluster_ids);
		Assignments performClustering(const std::unordered_map<core::ID, core::Vector3r>& points, const float& dt);

	private: // Implementation
		// Base K-means implementation. Initial assignments
		ClusterData computeInitialAssignments(const core::Matrix3Xr& tab_P, int n, int K);
		core::RowVectorXi selectFarthestPoints(const core::Matrix3Xr& tab_P, int n, int K);
		core::MatrixXr computeDistMatrix(const core::Matrix3Xr& tab_P, int n);
		core::RowVectorXi assignClusters(const core::Matrix3Xr& tab_P, const core::Matrix3Xr& centroids, int n, int K);

		// Consistent clustering implementation. Consistent centroids
		ClusterData reorderCentroidsConsistently(const ClusterData& prev_data, const ClusterData& new_data, int n, int K);
		core::RowVectorXi associateCentroids(const core::Matrix3Xr& tab1, const core::Matrix3Xr& tab2, int K);
		core::MatrixXr computeDistMatrixTwoGroups(const core::Matrix3Xr& tab_P1, const core::Matrix3Xr& tab_P2, int K);

		// Clustering persistence implementation. Cluster persistence
		ClusterData ensureClusterPersistence(const core::Matrix3Xr& tab_P, const ClusterData& prev_data, const ClusterData& new_data, int n, int K, float dt);
	};

} // namespace flychams::perception
