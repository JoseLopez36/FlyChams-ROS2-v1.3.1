#include "flychams_perception/clustering/kmeans_mod.hpp"

/* Hungarian algorithm: https://github.com/mcximing/hungarian-algorithm-cpp */
#include "Hungarian.h"

using namespace flychams::core;

namespace flychams::perception
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and initialization
	// ════════════════════════════════════════════════════════════════════════════

	KmeansMod::KmeansMod()
	{
		reset();
	}

	void KmeansMod::reset()
	{
		// Clear identification maps
		point_index_to_id_.clear();
		point_id_to_index_.clear();
		cluster_index_to_id_.clear();
		cluster_id_to_index_.clear();

		// Clear data
		K_ = 0;
		n_ = 0;
		prev_data_ = ClusterData(0, 0);
		new_data_ = ClusterData(0, 0);
		prev_calculated_assignments_ = RowVectorXi::Zero(0);
		cooldown_time_ = 0.0f;
		bonding_time_ = RowVectorXr::Zero(0);
		first_update_ = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CONFIGURATION: Set clustering parameters
	// ════════════════════════════════════════════════════════════════════════════

	void KmeansMod::setPersistenceWeights(const float& w_opt, const float& w_cool, const float& w_switch, const float& w_bond)
	{
		w_opt_ = w_opt;
		w_cool_ = w_cool;
		w_switch_ = w_switch;
		w_bond_ = w_bond;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CONTROL: Initialize and run clustering
	// ════════════════════════════════════════════════════════════════════════════

	void KmeansMod::initialize(const core::UnorderedIDs& point_ids, const core::UnorderedIDs& cluster_ids)
	{
		// Clear previous data
		reset();

		// Create identification maps
		int point_index = 0;
		for (const auto& point_id : point_ids)
		{
			point_index_to_id_.insert({ point_index, point_id });
			point_id_to_index_.insert({ point_id, point_index });
			point_index++;
		}

		int cluster_index = 0;
		for (const auto& cluster_id : cluster_ids)
		{
			cluster_index_to_id_.insert({ cluster_index, cluster_id });
			cluster_id_to_index_.insert({ cluster_id, cluster_index });
			cluster_index++;
		}

		// Get number of points and clusters
		n_ = static_cast<int>(point_ids.size());
		K_ = static_cast<int>(cluster_ids.size());

		// Initialize data
		prev_data_ = ClusterData(n_, K_);
		new_data_ = ClusterData(n_, K_);
		prev_calculated_assignments_ = RowVectorXi::Zero(n_);
		bonding_time_ = RowVectorXr::Zero(n_);
	}

	KmeansMod::Assignments KmeansMod::performClustering(const std::unordered_map<core::ID, core::Vector3r>& points, const float& dt)
	{
		// Check extreme cases
		if (K_ <= 0 || n_ < K_ || n_ != static_cast<int>(points.size()))
			return Assignments(); // Return empty map

		// Create point matrix using column index as point index
		Matrix3Xr tab_P = Matrix3Xr::Zero(3, n_);
		for (int i = 0; i < n_; i++)
		{
			tab_P.col(i) = points.at(point_index_to_id_.at(i));
		}

		// Perform clustering with different methods, depending on the first update flag
		ClusterData data;
		if (first_update_)
		{
			// Perform clustering with base K-means
			data = computeInitialAssignments(tab_P, n_, K_);
			first_update_ = false;
		}
		else
		{
			// Perform clustering with modified K-means
			// First compute initial assignments
			data = computeInitialAssignments(tab_P, n_, K_);
			// Then ensure consistency in cluster numbering
			data = reorderCentroidsConsistently(prev_data_, data, n_, K_);
			// Finally, ensure persistence of centroids
			data = ensureClusterPersistence(tab_P, prev_data_, data, n_, K_, dt);
		}

		// Update previous data
		prev_data_ = data;

		// Return assignments
		const RowVectorXi& assignments = data.assignments;
		Assignments assignments_map;
		for (int i = 0; i < n_; i++)
		{
			assignments_map[point_index_to_id_.at(i)] = cluster_index_to_id_.at(assignments(i));
		}

		return assignments_map;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// BASE K-MEANS: Compute initial assignments
	// ════════════════════════════════════════════════════════════════════════════

	KmeansMod::ClusterData KmeansMod::computeInitialAssignments(const Matrix3Xr& tab_P, int n, int K)
	{
		// Function that computes the initial assignments for the base K-means algorithm
		// Create output
		ClusterData data(n, K);

		// Initialize clusters with the K farthest points
		RowVectorXi farthest_list = selectFarthestPoints(tab_P, n, K);
		for (int i = 0; i < K; i++)
		{
			data.assignments(farthest_list(i)) = i;
		}

		// K-means Iteration
		RowVectorXi prev_iter_assign(n);
		bool changed = true;
		while (changed)
		{
			for (int i = 0; i < K; i++)
			{
				// Get cluster assigned points
				Matrix3Xr cluster_points(3, 0);
				int num_points = 0;
				for (int j = 0; j < n; j++)
				{
					if (i == data.assignments(j))
					{
						cluster_points.conservativeResize(3, num_points + 1);
						cluster_points.col(num_points) = tab_P.col(j);
						num_points++;
					}
				}

				// Compute new cluster centroid
				data.centroids.col(i) = ClusterAnalysis::computeCentroid(cluster_points);
			}

			// Store calculated assignments to check for changes
			prev_iter_assign = data.assignments;

			// Assign clusters based on the updated centroids
			data.assignments = assignClusters(tab_P, data.centroids, n, K);

			// Check if any assignments have changed
			changed = (data.assignments.array() != prev_iter_assign.array()).any();

			// If it hasn't changed, return the last result
		}

		return data;
	}

	RowVectorXi KmeansMod::selectFarthestPoints(const Matrix3Xr& tab_P, int n, int K)
	{
		// Function that selects the 'K' farthest points from a given set of points provided by the
		// columns of 'points'. If 'K' is 1, it simply selects the first point.
		RowVectorXi selected_list = RowVectorXi::Zero(1);

		if (K == 1 || n == 1)
			return selected_list;

		// Calculate the distance matrix between points
		MatrixXr dist_matrix = computeDistMatrix(tab_P, n);

		// Find the indices of the two most separated points
		RowVectorXr max_vals(n);
		RowVectorXi max_indices(n);
		for (int i = 0; i < n; i++)
		{
			max_vals(i) = dist_matrix.col(i).maxCoeff(&max_indices(i));
		}

		int max_col;
		max_vals.maxCoeff(&max_col);
		int idx1 = max_indices(max_col);
		int idx2 = max_col;

		// Initially, the selected list contains the two most distant points
		selected_list.resize(2);
		selected_list << idx1, idx2;
		int selected_count = 2;

		int max_min_index = 0;
		while (selected_count < K)
		{
			float current_max_min_dist = 0.0f;
			for (int i = 0; i < n; i++)
			{
				// If the i-th point is not already selected
				if (!((selected_list.array() == i).any()))
				{
					float min_dist = HUGE_VALF;
					for (int j = 0; j < selected_count; j++)
					{
						int current_cluster = selected_list(j);
						if (dist_matrix(i, current_cluster) < min_dist)
						{
							min_dist = dist_matrix(i, current_cluster);
						}
					}
					if (min_dist > current_max_min_dist)
					{
						current_max_min_dist = min_dist;
						max_min_index = i;
					}
				}
			}

			selected_list.conservativeResize(selected_count + 1);
			selected_list(selected_count) = max_min_index;
			selected_count++;
		}

		return selected_list;
	}

	MatrixXr KmeansMod::computeDistMatrix(const Matrix3Xr& tab_P, int n)
	{
		// Function that computes the distance matrix between each pair of points in a set.
		MatrixXr dist_matrix = MatrixXr::Zero(n, n);

		float dist;
		for (int i = 0; i < n; i++)
		{
			for (int j = i + 1; j < n; j++)
			{
				dist = (tab_P.col(i) - tab_P.col(j)).norm();
				dist_matrix(i, j) = dist;
				dist_matrix(j, i) = dist;
			}
		}

		return dist_matrix;
	}

	RowVectorXi KmeansMod::assignClusters(const Matrix3Xr& tab_P, const Matrix3Xr& centroids, int n, int K)
	{
		RowVectorXi assignments = RowVectorXi::Zero(n);

		for (int i = 0; i < n; i++)
		{
			float min_dist = HUGE_VALF;
			int best_cluster = 0;

			for (int j = 0; j < K; j++)
			{
				float dist = (tab_P.col(i) - centroids.col(j)).norm();
				if (dist < min_dist)
				{
					min_dist = dist;
					best_cluster = j;
				}
			}

			assignments(i) = best_cluster;
		}

		return assignments;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CONSISTENT CLUSTERING: Compute consistent centroids
	// ════════════════════════════════════════════════════════════════════════════

	KmeansMod::ClusterData KmeansMod::reorderCentroidsConsistently(const ClusterData& prev_data, const ClusterData& new_data, int n, int K)
	{
		// Function that reorders the centroids to ensure consistency in cluster numbering.
		// Create output
		ClusterData data(n, K);

		// Ensure consistency in cluster numbering
		RowVectorXi centroid_assignments = associateCentroids(prev_data.centroids, new_data.centroids, K);

		// Set reordered centroids
		for (int i = 0; i < K; i++)
		{
			data.centroids.col(i) = new_data.centroids.col(centroid_assignments(i));
		}

		// Update point assignments based on reordered centroids
		for (int i = 0; i < n; i++)
		{
			data.assignments(i) = centroid_assignments(prev_data.assignments(i));
		}

		return data;
	}

	RowVectorXi KmeansMod::associateCentroids(const Matrix3Xr& tab_P1, const Matrix3Xr& tab_P2, int K)
	{
		RowVectorXi assignments(K);

		// Calculate the distance matrix between elements of both groups
		MatrixXr dist_matrix = computeDistMatrixTwoGroups(tab_P1, tab_P2, K);

		// Hungarian algorithm
		// Convert the Eigen distance matrix to a vector of vectors for the Hungarian Algorithm
		std::vector<std::vector<double>> cost_matrix(K, std::vector<double>(K));
		for (int i = 0; i < K; i++)
		{
			for (int j = 0; j < K; j++)
			{
				cost_matrix[i][j] = static_cast<double>(dist_matrix(i, j));
			}
		}

		// Execute the Hungarian Algorithm
		HungarianAlgorithm hung_algo_obj;
		std::vector<int> assigned_idx;
		hung_algo_obj.Solve(cost_matrix, assigned_idx);

		// Convert the assignment result to a RowVectorXi
		for (int i = 0; i < K; i++)
		{
			assignments(i) = assigned_idx[i];
		}

		return assignments;
	}

	MatrixXr KmeansMod::computeDistMatrixTwoGroups(const Matrix3Xr& tab_P1, const Matrix3Xr& tab_P2, int K)
	{
		// Function that provides the distance matrix between points of two groups.
		// Unlike 'ComputeDistMatrix()', which calculated distances between all points in a single set
		MatrixXr dist_matrix = MatrixXr::Zero(K, K);

		// Calculate squared Euclidean distances
		for (int i = 0; i < K; i++)
		{
			for (int j = 0; j < K; j++)
			{
				dist_matrix(i, j) = std::pow((tab_P1.col(i) - tab_P2.col(j)).norm(), 2.0f);
			}
		}
		return dist_matrix;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// PERSISTENCE: Ensure persistence of centroids
	// ════════════════════════════════════════════════════════════════════════════

	KmeansMod::ClusterData KmeansMod::ensureClusterPersistence(const Matrix3Xr& tab_P, const ClusterData& prev_data, const ClusterData& new_data, int n, int K, float dt)
	{
		// Function that ensures the persistence of centroids over time
		// Create output
		ClusterData data(n, K);

		// Calculate costs for previous and current assignments
		// Calculate optimality cost
		RowVectorXr sum_dist_prev = RowVectorXr::Zero(K);
		RowVectorXr sum_dist_curr = RowVectorXr::Zero(K);
		for (int c = 0; c < K; c++)
		{
			for (int p = 0; p < n; p++)
			{
				if (prev_data.assignments(p) == c)
				{
					sum_dist_prev(c) += (tab_P.col(p) - prev_data.centroids.col(c)).norm();
				}
				if (new_data.assignments(p) == c)
				{
					sum_dist_curr(c) += (tab_P.col(p) - new_data.centroids.col(c)).norm();
				}
			}
		}
		float opt_cost_prev = sum_dist_prev.sum() / n;
		float opt_cost_curr = sum_dist_curr.sum() / n;

		// Calculate cooldown cost
		float cooldown_slope = -40.0f;	 // Slope for cooldown
		float initial_cooldown = 200.0f; // Initial cooldown cost
		float cooldown_cost = initial_cooldown + cooldown_time_ * cooldown_slope;
		if (cooldown_cost < 0.0f)
			cooldown_cost = 0.0f;
		// Update cooldown time
		cooldown_time_ += dt;

		// Calculate switch cost
		RowVectorXr sum_centroid_dist(K);
		for (int c = 0; c < K; c++)
		{
			sum_centroid_dist(c) = (new_data.centroids.col(c) - prev_data.centroids.col(c)).norm();
		}
		float switch_cost = sum_centroid_dist.sum() / K;

		// Calculate bonding cost
		float bonding_slope = -5.0f / n; // Slope for bonding
		RowVectorXr bonding_cost_per_point = RowVectorXr::Zero(n);
		for (int p = 0; p < n; p++)
		{
			if (new_data.assignments(p) != prev_data.assignments(p))
			{
				if (prev_calculated_assignments_(p) == new_data.assignments(p))
				{
					bonding_time_(p) += dt;
					bonding_cost_per_point(p) = bonding_time_(p) * bonding_slope;
				}
				else
				{
					bonding_time_(p) = 0.0f;
					bonding_cost_per_point(p) = 0.0f;
				}
			}
			else
			{
				bonding_cost_per_point(p) = 0.0f;
				bonding_time_(p) = 0.0f;
			}
		}
		float bonding_cost = bonding_cost_per_point.sum();
		if (std::abs(w_bond_ * bonding_cost) > (w_switch_ * switch_cost * 0.75f))
		{
			bonding_cost = -((w_switch_ * switch_cost) * 0.75f) / w_bond_;
		}

		// Multiply costs by their weights and sum them
		float total_cost_prev = w_opt_ * opt_cost_prev;
		float total_cost_curr = w_opt_ * opt_cost_curr + w_cool_ * cooldown_cost + bonding_cost + switch_cost;

		// Store assignments calculted in this iteration
		prev_calculated_assignments_ = new_data.assignments;

		// Compare total costs and choose the distribution with the lower cost
		if (total_cost_prev <= total_cost_curr)
		{
			return prev_data;
		}
		else
		{
			// Reset temporary parameters if there is a change in distribution
			cooldown_time_ = 0.0f;
			bonding_time_.setZero();

			return new_data;
		}
	}

} // namespace flychams::perception
