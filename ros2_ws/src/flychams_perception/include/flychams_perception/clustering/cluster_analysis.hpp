#pragma once

// Standard includes
#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <random>

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/utils/math_utils.hpp"
namespace flychams::perception
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Cluster analysis implementation
	 *
	 * @details
	 * This class implements a cluster analysis.
	 * It provides methods for calculating cluster data (such as centroid or enclosing circle).
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-02-26
	 * ════════════════════════════════════════════════════════════════
	 */
	class ClusterAnalysis
	{
	public: // Types
		// Input
		using Assignments = std::unordered_map<core::ID, core::ID>; // Point ID -> Cluster ID

	public: // Types
		struct Point2D
		{
			float x, y;
		};
		struct Circle
		{
			Point2D C;
			float R;
		};

	public: // Public methods
		static core::Matrix3Xr getClusterPoints(const std::string& cluster_id, const Assignments& assignments, const std::unordered_map<core::ID, core::Vector3r>& points);
		static core::Vector3r computeCentroid(const core::Matrix3Xr& tab_P);
		static Circle computeEnclosingCircle(const core::Matrix3Xr& tab_P, float r_min, float r_margin);

	private: // Implementation
		// Minimal enclosing circle implementation 
		// (based on https://www.geeksforgeeks.org/minimum-enclosing-circle-using-welzls-algorithm/)
		static Circle welzl(const std::vector<Point2D>& P_input);
		static Circle welzlHelper(std::vector<Point2D>& P, std::vector<Point2D> R, int n);
		static Circle minCircleTrivial(std::vector<Point2D>& P);
		static bool isValidCircle(const Circle& c, const std::vector<Point2D>& P);
		static Circle circleFrom(const Point2D& A, const Point2D& B);
		static Circle circleFrom(const Point2D& A, const Point2D& B, const Point2D& C);
		static Point2D getCircleCenter(float bx, float by, float cx, float cy);
		static bool isInside(const Circle& c, const Point2D& p);
		static float distance(const Point2D& a, const Point2D& b);
	};

} // namespace flychams::perception