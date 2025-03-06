#include "flychams_perception/clustering/cluster_analysis.hpp"

using namespace flychams::core;

namespace flychams::perception
{
	// ════════════════════════════════════════════════════════════════════════════
	// PUBLIC: Cluster analysis public methods
	// ════════════════════════════════════════════════════════════════════════════

	core::Matrix3Xr ClusterAnalysis::getClusterPoints(const std::string& cluster_id, const Assignments& assignments, const std::unordered_map<core::ID, core::Vector3r>& points)
	{
		// Get cluster points
		Matrix3Xr tab_P = Matrix3Xr::Zero(3, 0);
		int n = 0;
		for (const auto& [point_id, assigned_cluster_id] : assignments)
		{
			if (assigned_cluster_id == cluster_id)
			{
				tab_P.conservativeResize(3, n + 1);
				tab_P.col(n) = points.at(point_id);
				n++;
			}
		}

		return tab_P;
	}

	Vector3r ClusterAnalysis::computeCentroid(const Matrix3Xr& tab_P)
	{
		if (tab_P.cols() == 0)
			return Vector3r::Zero();

		Vector3r centroid = Vector3r::Zero();
		for (int i = 0; i < static_cast<int>(tab_P.cols()); i++)
		{
			centroid += tab_P.col(i);
		}

		return centroid / static_cast<float>(tab_P.cols());
	}

	ClusterAnalysis::Circle ClusterAnalysis::computeEnclosingCircle(const Matrix3Xr& tab_P, float r_min, float r_margin)
	{
		// Get cluster target count
		int num_points = static_cast<int>(tab_P.cols());

		// Handle edge cases
		if (num_points == 0)
			return { {0.0f, 0.0f}, r_min + r_margin };
		if (num_points == 1)
			return { tab_P(0, 0), tab_P(1, 0), r_min + r_margin };

		// Minimal enclosing circle (Welzl's algorithm)
		std::vector<Point2D> points_welzl;
		points_welzl.reserve(num_points);
		for (int i = 0; i < num_points; i++)
		{
			points_welzl.push_back({ tab_P(0, i), tab_P(1, i) });
		}
		Circle mec = welzl(points_welzl);

		// Set the enclosing radius, ensuring it's at least the minimum radius
		mec.R = std::max(mec.R, r_min) + r_margin;
		return mec;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// IMPLEMENTATION: Cluster analysis
	// ════════════════════════════════════════════════════════════════════════════

	// Euclidean distance between two Point2D
	float ClusterAnalysis::distance(const Point2D& a, const Point2D& b)
	{
		return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
	}

	// Check whether a Point2D lies inside or on the boundaries of the circle
	bool ClusterAnalysis::isInside(const Circle& c, const Point2D& p)
	{
		return distance(c.C, p) <= c.R;
	}

	// Get a circle defined by 3 Point2D
	ClusterAnalysis::Point2D ClusterAnalysis::getCircleCenter(float bx, float by, float cx, float cy)
	{
		float B = bx * bx + by * by;
		float C = cx * cx + cy * cy;
		float D = bx * cy - by * cx;
		return { (cy * B - by * C) / (2.0f * D), (bx * C - cx * B) / (2.0f * D) };
	}

	// Return a unique circle that intersects three Point2D
	ClusterAnalysis::Circle ClusterAnalysis::circleFrom(const Point2D& A, const Point2D& B, const Point2D& C)
	{
		Point2D I = getCircleCenter(B.x - A.x, B.y - A.y, C.x - A.x, C.y - A.y);
		I.x += A.x;
		I.y += A.y;
		return { I, distance(I, A) };
	}

	// Return the smallest circle that intersects two Point2D
	ClusterAnalysis::Circle ClusterAnalysis::circleFrom(const Point2D& A, const Point2D& B)
	{
		// Set the center to be the midpoint of A and B
		Point2D C = { (A.x + B.x) / 2.0f, (A.y + B.y) / 2.0f };

		// Set the radius to be half the distance AB
		return { C, distance(A, B) / 2.0f };
	}

	// Check whether a circle encloses the given Point2D points
	bool ClusterAnalysis::isValidCircle(const Circle& c, const std::vector<Point2D>& P)
	{
		// Iterate through all the points to check whether they lie inside the circle
		for (const Point2D& p : P)
			if (!isInside(c, p))
				return false;
		return true;
	}

	// Return the minimum enclosing circle for N <= 3
	ClusterAnalysis::Circle ClusterAnalysis::minCircleTrivial(std::vector<Point2D>& P)
	{
		assert(P.size() <= 3);
		if (P.empty())
		{
			return { {0.0f, 0.0f}, 0.0f };
		}
		else if (P.size() == 1)
		{
			return { P[0], 0.0f };
		}
		else if (P.size() == 2)
		{
			return circleFrom(P[0], P[1]);
		}

		// To check if MEC can be determined by 2 points only
		for (int i = 0; i < 3; i++)
		{
			for (int j = i + 1; j < 3; j++)
			{
				Circle c = circleFrom(P[i], P[j]);
				if (isValidCircle(c, P))
					return c;
			}
		}
		return circleFrom(P[0], P[1], P[2]);
	}

	// Returns the MEC using Welzl's algorithm
	// Takes a set of input Point2D P and a set R of points on the circle boundary.
	// n represents the number of Point2D in P that are not yet processed.
	ClusterAnalysis::Circle ClusterAnalysis::welzlHelper(std::vector<Point2D>& P, std::vector<Point2D> R, int n)
	{
		// Base case when all points are processed or |R| = 3
		if (n == 0 || R.size() == 3)
			return minCircleTrivial(R);

		// Select the last point
		Point2D p = P[n - 1];

		// Find the MEC without the last point
		Circle c = welzlHelper(P, R, n - 1);

		// If p is inside the MEC, return it
		if (isInside(c, p))
			return c;

		// Otherwise, p must be on the boundary of the MEC
		R.push_back(p);

		// Return the MEC for P - {p} and R U {p}
		return welzlHelper(P, R, n - 1);
	}

	// Function to return the smallest enclosing circle using Welzl's algorithm
	ClusterAnalysis::Circle ClusterAnalysis::welzl(const std::vector<Point2D>& P_input)
	{
		// Make a copy of P to allow modifications
		std::vector<Point2D> P = P_input;

		// Shuffle the points randomly to ensure random order
		static std::mt19937 rng(std::random_device{}());
		std::shuffle(P.begin(), P.end(), rng);

		// Call the recursive helper function with an empty boundary set
		std::vector<Point2D> R;
		return welzlHelper(P, R, static_cast<int>(P.size()));
	}

} // namespace flychams::perception
