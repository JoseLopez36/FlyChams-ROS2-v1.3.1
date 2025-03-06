#pragma once

// Standard includes
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <memory>


namespace flychams::bringup
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Trajectory parser and updater
     *
     * @details
     * This class implements a trajectory parser and updater.
     * It provides methods for parsing trajectories from a file and
     * updating the target's position.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-01-29
     * ════════════════════════════════════════════════════════════════
     */
    class TargetTrajectory
    {
    public: // Types
        using SharedPtr = std::shared_ptr<TargetTrajectory>;
        struct Point
        {
            float t;
            float x, y, z;
        };

    private: // Data
        int num_points_;
        std::vector<Point> points_; // Chronologically ordered sequence of trajectory points
        int current_idx_;           // Current point index in trajectory
        bool reverse_;              // Chronological direction of motion
        float time_elapsed_;        // Time elapsed since start of trajectory

    public: // Methods
        void parse(const std::string& trajectory_path)
        {
            // Initialize trajectory data
            points_.clear();
            current_idx_ = 0;
            reverse_ = false;

            // Get and open trajectory file
            std::ifstream file(trajectory_path);
            if (!file.is_open())
            {
                std::cerr << "Failed to open trajectory file: " << trajectory_path << std::endl;
                return;
            }

            // Read trajectory file
            std::string line;
            while (std::getline(file, line))
            {
                std::stringstream ss(line);
                std::string value;
                Point point;

                // Read time
                if (std::getline(ss, value, '\t'))
                    point.t = std::stod(value);

                // Read x
                if (std::getline(ss, value, '\t'))
                    point.x = std::stod(value);

                // Read y
                if (std::getline(ss, value, '\t'))
                    point.y = std::stod(value);

                // Read z
                if (std::getline(ss, value, '\t'))
                    point.z = std::stod(value);

                // Add point to trajectory
                points_.push_back(point);
            }

            // Get number of points
            num_points_ = static_cast<int>(points_.size());

            // Close file
            file.close();
        }

        void reset()
        {
            current_idx_ = 0;
            reverse_ = false;
            time_elapsed_ = 0.0f;
        }

        Point update(const float& dt)
        {
            // Check if trajectory is empty
            if (num_points_ == 0)
                return Point();

            // Update time elapsed
            time_elapsed_ += dt;

            // Get next and previous points
            Point next_point;
            Point prev_point;
            if (current_idx_ + 1 >= num_points_)
                next_point = points_[current_idx_];
            else
                next_point = points_[current_idx_ + 1];
            if (current_idx_ - 1 < 0)
                prev_point = points_[current_idx_];
            else
                prev_point = points_[current_idx_ - 1];

            // Find the closest trajectory point based on time
            if (!reverse_)
            {
                if (current_idx_ < num_points_ - 1 && next_point.t <= time_elapsed_)
                {
                    current_idx_++;
                    // If we reach the end, start moving backwards
                    if (current_idx_ == num_points_ - 1)
                    {
                        reverse_ = true;
                    }
                }
            }
            else
            {
                if (current_idx_ > 0 && prev_point.t <= time_elapsed_)
                {
                    current_idx_--;
                    // If we reach the start, start moving forwards again
                    if (current_idx_ == 0)
                    {
                        reverse_ = false;
                    }
                }
            }

            // Return current point
            return points_[current_idx_];
        }

        Point getStartPoint()
        {
            if (points_.empty())
                return Point();
            return points_[0];
        }

        Point getEndPoint()
        {
            if (points_.empty())
                return Point();
            return points_[points_.size() - 1];
        }
    };

} // namespace flychams::bringup