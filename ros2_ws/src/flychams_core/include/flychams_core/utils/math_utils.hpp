#pragma once

// Standard includes
#include <cmath>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Core includes
#include "flychams_core/types/core_types.hpp"

namespace flychams::core
{
    class MathUtils
    {
    public:
        // ════════════════════════════════════════════════════════════════════════════
        // ANGLE: Angle utilities
        // ════════════════════════════════════════════════════════════════════════════

        static float degToRad(float degrees)
        {
            return degrees * M_PIf / 180.0f;
        }

        static float radToDeg(float radians)
        {
            return radians * 180.0f / M_PIf;
        }

        static float normalizeAngle(float angle)
        {
            // Normalize angle to [-π, π]
            return std::atan2(std::sin(angle), std::cos(angle));
        }

        // ════════════════════════════════════════════════════════════════════════════
        // VECTOR: Vector utilities
        // ════════════════════════════════════════════════════════════════════════════

        static float distance(const Vector3r& p1, const Vector3r& p2)
        {
            return (p1 - p2).norm();
        }

        static Vector3r direction(const Vector3r& from, const Vector3r& to)
        {
            Vector3r dir = to - from;
            if (dir.norm() < 1e-6) {
                return Vector3r::Zero();
            }
            return dir.normalized();
        }

        // ════════════════════════════════════════════════════════════════════════════
        // GEOMETRY: Geometry utilities
        // ════════════════════════════════════════════════════════════════════════════

        static Vector3r quaternionToEuler(const Quaternionr& ori)
        {
            return ori.toRotationMatrix().eulerAngles(0, 1, 2);
        }

        static Quaternionr eulerToQuaternion(const Vector3r& euler)
        {
            Eigen::AngleAxisf rollAngle(euler.x(), Vector3r::UnitX());
            Eigen::AngleAxisf pitchAngle(euler.y(), Vector3r::UnitY());
            Eigen::AngleAxisf yawAngle(euler.z(), Vector3r::UnitZ());

            return yawAngle * pitchAngle * rollAngle;
        }

        static Matrix3r quaternionToRotationMatrix(const Quaternionr& ori)
        {
            return ori.toRotationMatrix();
        }

        static Quaternionr rotationMatrixToQuaternion(const Matrix3r& rot)
        {
            return Quaternionr(rot);
        }

        // ════════════════════════════════════════════════════════════════════════════
        // CAMERA: Camera utilities
        // ════════════════════════════════════════════════════════════════════════════

        static float computeFov(float focal, float sensor_width)
        {
            return 2.0f * std::atan((sensor_width / 2.0f) / focal);
        }

        static Vector2r projectPoint(const Vector3r& wP, const Matrix4r& wTc, const Matrix3r& K)
        {
            // Args:
            // - wP: World point
            // - wTc: World to camera transform
            // - K: Camera intrinsic matrix

            // Get transform from camera to world
            const Matrix4r cTw = wTc.inverse();

            // Get homogeneous point
            const Vector4r wP_ = wP.homogeneous();

            // Project point
            const Vector3r p = K * cTw.block<3, 4>(0, 0) * wP_;

            // Get projected coordinates
            const float u = p.x() / p.z();
            const float v = p.y() / p.z();

            // Return projected point
            return Vector2r(u, v);
        }
    };

}  // namespace flychams::core