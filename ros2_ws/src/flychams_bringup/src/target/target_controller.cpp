#include "flychams_bringup/target/target_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::bringup
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void TargetController::onInit()
    {
        // Get parameters from parameter server
        // Get update rate
        float update_rate = RosUtils::getParameterOr<float>(node_, "target_registration.target_update_rate", 20.0f);

        // Initialize target info publisher
        info_pub_ = topic_tools_->createTargetInfoPublisher(target_id_);
    }

    void TargetController::onShutdown()
    {
        // Destroy trajectory
        trajectory_.reset();
        // Destroy publisher
        info_pub_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PUBLIC METHODS: Methods for initializing and updating the target
    // ════════════════════════════════════════════════════════════════════════════

    void TargetController::initializeTrajectory(const std::string& trajectory_path)
    {
        // Initialize trajectory
        trajectory_ = std::make_shared<TargetTrajectory>();
        trajectory_->parse(trajectory_path);

        // Initialize pose
        pose_.position.x = trajectory_->getStartPoint().x;
        pose_.position.y = trajectory_->getStartPoint().y;
        pose_.position.z = trajectory_->getStartPoint().z;
        pose_.orientation.w = 1.0f;
    }

    void TargetController::updateControl(const float& dt)
    {
        // Update target pose
        const auto& curr_point = updatePosition(dt);
        pose_.position.x = curr_point.x;
        pose_.position.y = curr_point.y;
        pose_.position.z = curr_point.z;
        pose_.orientation.w = 1.0f;
    }

    PoseMsg TargetController::getPose() const
    {
        return pose_;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // PRIVATE METHODS: Methods for updating the target
    // ════════════════════════════════════════════════════════════════════════════

    TargetTrajectory::Point TargetController::updatePosition(const float& dt)
    {
        // Update trajectory
        TargetTrajectory::Point point = trajectory_->update(dt);

        // Create target info message
        TargetInfoMsg msg;
        msg.header = RosUtils::createHeader(node_, tf_tools_->getWorldFrame());
        msg.position.x = point.x;
        msg.position.y = point.y;
        msg.position.z = point.z;

        // Publish target info
        info_pub_->publish(msg);

        // Return current point
        return point;
    }

} // namespace flychams::bringup