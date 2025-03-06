#include "rclcpp/rclcpp.hpp"

// Target includes
#include "flychams_bringup/target/target_controller.hpp"

// Core includes
#include "flychams_core/base/registrator_node.hpp"

using namespace flychams::core;
using namespace flychams::bringup;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Target registration node for registering the different targets
 * in the simulation
 *
 * @details
 * This class implements the target registration node for registering the
 * different targets in the simulation. It uses the registrator node to
 * register the different targets, so that they can be discovered by the
 * different nodes. It also creates the target controllers and
 * initializes/updates their trajectories.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class TargetRegistrationNode : public RegistratorNode
{
public: // Constructor/Destructor
    TargetRegistrationNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : RegistratorNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Cycle through all target groups in the config
        std::vector<float> target_scales;
        std::vector<TargetType> target_types;
        for (const auto& [id, config_ptr] : config_tools_->getGroups())
        {
            // Extract number of targets in group
            const int num_targets = config_ptr->target_count;

            // Cycle through all targets in group and register them
            for (int i = 0; i < num_targets; i++)
            {
                // Generate target ID
                std::stringstream ss;
                ss << id << "_TARGET" << std::setw(2) << std::setfill('0') << i;
                const ID target_id = ss.str();

                // Create target controller
                auto target_controller = std::make_shared<TargetController>(target_id, node_, config_tools_, ext_tools_, topic_tools_, tf_tools_);
                target_controller->init();

                // Parse target trajectory
                const std::string& trajectory_root = RosUtils::getParameter<std::string>(node_, "trajectory_root");
                const std::string& trajectory_folder = config_ptr->trajectory_folder;
                const std::string& trajectory_path = trajectory_root + trajectory_folder + "/TRAJ" + std::to_string(i + 1) + ".csv";
                target_controller->initializeTrajectory(trajectory_path);

                // Register target
                registerTarget(target_id);

                // Add target to vectors
                target_controllers_.push_back(target_controller);
                target_ids_.push_back(target_id);
                target_poses_.push_back(target_controller->getPose());
                target_scales.push_back(1.0f);
                target_types.push_back(TargetType::Human);
            }
        }

        // Spawn targets
        ext_tools_->spawnObjectGroup(target_ids_, target_poses_, target_scales, target_types);

        // Set target update timer
        prev_time_ = RosUtils::getTimeNow(node_);
        const auto& update_rate = RosUtils::getParameter<float>(node_, "target_registration.target_update_rate");
        update_timer_ = RosUtils::createTimerByRate(node_, update_rate,
            std::bind(&TargetRegistrationNode::onUpdate, this));
    }

    void onShutdown() override
    {
        // Reset target update components
        update_timer_.reset();
        target_controllers_.clear();
        target_ids_.clear();
        target_poses_.clear();
    }

private: // Methods
    void onUpdate()
    {
        // Get current time
        const auto current_time = RosUtils::getTimeNow(node_);
        const float dt = (current_time - prev_time_).seconds();
        prev_time_ = current_time;

        // Cycle through all target controllers
        for (int i = 0; i < target_controllers_.size(); i++)
        {
            // Update target controller
            target_controllers_[i]->updateControl(dt);

            // Update target pose
            target_poses_[i] = target_controllers_[i]->getPose();
        }

        // Command targets to move to given poses
        ext_tools_->setObjectPoseGroup(target_ids_, target_poses_, tf_tools_->getWorldFrame());
    }

private: // Components
    // Target update
    std::vector<ID> target_ids_;
    std::vector<PoseMsg> target_poses_;
    std::vector<TargetController::SharedPtr> target_controllers_;
    Time prev_time_;
    TimerPtr update_timer_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create and initialize node
    auto node = std::make_shared<TargetRegistrationNode>("target_registration_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}