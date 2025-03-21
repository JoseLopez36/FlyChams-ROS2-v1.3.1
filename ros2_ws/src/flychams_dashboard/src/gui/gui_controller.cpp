#include "flychams_dashboard/gui/gui_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::dashboard
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    void GuiController::onInit()
    {
        // Get parameters from parameter server
        // Get windows IDs
        fixed_window_ids_ = RosUtils::getParameter<IDs>(node_, "fixed_window_ids");
        dynamic_window_ids_ = RosUtils::getParameter<IDs>(node_, "dynamic_window_ids");
        num_fixed_windows_ = static_cast<int>(fixed_window_ids_.size());
        num_dynamic_windows_ = static_cast<int>(dynamic_window_ids_.size());
        // Get fixed camera IDs
        fixed_camera_ids_ = RosUtils::getParameter<IDs>(node_, "fixed_camera_ids");

        // Replace fixed camera placeholders with agent ID
        for (auto& camera_id : fixed_camera_ids_)
            camera_id = RosUtils::replacePlaceholder(camera_id, "AGENTID", agent_id_);

        // Get central camera ID
        central_camera_id_ = config_tools_->getAgent(agent_id_)->central_head_id;

        // Set each fixed window parameters
        fixed_vehicle_id_cmds_.resize(num_fixed_windows_);
        fixed_camera_id_cmds_.resize(num_fixed_windows_);
        fixed_crop_x_cmds_.resize(num_fixed_windows_);
        fixed_crop_y_cmds_.resize(num_fixed_windows_);
        fixed_crop_w_cmds_.resize(num_fixed_windows_);
        fixed_crop_h_cmds_.resize(num_fixed_windows_);
        for (int i = 0; i < num_fixed_windows_; i++)
        {
            fixed_vehicle_id_cmds_[i] = agent_id_;
            fixed_camera_id_cmds_[i] = fixed_camera_ids_[i];
            fixed_crop_x_cmds_[i] = 0;
            fixed_crop_y_cmds_[i] = 0;
            fixed_crop_w_cmds_[i] = 0;
            fixed_crop_h_cmds_[i] = 0;
        }

        // Set each dynamic window parameters
        dynamic_vehicle_id_cmds_.resize(num_dynamic_windows_);
        dynamic_camera_id_cmds_.resize(num_dynamic_windows_);
        dynamic_crop_x_cmds_.resize(num_dynamic_windows_);
        dynamic_crop_y_cmds_.resize(num_dynamic_windows_);
        dynamic_crop_w_cmds_.resize(num_dynamic_windows_);
        dynamic_crop_h_cmds_.resize(num_dynamic_windows_);
        for (int i = 0; i < num_dynamic_windows_; i++)
        {
            dynamic_vehicle_id_cmds_[i] = agent_id_;
            if (i == 0)
                dynamic_camera_id_cmds_[i] = central_camera_id_;
            else
                dynamic_camera_id_cmds_[i] = "";
            dynamic_crop_x_cmds_[i] = 0;
            dynamic_crop_y_cmds_[i] = 0;
            dynamic_crop_w_cmds_[i] = 0;
            dynamic_crop_h_cmds_[i] = 0;
        }

        // Initialize GUI data
        goal_ = TrackingGoalMsg();
        has_goal_ = false;

        // Subscribe to tracking goal
        tracking_sub_ = topic_tools_->createAgentTrackingGoalSubscriber(agent_id_,
            std::bind(&GuiController::trackingCallback, this, std::placeholders::_1));
    }

    void GuiController::onShutdown()
    {
        // Destroy subscriber
        tracking_sub_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // SAFE CALLBACKS: Thread-safe callback functions
    // ════════════════════════════════════════════════════════════════════════════

    void GuiController::trackingCallback(const core::TrackingGoalMsg::SharedPtr msg)
    {
        // Update tracking goal under lock
        std::lock_guard<std::mutex> lock(mutex_);
        goal_ = *msg;
        has_goal_ = true;
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update GUI and listen for user input
    // ════════════════════════════════════════════════════════════════════════════

    void GuiController::setFixedWindows()
    {
        ext_tools_->setWindowImageGroup(fixed_window_ids_, fixed_vehicle_id_cmds_, fixed_camera_id_cmds_, fixed_crop_x_cmds_, fixed_crop_y_cmds_, fixed_crop_w_cmds_, fixed_crop_h_cmds_);
    }

    void GuiController::setTrackingWindows()
    {
        // Get tracking goal under lock
        TrackingGoalMsg goal;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_goal_)
                return;
            goal = goal_;
        }

        // Update tracking command vectors
        const int num_goals = static_cast<int>(goal.unit_types.size());
        for (int i = 0; i < std::min(num_dynamic_windows_, num_goals); i++)
        {
            const auto& goal_type = static_cast<TrackingUnitType>(goal.unit_types[i]);

            switch (goal_type)
            {
            case TrackingUnitType::Physical:
            {
                // Physical tracking unit data
                const ID head_id = goal.head_ids[i];
                // Add tracking window (no crop)
                dynamic_camera_id_cmds_[i] = head_id;
                dynamic_crop_x_cmds_[i] = 0;
                dynamic_crop_y_cmds_[i] = 0;
                dynamic_crop_w_cmds_[i] = 0;
                dynamic_crop_h_cmds_[i] = 0;
            }
            break;

            case TrackingUnitType::Digital:
            {
                // Digital tracking unit data
                const ID camera_id = goal.camera_id;
                const CropMsg crop = goal.crops[i];
                // Check if crop is out of bounds
                if (crop.is_out_of_bounds)
                {
                    dynamic_camera_id_cmds_[i] = "";
                    continue;
                }
                // Add tracking window (with crop)
                dynamic_camera_id_cmds_[i] = camera_id;
                dynamic_crop_x_cmds_[i] = crop.x;
                dynamic_crop_y_cmds_[i] = crop.y;
                dynamic_crop_w_cmds_[i] = crop.w;
                dynamic_crop_h_cmds_[i] = crop.h;
            }
            break;

            default:
            {
                RCLCPP_ERROR(node_->get_logger(), "GuiController: Invalid tracking unit type: %d", static_cast<int>(goal_type));
                // Set empty image to current window
                dynamic_camera_id_cmds_[i] = "";
                dynamic_crop_x_cmds_[i] = 0;
                dynamic_crop_y_cmds_[i] = 0;
                dynamic_crop_w_cmds_[i] = 0;
                dynamic_crop_h_cmds_[i] = 0;
            }
            break;
            };
        }

        // Set tracking view windows
        ext_tools_->setWindowImageGroup(dynamic_window_ids_, dynamic_vehicle_id_cmds_, dynamic_camera_id_cmds_, dynamic_crop_x_cmds_, dynamic_crop_y_cmds_, dynamic_crop_w_cmds_, dynamic_crop_h_cmds_);
    }

} // namespace flychams::dashboard