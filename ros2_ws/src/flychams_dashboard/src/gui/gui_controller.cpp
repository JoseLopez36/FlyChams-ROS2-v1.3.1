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
        // Get update rates
        float gui_update_rate = RosUtils::getParameterOr<float>(node_, "gui.gui_update_rate", 10.0f);
        float control_panel_rate = RosUtils::getParameterOr<float>(node_, "gui.control_panel_rate", 20.0f);
        // Get window IDs
        scene_window_id_ = RosUtils::getParameter<core::ID>(node_, "window_ids.scene_id");
        agent_window_id_ = RosUtils::getParameter<core::ID>(node_, "window_ids.agent_id");
        central_window_id_ = RosUtils::getParameter<core::ID>(node_, "window_ids.central_id");
        tracking_window_ids_ = RosUtils::getParameter<std::vector<core::ID>>(node_, "window_ids.tracking_ids");
        num_tracking_windows_ = RosUtils::getParameter<int>(node_, "tracking.num_tracking_windows");
        if (tracking_window_ids_.size() != num_tracking_windows_)
        {
            RCLCPP_ERROR(node_->get_logger(), "GuiController: Tracking IDs size (%d) does not match number of tracking windows (%d)", static_cast<int>(tracking_window_ids_.size()), num_tracking_windows_);
            rclcpp::shutdown();
            return;
        }
        num_windows_ = 3 + num_tracking_windows_;

        // Initialize GUI data
        goal_ = TrackingGoalMsg();
        has_goal_ = false;
        selected_agent_id_ = "NONE";
        agent_ids_.clear();

        // Set update timers
        gui_control_timer_ = RosUtils::createWallTimerByRate(node_, gui_update_rate,
            std::bind(&GuiController::updateControl, this));
    }

    void GuiController::onShutdown()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Destroy data
        goal_ = TrackingGoalMsg();
        selected_agent_id_ = "NONE";
        agent_ids_.clear();
        // Destroy subscribers
        tracking_sub_.reset();
        // Destroy update timers
        gui_control_timer_.reset();
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
    // SAFE ADDERS: Thread-safe adders
    // ════════════════════════════════════════════════════════════════════════════

    void GuiController::addAgent(const core::ID& agent_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Add agent to set
        agent_ids_.insert(agent_id);
        // If no agent selected, select this one
        if (selected_agent_id_ == "NONE")
        {
            changeToAgent(agent_id);
            RCLCPP_WARN(node_->get_logger(), "Selected first agent: %s", selected_agent_id_.c_str());
        }
    }

    void GuiController::removeAgent(const core::ID& agent_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Remove agent from set
        agent_ids_.erase(agent_id);
        if (selected_agent_id_ == agent_id)
        {
            changeToAgent("NONE");
            RCLCPP_WARN(node_->get_logger(), "Selected agent removed: %s", selected_agent_id_.c_str());
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE: Update GUI and listen for user input
    // ════════════════════════════════════════════════════════════════════════════

    void GuiController::updateControl()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Create command vectors
        int window_idx = 0;
        IDs window_ids(num_windows_);
        IDs vehicle_ids(num_windows_);
        IDs camera_ids(num_windows_);
        std::vector<int> crop_x(num_windows_);
        std::vector<int> crop_y(num_windows_);
        std::vector<int> crop_w(num_windows_);
        std::vector<int> crop_h(num_windows_);

        // Set scene view window
        window_ids[window_idx] = scene_window_id_;
        vehicle_ids[window_idx] = "";
        camera_ids[window_idx] = scene_camera_id_;
        crop_x[window_idx] = 0;
        crop_y[window_idx] = 0;
        crop_w[window_idx] = 0;
        crop_h[window_idx] = 0;
        window_idx++;

        // Set agent windows
        if (selected_agent_id_ != "NONE" && !selected_agent_id_.empty())
        {
            // Set agent view window (no crop)
            window_ids[window_idx] = agent_window_id_;
            vehicle_ids[window_idx] = selected_agent_id_;
            camera_ids[window_idx] = RosUtils::replacePlaceholder(agent_camera_id_pattern_, "AGENTID", selected_agent_id_);
            crop_x[window_idx] = 0;
            crop_y[window_idx] = 0;
            crop_w[window_idx] = 0;
            crop_h[window_idx] = 0;
            window_idx++;

            // Set central view window (no crop)
            window_ids[window_idx] = central_window_id_;
            vehicle_ids[window_idx] = selected_agent_id_;
            camera_ids[window_idx] = config_tools_->getAgent(selected_agent_id_)->central_head_id;
            crop_x[window_idx] = 0;
            crop_y[window_idx] = 0;
            crop_w[window_idx] = 0;
            crop_h[window_idx] = 0;
            window_idx++;

            // Set tracking view windows
            if (has_goal_)
                setTrackingWindows(goal_, selected_agent_id_, window_idx, window_ids, vehicle_ids, camera_ids, crop_x, crop_y, crop_w, crop_h);
        }

        // Set target view windows
        ext_tools_->setWindowImageGroup(window_ids, vehicle_ids, camera_ids, crop_x, crop_y, crop_w, crop_h);
    }

    void GuiController::setTrackingWindows(const TrackingGoalMsg& goal, const core::ID& selected_agent_id, int& window_idx, IDs& window_ids, IDs& vehicle_ids, IDs& camera_ids, std::vector<int>& crop_x, std::vector<int>& crop_y, std::vector<int>& crop_w, std::vector<int>& crop_h)
    {
        // Cycle through all tracking windows
        for (size_t i = 0; i < num_tracking_windows_; i++)
        {
            const auto& goal_window_id = goal.window_ids[i];
            const auto& goal_type = static_cast<TrackingUnitType>(goal.unit_types[i]);

            switch (goal_type)
            {
            case TrackingUnitType::Physical:
            {
                // Physical tracking unit data
                const ID head_id = goal.head_ids[i];
                // Add tracking window (no crop)
                window_ids[window_idx] = goal_window_id;
                vehicle_ids[window_idx] = selected_agent_id;
                camera_ids[window_idx] = head_id;
                crop_x[window_idx] = 0;
                crop_y[window_idx] = 0;
                crop_w[window_idx] = 0;
                crop_h[window_idx] = 0;
                window_idx++;
            }
            break;

            case TrackingUnitType::Digital:
            {
                // Digital tracking unit data
                const ID camera_id = goal.camera_id;
                const CropMsg crop = goal.crops[i];
                // Add tracking window (with crop)
                window_ids[window_idx] = goal_window_id;
                vehicle_ids[window_idx] = selected_agent_id;
                camera_ids[window_idx] = camera_id;
                crop_x[window_idx] = crop.x;
                crop_y[window_idx] = crop.y;
                crop_w[window_idx] = crop.w;
                crop_h[window_idx] = crop.h;
                window_idx++;
            }
            break;

            default:
            {
                RCLCPP_ERROR(node_->get_logger(), "GuiController: Invalid tracking unit type: %d", static_cast<int>(goal_type));
                // Set empty image to current window
                window_ids[window_idx] = goal_window_id;
                vehicle_ids[window_idx] = "";
                camera_ids[window_idx] = "";
                crop_x[window_idx] = 0;
                crop_y[window_idx] = 0;
                crop_w[window_idx] = 0;
                crop_h[window_idx] = 0;
                window_idx++;
            }
            break;
            };
        }
    }

    // ════════════════════════════════════════════════════════════════════════════
    // METHODS: Helper methods
    // ════════════════════════════════════════════════════════════════════════════

    void GuiController::changeToAgent(const core::ID& agent_id)
    {
        if (agent_id == "NONE" || agent_id.empty())
        {
            selected_agent_id_ = "NONE";
            tracking_sub_.reset();
            RCLCPP_INFO(node_->get_logger(), "No agent selected");
            return;
        }

        // Change to agent
        selected_agent_id_ = agent_id;
        // Create goal
        goal_ = TrackingGoalMsg();
        has_goal_ = false;
        // Subscribe to tracking goal
        tracking_sub_ = topic_tools_->createTrackingGoalSubscriber(agent_id,
            std::bind(&GuiController::trackingCallback, this, std::placeholders::_1));
    }

    void GuiController::nextOrPrevAgent(bool next)
    {
        // If no agents, transition to None state
        if (agent_ids_.empty())
        {
            selected_agent_id_ = "NONE";
            RCLCPP_ERROR(node_->get_logger(), "Agent selection failed. No agents available");
            return;
        }

        // If empty selection, select the first agent available
        if (selected_agent_id_ == "NONE")
        {
            selected_agent_id_ = *agent_ids_.begin();
            RCLCPP_INFO(node_->get_logger(), "Selected first agent: %s", selected_agent_id_.c_str());
            return;
        }

        // Find current agent and move to next/previous
        auto it = agent_ids_.find(selected_agent_id_);
        if (next)
        {
            // Move to next agent, or wrap around to first
            it++;
            if (it == agent_ids_.end())
            {
                // Reached the end, wrap around to beginning
                selected_agent_id_ = *agent_ids_.begin();
            }
            else
            {
                selected_agent_id_ = *it;
            }
        }
        else
        {
            // Move to previous agent, or wrap around to last
            if (it == agent_ids_.begin())
            {
                // At the beginning, wrap around to end
                selected_agent_id_ = *std::prev(agent_ids_.end());
            }
            else
            {
                it--;
                selected_agent_id_ = *it;
            }
        }

        RCLCPP_INFO(node_->get_logger(), "Selected agent: %s", selected_agent_id_.c_str());
    }

} // namespace flychams::dashboard