#pragma once

// Standard includes
#include <mutex>
#include <set>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::dashboard
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Controller for GUI
     *
     * @details
     * This class is responsible for controlling the GUI. It manages
     * multiple agents and their respective control parameters.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-27
     * ════════════════════════════════════════════════════════════════
     */
    class GuiController : public core::BaseModule
    {
    public: // Constructor/Destructor
        GuiController(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::ExternalTools::SharedPtr ext_tools, core::TopicTools::SharedPtr topic_tools, core::TfTools::SharedPtr tf_tools)
            : BaseModule(node, config_tools, ext_tools, topic_tools, tf_tools)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<GuiController>;

    private: // Parameters
        // Window IDs
        core::ID scene_window_id_;
        core::ID agent_window_id_;
        core::ID data_window_id_;
        core::ID map_window_id_;
        core::ID central_window_id_;
        std::vector<core::ID> tracking_window_ids_;
        // Camera IDs (for fixed windows)
        const core::ID scene_camera_id_ = "SCENECAM";
        const core::ID agent_camera_id_pattern_ = "AGENTCAM_AGENTID"; // Agent ID will be substituted for AGENTID
        // Number of windows
        int num_tracking_windows_;
        int num_windows_;

    private: // Data
        // Tracking goal of the selected agent
        core::TrackingGoalMsg goal_;
        bool has_goal_;
        // Selected agent ID
        core::ID selected_agent_id_;
        // Agent IDs
        std::set<core::ID> agent_ids_;
        // State, tracking goal and agent IDs mutex
        std::mutex mutex_;

    private: // Safe callbacks
        void trackingCallback(const core::TrackingGoalMsg::SharedPtr msg);

    public: // Safe adders
        void addAgent(const core::ID& agent_id);
        void removeAgent(const core::ID& agent_id);

    private: // Methods
        // Update
        void updateControl();
        void setTrackingWindows(const core::TrackingGoalMsg& goal, const core::ID& selected_id, int& window_idx, core::IDs& window_ids, core::IDs& vehicle_ids, core::IDs& camera_ids, std::vector<int>& crop_x, std::vector<int>& crop_y, std::vector<int>& crop_w, std::vector<int>& crop_h);
        // Helper methods
        void changeToAgent(const core::ID& agent_id);
        void nextOrPrevAgent(bool next);

    private:
        // Subscribers
        core::SubscriberPtr<core::TrackingGoalMsg> tracking_sub_;
        // Timers
        core::TimerPtr gui_control_timer_;
    };

} // namespace flychams::dashboard