#pragma once

// Standard includes
#include <mutex>
#include <set>

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
        GuiController(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::ExternalTools::SharedPtr ext_tools, core::TopicTools::SharedPtr topic_tools, core::TfTools::SharedPtr tf_tools)
            : BaseModule(node, config_tools, ext_tools, topic_tools, tf_tools), agent_id_(agent_id)
        {
            init();
        }

    protected: // Overrides
        void onInit() override;
        void onShutdown() override;

    public: // Types
        using SharedPtr = std::shared_ptr<GuiController>;

    private: // Parameters
        // Agent parameters
        core::ID agent_id_;
        // Window IDs
        core::IDs fixed_window_ids_;
        core::IDs dynamic_window_ids_;
        int num_fixed_windows_;
        int num_dynamic_windows_;
        // Camera IDs
        core::IDs fixed_camera_ids_;
        core::ID central_camera_id_;
        // Fixed command vectors
        core::IDs fixed_vehicle_id_cmds_;
        core::IDs fixed_camera_id_cmds_;
        std::vector<int> fixed_crop_x_cmds_;
        std::vector<int> fixed_crop_y_cmds_;
        std::vector<int> fixed_crop_w_cmds_;
        std::vector<int> fixed_crop_h_cmds_;

    private: // Data
        // Agent tracking goal
        core::TrackingGoalMsg goal_;
        bool has_goal_;
        // Tracking goal mutex
        std::mutex mutex_;
        // Dynamic command vectors
        core::IDs dynamic_vehicle_id_cmds_;
        core::IDs dynamic_camera_id_cmds_;
        std::vector<int> dynamic_crop_x_cmds_;
        std::vector<int> dynamic_crop_y_cmds_;
        std::vector<int> dynamic_crop_w_cmds_;
        std::vector<int> dynamic_crop_h_cmds_;

    private: // Safe callbacks
        void trackingCallback(const core::TrackingGoalMsg::SharedPtr msg);

    public: // Public methods
        // Update
        void setFixedWindows();
        void setTrackingWindows();

    private:
        // Subscribers
        core::SubscriberPtr<core::TrackingGoalMsg> tracking_sub_;
    };

} // namespace flychams::dashboard