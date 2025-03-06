#pragma once

// Standard includes
#include <mutex>

// PID controller include
#include "flychams_control/agent_control/pid_controller.hpp"

// Base module include
#include "flychams_core/base/base_module.hpp"

namespace flychams::control
{
	/**
	 * ════════════════════════════════════════════════════════════════
	 * @brief Controller for UAV agent position and yaw
	 *
	 * @details
	 * This class is responsible for controlling the position and yaw
	 * of UAV agents using PID controllers. It manages multiple agents
	 * and their respective control parameters.
	 *
	 * ════════════════════════════════════════════════════════════════
	 * @author Jose Francisco Lopez Ruiz
	 * @date 2025-01-29
	 * ════════════════════════════════════════════════════════════════
	 */
	class AgentController : public core::BaseModule
	{
	public: // Constructor/Destructor
		AgentController(const core::ID& agent_id, core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::ExternalTools::SharedPtr ext_tools, core::TopicTools::SharedPtr topic_tools, core::TfTools::SharedPtr tf_tools)
			: BaseModule(node, config_tools, ext_tools, topic_tools, tf_tools), agent_id_(agent_id)
		{
			init();
		}

	protected: // Overrides
		void onInit() override;
		void onShutdown() override;

	public: // Types
		using SharedPtr = std::shared_ptr<AgentController>;
		enum class State
		{
			Landed,         // Agent is landed
			Takeoff, 		// Agent will takeoff 
			Takingoff, 		// Agent is taking off
			Hover,   		// Agent will hover
			Hovering,       // Agent is hovering
			MovingToGoal,   // Agent is moving to an assigned goal
			Land,  		    // Agent will land
			Landing         // Agent is landing
		};

	private: // Parameters
		core::ID agent_id_;
		float goal_tolerance_;
		float vel_cmd_duration_;

	private: // Data
		// Agent state
		State state_;
		// Odom
		core::Vector3r curr_pos_; 	// Current position (x, y, z)
		core::Vector3r curr_vel_; 	// Current linear velocity (vx, vy, vz)
		bool has_odom_;
		// Goal
		core::Vector3r target_pos_; // Target position (x, y, z)
		bool has_goal_;
		// State, odom and goal mutex
		std::mutex mutex_;
		// PID controllers
		PIDController pid_x_;
		PIDController pid_y_;
		PIDController pid_z_;
		// Time data
		core::Time prev_time_;

	private: // Methods
		// Callbacks
		void odomCallback(const core::OdometryMsg::SharedPtr msg);
		void goalCallback(const core::AgentGoalMsg::SharedPtr msg);
		// Update
		void updateControl();
		// Helper methods
		core::Vector3r computeVelocityCommand(const core::Vector3r& curr_pos, const core::Vector3r& target_pos);

	private:
		// Subscribers
		core::SubscriberPtr<core::OdometryMsg> odom_sub_;
		core::SubscriberPtr<core::AgentGoalMsg> goal_sub_;
		// Timers
		core::TimerPtr control_timer_;
	};

} // namespace flychams::control