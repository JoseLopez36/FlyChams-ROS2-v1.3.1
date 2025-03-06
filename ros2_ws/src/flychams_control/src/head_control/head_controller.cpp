#include "flychams_control/head_control/head_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::control
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void HeadController::onInit()
	{
		// Get parameters from parameter server
		// Get update rates
		float update_rate = RosUtils::getParameterOr<float>(node_, "head_control.control_update_rate", 200.0f);

		// Initialize agent data
		goal_ = TrackingGoalMsg();
		has_goal_ = false;

		// Subscribe to goal topic
		goal_sub_ = topic_tools_->createTrackingGoalSubscriber(agent_id_,
			std::bind(&HeadController::goalCallback, this, std::placeholders::_1));

		// Set update timer
		control_timer_ = RosUtils::createTimerByRate(node_, update_rate,
			std::bind(&HeadController::updateControl, this));
	}

	void HeadController::onShutdown()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		// Destroy subscribers
		goal_sub_.reset();
		// Destroy update timer
		control_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void HeadController::goalCallback(const core::TrackingGoalMsg::SharedPtr msg)
	{
		// Get target position
		std::lock_guard<std::mutex> lock(mutex_);
		goal_ = *msg;
		has_goal_ = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update PID controllers
	// ════════════════════════════════════════════════════════════════════════════

	void HeadController::updateControl()
	{
		std::lock_guard<std::mutex> lock(mutex_);

		// Check if goal is set
		if (!has_goal_)
		{
			RCLCPP_WARN(node_->get_logger(), "Head controller: No goal set, skipping update");
			return;
		}

		// Command tracking heads to move to given angles and fovs
		if (goal_.head_ids.size() >= 1)
		{
			ext_tools_->setGimbalAngles(agent_id_, goal_.head_ids, goal_.orientations, tf_tools_->getWorldFrame());
			ext_tools_->setCameraFovs(agent_id_, goal_.head_ids, goal_.fovs, tf_tools_->getWorldFrame());
		}
		else
		{
			RCLCPP_WARN(node_->get_logger(), "Head controller: No head ids set, skipping update");
		}
	}

} // namespace flychams::control