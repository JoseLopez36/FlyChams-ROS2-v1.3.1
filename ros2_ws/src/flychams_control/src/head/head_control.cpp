#include "flychams_control/head/head_control.hpp"

using namespace flychams::core;

namespace flychams::control
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void HeadControl::onInit()
	{
		// Get parameters from parameter server
		// Get update rate
		update_rate_ = RosUtils::getParameterOr<float>(node_, "head_control.control_update_rate", 20.0f);

		// Initialize agent state
		curr_state_ = AgentState::IDLE;
		has_state_ = false;

		// Initialize tracking goal
		goal_ = TrackingGoalMsg();
		has_goal_ = false;

		// Compute central head command
		const auto& central_head_ptr = config_tools_->getCentralHead(agent_id_);
		central_cmd_.id = central_head_ptr->id;
		const auto& central_head_rpy = Vector3r(central_head_ptr->orientation.x(), 0.0f, central_head_ptr->orientation.z());
		RosUtils::toMsg(MathUtils::eulerToQuaternion(central_head_rpy), central_cmd_.ori);
		central_cmd_.fov = MathUtils::computeFov(central_head_ptr->ref_focal, central_head_ptr->camera.sensor_width);

		// Create callback group
		callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		auto sub_options = rclcpp::SubscriptionOptions();
		sub_options.callback_group = callback_group_;

		// Subscribe to state, odom and goal topics
		state_sub_ = topic_tools_->createAgentStateSubscriber(agent_id_,
			std::bind(&HeadControl::stateCallback, this, std::placeholders::_1), sub_options);
		goal_sub_ = topic_tools_->createAgentTrackingGoalSubscriber(agent_id_,
			std::bind(&HeadControl::goalCallback, this, std::placeholders::_1), sub_options);

		// Set update timer
		update_timer_ = RosUtils::createTimer(node_, update_rate_,
			std::bind(&HeadControl::update, this), callback_group_);
	}

	void HeadControl::onShutdown()
	{
		// Destroy subscribers
		state_sub_.reset();
		goal_sub_.reset();
		// Destroy update timer
		update_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void HeadControl::stateCallback(const core::AgentStateMsg::SharedPtr msg)
	{
		// Update current state
		curr_state_ = static_cast<AgentState>(msg->state);
		has_state_ = true;
	}

	void HeadControl::goalCallback(const core::TrackingGoalMsg::SharedPtr msg)
	{
		// Update tracking goal
		goal_ = *msg;
		has_goal_ = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update heads
	// ════════════════════════════════════════════════════════════════════════════

	void HeadControl::update()
	{
		// Check if we have a valid state
		if (!has_state_)
		{
			RCLCPP_WARN(node_->get_logger(), "Head control: No state data received for agent %s",
				agent_id_.c_str());
			return;
		}

		// Check if we are in the correct state to move
		if (curr_state_ != AgentState::TRACKING)
		{
			RCLCPP_WARN(node_->get_logger(), "Head control: Agent %s is not in the correct state to control heads",
				agent_id_.c_str());
			return;
		}

		// Create command vectors and initialize with central head command
		std::vector<ID> head_ids = { central_cmd_.id };
		std::vector<QuaternionMsg> head_orientations = { central_cmd_.ori };
		std::vector<float> head_fovs = { central_cmd_.fov };

		// Check if tracking goal is set
		if (has_goal_)
		{
			int n = static_cast<int>(goal_.head_ids.size());
			for (int i = 0; i < n; i++)
			{
				// Add tracking command to vectors
				head_ids.push_back(goal_.head_ids[i]);
				head_orientations.push_back(goal_.orientations[i]);
				head_fovs.push_back(goal_.fovs[i]);
			}
		}

		// Send commands to heads
		framework_tools_->setGimbalOrientations(agent_id_, head_ids, head_orientations);
		framework_tools_->setCameraFovs(agent_id_, head_ids, head_fovs);
	}

} // namespace flychams::control