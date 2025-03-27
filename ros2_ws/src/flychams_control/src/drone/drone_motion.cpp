#include "flychams_control/drone/drone_motion.hpp"

using namespace flychams::core;

namespace flychams::control
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void DroneMotion::onInit()
	{
		// Get parameters from parameter server
		// Get update rate
		update_rate_ = RosUtils::getParameterOr<float>(node_, "drone_motion.motion_update_rate", 10.0f);
		// Get motion mode
		motion_mode_ = static_cast<MotionMode>(RosUtils::getParameterOr<uint8_t>(node_, "drone_motion.motion_mode", 0));
		// Get speed planner parameters
		float min_speed = RosUtils::getParameterOr<float>(node_, "drone_motion.min_speed", 0.5f);
		float max_speed = RosUtils::getParameterOr<float>(node_, "drone_motion.max_speed", 12.0f);
		float min_distance = RosUtils::getParameterOr<float>(node_, "drone_motion.min_distance", 0.20f);
		float max_distance = RosUtils::getParameterOr<float>(node_, "drone_motion.max_distance", 50.0f);
		float max_acceleration = RosUtils::getParameterOr<float>(node_, "drone_motion.max_acceleration", 2.0f);

		// Compute command timeout
		cmd_timeout_ = (1.0f / update_rate_) * 1.25f;

		// Initialize agent state
		curr_state_ = AgentState::IDLE;
		has_state_ = false;

		// Initialize agent position
		curr_position_ = PointMsg();
		has_position_ = false;

		// Initialize goal position
		goal_position_ = PointMsg();
		has_goal_ = false;

		// Set speed planner parameters
		speed_planner_.setParameters(min_speed, max_speed, min_distance, max_distance, max_acceleration);

		// Create callback group
		callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		auto sub_options = rclcpp::SubscriptionOptions();
		sub_options.callback_group = callback_group_;

		// Subscribe to state, odom and goal topics
		state_sub_ = topic_tools_->createAgentStateSubscriber(agent_id_,
			std::bind(&DroneMotion::stateCallback, this, std::placeholders::_1), sub_options);
		odom_sub_ = topic_tools_->createAgentOdomSubscriber(agent_id_,
			std::bind(&DroneMotion::odomCallback, this, std::placeholders::_1), sub_options);
		goal_sub_ = topic_tools_->createAgentPositionGoalSubscriber(agent_id_,
			std::bind(&DroneMotion::goalCallback, this, std::placeholders::_1), sub_options);

		// Set update timer
		last_update_time_ = RosUtils::now(node_);
		update_timer_ = RosUtils::createTimer(node_, update_rate_,
			std::bind(&DroneMotion::update, this), callback_group_);
	}

	void DroneMotion::onShutdown()
	{
		// Destroy subscribers
		state_sub_.reset();
		odom_sub_.reset();
		goal_sub_.reset();
		// Destroy update timer
		update_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void DroneMotion::stateCallback(const core::AgentStateMsg::SharedPtr msg)
	{
		// Update current state
		curr_state_ = static_cast<AgentState>(msg->state);
		has_state_ = true;
	}

	void DroneMotion::odomCallback(const core::OdometryMsg::SharedPtr msg)
	{
		// Update current position
		curr_position_ = msg->pose.pose.position;
		has_position_ = true;
	}

	void DroneMotion::goalCallback(const core::PositionGoalMsg::SharedPtr msg)
	{
		// Update goal position
		goal_position_ = msg->position;
		has_goal_ = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update motion
	// ════════════════════════════════════════════════════════════════════════════

	void DroneMotion::update()
	{
		// Check if we have a valid state, position and goal
		if (!has_state_ || !has_position_ || !has_goal_)
		{
			RCLCPP_WARN(node_->get_logger(), "Drone motion: No state, position or goal data received for agent %s",
				agent_id_.c_str());
			return;
		}

		// Check if we are in the correct state to move
		if (curr_state_ != AgentState::TRACKING)
		{
			RCLCPP_WARN(node_->get_logger(), "Drone motion: Agent %s is not in the correct state to move",
				agent_id_.c_str());
			return;
		}

		// Compute time step
		auto current_time = RosUtils::now(node_);
		float dt = (current_time - last_update_time_).seconds();
		last_update_time_ = current_time;

		// Limit dt to prevent extreme values after pauses
		dt = std::min(dt, cmd_timeout_);

		switch (motion_mode_)
		{
		case MotionMode::POSITION:
			handlePositionMotion(dt);
			break;

		case MotionMode::VELOCITY:
			handleVelocityMotion(dt);
			break;

		default:
			RCLCPP_ERROR(node_->get_logger(), "Drone motion: Invalid motion mode for agent %s",
				agent_id_.c_str());
			break;
		}
	}

	void DroneMotion::handlePositionMotion(const float& dt)
	{
		// Plan speed based on distance to goal and other criteria
		float target_speed = speed_planner_.planSpeed(curr_position_.x, curr_position_.y, curr_position_.z, goal_position_.x, goal_position_.y, goal_position_.z, dt);

		// Send command to move to goal position
		framework_tools_->setPosition(agent_id_, goal_position_.x, goal_position_.y, goal_position_.z, target_speed, dt);
	}

	void DroneMotion::handleVelocityMotion(const float& dt)
	{
		// TODO: Implement velocity motion
	}

} // namespace flychams::control