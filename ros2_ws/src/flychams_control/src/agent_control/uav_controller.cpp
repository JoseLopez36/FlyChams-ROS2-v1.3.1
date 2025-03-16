#include "flychams_control/agent_control/uav_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::control
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::onInit()
	{
		// Get parameters from parameter server
		// Get update rates
		update_rate_ = RosUtils::getParameterOr<float>(node_, "uav_control.control_update_rate", 200.0f);
		// Get takeoff altitude
		takeoff_altitude_ = RosUtils::getParameterOr<float>(node_, "uav_control.takeoff_altitude", 5.0f);
		// Get timeouts
		arm_timeout_ = RosUtils::getParameterOr<float>(node_, "uav_control.arm_timeout", 5.0f);
		takeoff_timeout_ = RosUtils::getParameterOr<float>(node_, "uav_control.takeoff_timeout", 10.0f);
		landing_timeout_ = RosUtils::getParameterOr<float>(node_, "uav_control.landing_timeout", 10.0f);
		// Get goal reach threshold
		goal_reach_threshold_ = RosUtils::getParameterOr<float>(node_, "uav_control.goal_reach_threshold", 0.5f);

		// Initialize agent data
		curr_pos_ = PointMsg();
		has_odom_ = false;
		goal_pos_ = PointMsg();
		has_goal_ = false;
		pos_timeout_ = update_rate_ * 1.1f; // 10% more than update rate

		// Initialize state
		setState(State::INITIALIZING);

		// Subscribe to odom and goal topic
		odom_sub_ = topic_tools_->createAgentOdomSubscriber(agent_id_,
			std::bind(&UAVController::odomCallback, this, std::placeholders::_1));
		goal_sub_ = topic_tools_->createAgentPositionGoalSubscriber(agent_id_,
			std::bind(&UAVController::goalCallback, this, std::placeholders::_1));

		// Set update timer
		control_timer_ = RosUtils::createTimerByRate(node_, update_rate_,
			std::bind(&UAVController::update, this));

		// Set state to disarmed after initialization
		setState(State::DISARMED);
	}

	void UAVController::onShutdown()
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		// Destroy subscribers
		odom_sub_.reset();
		goal_sub_.reset();
		// Destroy update timer
		control_timer_.reset();
		// Set state to idle
		setState(State::IDLE);
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::odomCallback(const core::OdometryMsg::SharedPtr msg)
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		// Get current position
		curr_pos_ = msg->pose.pose.position;
		has_odom_ = true;
	}

	void UAVController::goalCallback(const core::PositionGoalMsg::SharedPtr msg)
	{
		// Transition to moving
		if (state_ == State::HOVERING)
		{
			requestMove(msg->position);
		}
	}

	// ════════════════════════════════════════════════════════════════════════════
	// STATE MANAGEMENT: State transition and validation methods
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::setState(const State& new_state)
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		// Check if this is a valid transition
		if (!isValidTransition(state_, new_state))
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Invalid state transition from %d to %d",
				static_cast<int>(state_), static_cast<int>(new_state));
			return;
		}

		// Set state
		State old_state = state_;
		state_ = new_state;

		// Update state entry time
		state_entry_time_ = RosUtils::getTimeNow(node_);

		// Log state transition
		RCLCPP_INFO(node_->get_logger(), "UAV controller: State transition from %d to %d",
			static_cast<int>(old_state), static_cast<int>(new_state));
	}

	bool UAVController::isValidTransition(const State& from, const State& to) const
	{
		// Check state transitions
		switch (from)
		{
		case State::IDLE:
			// From IDLE, we can only go to INITIALIZING
			return to == State::INITIALIZING;

		case State::INITIALIZING:
			// From INITIALIZING, we can go to DISARMED or ERROR
			return to == State::DISARMED || to == State::ERROR;

		case State::DISARMED:
			// From DISARMED, we can go to ARMING, IDLE, or ERROR
			return to == State::ARMING || to == State::IDLE || to == State::ERROR;

		case State::ARMING:
			// From ARMING, we can go to ARMED, DISARMED, or ERROR
			return to == State::ARMED || to == State::DISARMED || to == State::ERROR;

		case State::ARMED:
			// From ARMED, we can go to TAKING_OFF, DISARMED, or ERROR
			return to == State::TAKING_OFF || to == State::DISARMED || to == State::ERROR;

		case State::TAKING_OFF:
			// From TAKING_OFF, we can go to HOVERING, LANDING, or ERROR
			return to == State::HOVERING || to == State::LANDING || to == State::ERROR;

		case State::HOVERING:
			// From HOVERING, we can go to MOVING, LANDING, or ERROR
			return to == State::MOVING || to == State::LANDING || to == State::ERROR;

		case State::MOVING:
			// From MOVING, we can go to REACHED, HOVERING, LANDING, or ERROR
			return to == State::REACHED || to == State::HOVERING || to == State::LANDING || to == State::ERROR;

		case State::REACHED:
			// From REACHED, we can go to HOVERING, MOVING, LANDING, or ERROR
			return to == State::HOVERING || to == State::MOVING || to == State::LANDING || to == State::ERROR;

		case State::LANDING:
			// From LANDING, we can go to DISARMED or ERROR
			return to == State::DISARMED || to == State::ERROR;

		case State::ERROR:
			// From ERROR, we can only go to IDLE (reset)
			return to == State::IDLE;

		default:
			// Unknown state, reject transition
			return false;
		}
	}

	// ════════════════════════════════════════════════════════════════════════════
	// STATE HANDLERS: Methods for handling different states
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::handleStateTransition()
	{
		// Lock mutex
		std::lock_guard<std::mutex> lock(mutex_);

		// Handle state based on current state
		switch (state_)
		{
		case State::IDLE:
			handleIdleState();
			break;

		case State::INITIALIZING:
			handleInitializingState();
			break;

		case State::DISARMED:
			handleDisarmedState();
			break;

		case State::ARMING:
			handleArmingState();
			break;

		case State::ARMED:
			handleArmedState();
			break;

		case State::TAKING_OFF:
			handleTakingOffState();
			break;

		case State::HOVERING:
			handleHoveringState();
			break;

		case State::MOVING:
			handleMovingState();
			break;

		case State::REACHED:
			handleReachedState();
			break;

		case State::LANDING:
			handleLandingState();
			break;

		case State::ERROR:
			handleErrorState();
			break;

		default:
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Unknown state: %d", static_cast<int>(state_));
			break;
		}
	}

	// State handler implementations
	void UAVController::handleIdleState()
	{
		// Nothing to do in IDLE state
	}

	void UAVController::handleInitializingState()
	{
		// Check if initialization is complete
		if ((RosUtils::getTimeNow(node_) - state_entry_time_).seconds() > 2.0)
		{
			// Move to DISARMED state
			setState(State::DISARMED);
		}
	}

	void UAVController::handleDisarmedState()
	{
		// Nothing to do in DISARMED state, waiting for arm request
	}

	void UAVController::handleArmingState()
	{
		// Check if arming has timed out
		if ((RosUtils::getTimeNow(node_) - state_entry_time_).seconds() > arm_timeout_)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Arming timeout");
			setState(State::ERROR);
			return;
		}

		// Check if armed status is confirmed
		// We'll assume it worked after a short delay
		if ((RosUtils::getTimeNow(node_) - state_entry_time_).seconds() > 1.0)
		{
			setState(State::ARMED);
		}
	}

	void UAVController::handleArmedState()
	{
		// Nothing to do in ARMED state, waiting for takeoff request
	}

	void UAVController::handleTakingOffState()
	{
		// Check if takeoff has timed out
		if ((RosUtils::getTimeNow(node_) - state_entry_time_).seconds() > takeoff_timeout_)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Takeoff timeout");
			setState(State::ERROR);
			return;
		}

		// Check if takeoff altitude is reached
		if (has_odom_ && curr_pos_.z >= takeoff_altitude_)
		{
			RCLCPP_INFO(node_->get_logger(), "UAV controller: Takeoff complete. Altitude: %.2f m", curr_pos_.z);
			setState(State::HOVERING);
		}
	}

	void UAVController::handleHoveringState()
	{
		// In hovering state, just maintain current position
		if (has_odom_)
		{
			ext_tools_->hover(agent_id_);
		}
	}

	void UAVController::handleMovingState()
	{
		// Check if goal and odom are set
		if (!has_goal_ || !has_odom_)
		{
			RCLCPP_WARN(node_->get_logger(), "UAV controller: No goal or odom set in MOVING state");
			setState(State::HOVERING);
			return;
		}

		// Calculate distance to goal
		const auto& curr_pos_vec = Vector3r(curr_pos_.x, curr_pos_.y, curr_pos_.z);
		const auto& goal_pos_vec = Vector3r(goal_pos_.x, goal_pos_.y, goal_pos_.z);
		const auto& dist = (goal_pos_vec - curr_pos_vec).norm();

		// Check if goal is reached
		if (dist < goal_reach_threshold_)
		{
			RCLCPP_INFO(node_->get_logger(), "UAV controller: Goal reached. Distance to goal: %.2f m", dist);
			setState(State::REACHED);
			return;
		}

		// Compute velocity for movement
		const auto& pos_vel = 5.0f;

		// Send position command to external tools
		ext_tools_->setPosition(agent_id_, goal_pos_.x, goal_pos_.y, goal_pos_.z, pos_vel, pos_timeout_);
	}

	void UAVController::handleReachedState()
	{
		// In REACHED state, hold position briefly then go to hovering
		if ((RosUtils::getTimeNow(node_) - state_entry_time_).seconds() > 2.0)
		{
			setState(State::HOVERING);
		}
	}

	void UAVController::handleLandingState()
	{
		// Check if landing has timed out
		if ((RosUtils::getTimeNow(node_) - state_entry_time_).seconds() > landing_timeout_)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Landing timeout");
			setState(State::ERROR);
			return;
		}

		// Check if we've landed (near ground level)
		if (has_odom_ && curr_pos_.z < 0.5f)
		{
			RCLCPP_INFO(node_->get_logger(), "UAV controller: Landing complete");
			// Disarm after landing
			ext_tools_->armDisarm(agent_id_, false);
			setState(State::DISARMED);
		}
	}

	void UAVController::handleErrorState()
	{
		// In ERROR state, attempt recovery
		// For safety, try to land if we're not already on the ground
		if (has_odom_ && curr_pos_.z > 0.5f)
		{
			ext_tools_->land(agent_id_);
		}
	}

	// ════════════════════════════════════════════════════════════════════════════
	// PUBLIC STATE REQUESTS: Methods for requesting state transitions
	// ════════════════════════════════════════════════════════════════════════════

	bool UAVController::requestArm()
	{
		// Check if we're in DISARMED state
		if (state_ != State::DISARMED)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot arm from state %d", static_cast<int>(state_));
			return false;
		}

		// Request arming
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Arming...");
		bool result = ext_tools_->armDisarm(agent_id_, true);

		if (result)
		{
			// Transition to ARMING state
			setState(State::ARMING);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Arming failed");
			return false;
		}
	}

	bool UAVController::requestTakeoff()
	{
		// Check if we're in ARMED state
		if (state_ != State::ARMED)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot takeoff from state %d", static_cast<int>(state_));
			return false;
		}

		// Request takeoff
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Taking off...");
		bool result = ext_tools_->takeoff(agent_id_);

		if (result)
		{
			// Transition to TAKING_OFF state
			setState(State::TAKING_OFF);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Takeoff failed");
			return false;
		}
	}

	bool UAVController::requestHover()
	{
		// Check if we're in a state where hovering is permitted
		if (state_ != State::MOVING && state_ != State::REACHED)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot hover from state %d", static_cast<int>(state_));
			return false;
		}

		// Request hovering
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Hovering...");
		bool result = ext_tools_->hover(agent_id_);

		if (result)
		{
			// Transition to HOVERING state
			setState(State::HOVERING);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Hovering failed");
			return false;
		}
	}

	bool UAVController::requestMove(const PointMsg& goal_pos)
	{
		// Check if we're in a state where movement is permitted
		if (state_ != State::HOVERING && state_ != State::REACHED)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot move from state %d", static_cast<int>(state_));
			return false;
		}

		// Set goal position
		goal_pos_ = goal_pos;
		has_goal_ = true;

		// Transition to MOVING state
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Moving to goal (%.2f, %.2f, %.2f)...",
			goal_pos_.x, goal_pos_.y, goal_pos_.z);
		setState(State::MOVING);
		return true;
	}

	bool UAVController::requestLand()
	{
		// Check if we're in a state where landing is permitted
		if (state_ != State::HOVERING && state_ != State::REACHED && state_ != State::MOVING)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot land from state %d", static_cast<int>(state_));
			return false;
		}

		// Request landing
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Landing...");
		bool result = ext_tools_->land(agent_id_);

		if (result)
		{
			// Transition to LANDING state
			setState(State::LANDING);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Landing failed");
			return false;
		}
	}

	bool UAVController::requestDisarm()
	{
		// Check if we're in a state where we can disarm
		if (state_ != State::ARMED)
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Cannot disarm from state %d", static_cast<int>(state_));
			return false;
		}

		// Request disarming
		RCLCPP_INFO(node_->get_logger(), "UAV controller: Disarming...");
		bool result = ext_tools_->armDisarm(agent_id_, false);

		if (result)
		{
			// Transition to DISARMED state
			setState(State::DISARMED);
			return true;
		}
		else
		{
			RCLCPP_ERROR(node_->get_logger(), "UAV controller: Disarming failed");
			return false;
		}
	}

	bool UAVController::reset()
	{
		// Attempt to safely land and disarm if needed
		if (state_ != State::DISARMED && state_ != State::IDLE)
		{
			// If we're in flight, try to land
			if (state_ == State::HOVERING || state_ == State::MOVING || state_ == State::REACHED)
			{
				ext_tools_->land(agent_id_);
			}

			// Wait a bit for landing
			rclcpp::sleep_for(std::chrono::seconds(2));

			// Disarm
			ext_tools_->armDisarm(agent_id_, false);
		}

		// Reset state
		setState(State::IDLE);

		// Reset data
		has_odom_ = false;
		has_goal_ = false;

		// Re-initialize
		setState(State::INITIALIZING);
		return true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update state machine
	// ════════════════════════════════════════════════════════════════════════════

	void UAVController::update()
	{
		// Handle state transitions and associated actions
		handleStateTransition();
	}

} // namespace flychams::control