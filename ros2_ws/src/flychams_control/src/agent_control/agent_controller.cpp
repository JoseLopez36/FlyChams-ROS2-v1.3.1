#include "flychams_control/agent_control/agent_controller.hpp"

using namespace std::chrono_literals;
using namespace flychams::core;

namespace flychams::control
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void AgentController::onInit()
	{
		// Get parameters from parameter server
		// Get update rates
		float update_rate = RosUtils::getParameterOr<float>(node_, "agent_control.control_update_rate", 200.0f);
		// Get goal tolerance
		goal_tolerance_ = RosUtils::getParameterOr<float>(node_, "agent_control.goal_tolerance", 0.2f);
		// Get proportional gains
		float kp_x = RosUtils::getParameterOr<float>(node_, "agent_control.kp_x", 1.2f);
		float kp_y = RosUtils::getParameterOr<float>(node_, "agent_control.kp_y", 1.2f);
		float kp_z = RosUtils::getParameterOr<float>(node_, "agent_control.kp_z", 1.5f);
		// Get integral gains
		float ki_x = RosUtils::getParameterOr<float>(node_, "agent_control.ki_x", 0.01f);
		float ki_y = RosUtils::getParameterOr<float>(node_, "agent_control.ki_y", 0.01f);
		float ki_z = RosUtils::getParameterOr<float>(node_, "agent_control.ki_z", 0.02f);
		// Get derivative gains
		float kd_x = RosUtils::getParameterOr<float>(node_, "agent_control.kd_x", 0.4f);
		float kd_y = RosUtils::getParameterOr<float>(node_, "agent_control.kd_y", 0.4f);
		float kd_z = RosUtils::getParameterOr<float>(node_, "agent_control.kd_z", 0.5f);
		// Get velocity limits
		float max_v_x = RosUtils::getParameterOr<float>(node_, "agent_control.max_v_x", 3.0f);
		float max_v_y = RosUtils::getParameterOr<float>(node_, "agent_control.max_v_y", 3.0f);
		float max_v_z = RosUtils::getParameterOr<float>(node_, "agent_control.max_v_z", 2.0f);
		// Get integral limits
		float i_max_x = RosUtils::getParameterOr<float>(node_, "agent_control.i_max_x", 0.5f);
		float i_max_y = RosUtils::getParameterOr<float>(node_, "agent_control.i_max_y", 0.5f);
		float i_max_z = RosUtils::getParameterOr<float>(node_, "agent_control.i_max_z", 0.3f);
		// Calculate velocity command duration
		vel_cmd_duration_ = 0.15f * config_tools_->getSimulation()->clock_speed;

		// Initialize agent data
		state_ = State::Landed;
		curr_pos_ = Vector3r::Zero();
		has_odom_ = false;
		target_pos_ = Vector3r::Zero();
		has_goal_ = false;

		// Initialize PID controllers
		// Set gains
		pid_x_.setGains(kp_x, ki_x, kd_x);
		pid_y_.setGains(kp_y, ki_y, kd_y);
		pid_z_.setGains(kp_z, ki_z, kd_z);
		// Set output limits
		pid_x_.setOutputLimits(-max_v_x, max_v_x);
		pid_y_.setOutputLimits(-max_v_y, max_v_y);
		pid_z_.setOutputLimits(-max_v_z, max_v_z);
		// Set integral limits
		pid_x_.setIntegralLimits(-i_max_x, i_max_x);
		pid_y_.setIntegralLimits(-i_max_y, i_max_y);
		pid_z_.setIntegralLimits(-i_max_z, i_max_z);
		// Reset PIDs
		pid_x_.reset();
		pid_y_.reset();
		pid_z_.reset();

		// Subscribe to odom and goal topics
		odom_sub_ = topic_tools_->createAgentOdomSubscriber(agent_id_,
			std::bind(&AgentController::odomCallback, this, std::placeholders::_1));
		goal_sub_ = topic_tools_->createAgentGoalSubscriber(agent_id_,
			std::bind(&AgentController::goalCallback, this, std::placeholders::_1));

		// Set update timers
		prev_time_ = RosUtils::getTimeNow(node_);
		control_timer_ = RosUtils::createTimerByRate(node_, update_rate,
			std::bind(&AgentController::updateControl, this));
	}

	void AgentController::onShutdown()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		// Land vehicle
		state_ = State::Land;
		// Destroy subscribers
		odom_sub_.reset();
		goal_sub_.reset();
		// Destroy update timer
		control_timer_.reset();
	}

	// ════════════════════════════════════════════════════════════════════════════
	// CALLBACKS: Callback functions
	// ════════════════════════════════════════════════════════════════════════════

	void AgentController::odomCallback(const core::OdometryMsg::SharedPtr msg)
	{
		// Transform to world frame
		const std::string& source_frame = msg->header.frame_id;
		const std::string& child_frame = msg->child_frame_id;
		const std::string& target_frame = tf_tools_->getWorldFrame();
		const PointMsg& curr_pos_msg = tf_tools_->transformPointMsg(msg->pose.pose.position, source_frame, target_frame);
		const Vector3Msg& curr_vel_msg = tf_tools_->transformVelocityMsg(msg->twist.twist.linear, child_frame, target_frame);
		// Convert to Eigen
		std::lock_guard<std::mutex> lock(mutex_);
		curr_pos_ = MsgConversions::fromMsg(curr_pos_msg);
		curr_vel_ = MsgConversions::fromMsg(curr_vel_msg);
		has_odom_ = true;
	}

	void AgentController::goalCallback(const core::AgentGoalMsg::SharedPtr msg)
	{
		// Get target position
		std::lock_guard<std::mutex> lock(mutex_);
		target_pos_ = MsgConversions::fromMsg(msg->position);
		has_goal_ = true;
	}

	// ════════════════════════════════════════════════════════════════════════════
	// UPDATE: Update PID controllers
	// ════════════════════════════════════════════════════════════════════════════

	void AgentController::updateControl()
	{
		std::lock_guard<std::mutex> lock(mutex_);

		// Check if odometry is available
		if (!has_odom_)
		{
			RCLCPP_WARN(node_->get_logger(), "Agent controller: No odometry received for agent %s, skipping update", agent_id_.c_str());
			return;
		}

		switch (state_)
		{
		case State::Landed:
		{
			// Check if vehicle is already in the air (above 0.50m) or with a certain Z velocity (0.1 m/s)
			if (curr_pos_.z() > 0.50f || std::abs(curr_vel_.z()) > 0.1f)
			{
				// Vehicle is already in the air, transition to Hover state
				state_ = State::Hover;
				RCLCPP_WARN(node_->get_logger(), "Agent controller: Vehicle is already in the air, transition to Hover state");
			}
			else
			{
				// Vehicle is on the ground, transition to Takeoff state
				state_ = State::Takeoff;
				RCLCPP_WARN(node_->get_logger(), "Agent controller: Vehicle is on the ground, transition to Takeoff state");
			}
		}
		break;

		case State::Takeoff:
		{
			// Command takeoff
			if (ext_tools_->takeoffVehicleGroup({ agent_id_ }))
			{
				// Vehicle is taking off, transition to Takingoff state
				state_ = State::Takingoff;
				RCLCPP_INFO(node_->get_logger(), "Agent controller: Vehicle is taking off, transition to Takingoff state");
			}
			else
			{
				// Vehicle did not take off, wait 1 second and try again
				std::this_thread::sleep_for(1s);
				RCLCPP_WARN(node_->get_logger(), "Agent controller: Vehicle did not take off, wait 1 second and try again");
			}
		}
		break;

		case State::Takingoff:
		{
			// Vehicle is taking off, wait until velocity is 0 and transition to Hover state
			while (std::abs(curr_vel_.norm()) > 0.1f)
			{
				std::this_thread::sleep_for(500ms);
			}
			state_ = State::Hover;
			RCLCPP_WARN(node_->get_logger(), "Agent controller: Vehicle took off, transition to Hover state");
		}
		break;

		case State::Hover:
		{
			// Command hover
			if (ext_tools_->hoverVehicleGroup({ agent_id_ }))
			{
				// Vehicle is hovering, wait 50 ms and transition to Hovering state
				std::this_thread::sleep_for(50ms);
				state_ = State::Hovering;
				RCLCPP_INFO(node_->get_logger(), "Agent controller: Vehicle is hovering, transition to Hovering state");
			}
			else
			{
				// Vehicle did not hover, wait 500 ms and try again
				std::this_thread::sleep_for(500ms);
				RCLCPP_WARN(node_->get_logger(), "Agent controller: Vehicle did not hover, wait 500 ms and try again");
			}
		}
		break;

		case State::Hovering:
		{
			// Vehicle is hovering. Check if goal is set
			if (has_goal_)
			{
				// Goal is set, transition to MovingToGoal state
				state_ = State::MovingToGoal;
				RCLCPP_WARN(node_->get_logger(), "Agent controller: Goal is set, transition to MovingToGoal state");
			}
			else
			{
				// Goal is not set, wait 250 ms and try again
				RCLCPP_INFO(node_->get_logger(), "Agent controller: Goal is not set");
			}
		}
		break;

		case State::MovingToGoal:
		{
			// Vehicle is moving to goal. Check if goal is reached
			if (MathUtils::distance(curr_pos_, target_pos_) < goal_tolerance_)
			{
				// Goal is reached, transition to Hover state
				state_ = State::Hover;
				RCLCPP_WARN(node_->get_logger(), "Agent controller: Goal is reached, transition to Hover state");
			}
			else
			{
				// Goal is not reached, compute velocity command and update PIDs
				Vector3r vel_cmd = computeVelocityCommand(curr_pos_, target_pos_);
				ext_tools_->setVehicleVelocity(agent_id_, vel_cmd.x(), vel_cmd.y(), vel_cmd.z(), vel_cmd_duration_, tf_tools_->getWorldFrame());
			}
		}
		break;

		case State::Land:
		{
			// Command landing
			if (ext_tools_->landVehicleGroup({ agent_id_ }))
			{
				// Vehicle is landing, transition to Landing state
				state_ = State::Landing;
				RCLCPP_INFO(node_->get_logger(), "Agent controller: Vehicle is landing, transition to Landing state");
			}
			else
			{
				// Vehicle did not land, wait 1 second and try again
				std::this_thread::sleep_for(1s);
				RCLCPP_WARN(node_->get_logger(), "Agent controller: Vehicle did not land, wait 1 second and try again");
			}
		}
		break;

		case State::Landing:
		{
			// Vehicle is landing, wait until velocity is 0 and transition to Landed state
			while (std::abs(curr_vel_.norm()) > 0.1f)
			{
				std::this_thread::sleep_for(500ms);
			}
			state_ = State::Landed;
			RCLCPP_WARN(node_->get_logger(), "Agent controller: Vehicle is landed, transition to Landed state");
		}
		break;

		default:
		{
			state_ = State::Land;
			RCLCPP_ERROR(node_->get_logger(), "AgentController: Invalid state: %d. Landing vehicle", static_cast<int>(state_));
		}
		break;
		}
	}

	// ════════════════════════════════════════════════════════════════════════════
	// METHODS: Helper methods
	// ════════════════════════════════════════════════════════════════════════════

	Vector3r AgentController::computeVelocityCommand(const Vector3r& curr_pos, const Vector3r& target_pos)
	{
		// Get current time
		auto curr_time = RosUtils::getTimeNow(node_);
		float dt = (curr_time - prev_time_).seconds();
		prev_time_ = curr_time;

		// Update PIDs
		Vector3r vel_cmd(
			pid_x_.update(curr_pos.x(), target_pos.x(), dt),
			pid_y_.update(curr_pos.y(), target_pos.y(), dt),
			pid_z_.update(curr_pos.z(), target_pos.z(), dt)
		);

		return vel_cmd;
	}

} // namespace flychams::control