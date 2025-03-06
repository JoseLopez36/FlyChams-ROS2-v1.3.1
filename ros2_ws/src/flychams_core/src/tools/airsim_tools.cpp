#include "flychams_core/tools/airsim_tools.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace airsim_interfaces::msg;
using namespace airsim_interfaces::srv;

namespace flychams::core
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    AirsimTools::AirsimTools(NodePtr node)
        : node_(node), is_connected_(false), is_running_(false)
    {
        // Initialize ROS components
        // AirSim status
        airsim_status_sub_ = node_->create_subscription<airsim_interfaces::msg::Status>("/airsim/status", 1, std::bind(&AirsimTools::status_cb, this, _1));
        // Global commands
        reset_client_ = node_->create_client<Reset>("/airsim/reset");
        run_client_ = node_->create_client<Run>("/airsim/run");
        pause_client_ = node_->create_client<Pause>("/airsim/pause");
        // Vehicle commands
        takeoff_group_client_ = node_->create_client<TakeoffGroup>("/airsim/group_of_robots/takeoff");
        land_group_client_ = node_->create_client<LandGroup>("/airsim/group_of_robots/land");
        hover_group_client_ = node_->create_client<HoverGroup>("/airsim/group_of_robots/hover");
        // Object commands
        spawn_object_group_client_ = node_->create_client<SpawnObjectGroup>("/airsim/group_of_objects/spawn");
        despawn_object_group_client_ = node_->create_client<DespawnObjectGroup>("/airsim/group_of_objects/despawn");
        object_pose_cmd_group_pub_ = node_->create_publisher<ObjectPoseCmdGroup>("/airsim/group_of_objects/pose_cmd", 10);
        // Window commands
        window_image_cmd_group_pub_ = node_->create_publisher<WindowImageCmdGroup>("/airsim/group_of_windows/image_cmd", 10);
    }

    AirsimTools::~AirsimTools()
    {
        shutdown();
    }

    void AirsimTools::shutdown()
    {
        // Destroy status subscriber
        airsim_status_sub_.reset();
        // Destroy clients
        reset_client_.reset();
        run_client_.reset();
        pause_client_.reset();
        takeoff_group_client_.reset();
        land_group_client_.reset();
        hover_group_client_.reset();
        spawn_object_group_client_.reset();
        despawn_object_group_client_.reset();
        // Destroy publishers
        vel_cmd_pub_map_.clear();
        gimbal_angle_cmd_pub_map_.clear();
        camera_fov_cmd_pub_map_.clear();
        object_pose_cmd_group_pub_.reset();
        window_image_cmd_group_pub_.reset();
        // Destroy node
        node_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // VEHICLE ADDERS: Vehicle adders
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimTools::addVehicle(const ID& vehicle_id)
    {
        // Create publishers
        vel_cmd_pub_map_[vehicle_id] = node_->create_publisher<VelCmd>("/airsim/" + vehicle_id + "/vel_cmd", 10);
        gimbal_angle_cmd_pub_map_[vehicle_id] = node_->create_publisher<GimbalAngleCmd>("/airsim/" + vehicle_id + "/gimbal_angle_cmd", 10);
        camera_fov_cmd_pub_map_[vehicle_id] = node_->create_publisher<CameraFovCmd>("/airsim/" + vehicle_id + "/camera_fov_cmd", 10);
    }

    void AirsimTools::removeVehicle(const ID& vehicle_id)
    {
        // Destroy publishers
        vel_cmd_pub_map_.erase(vehicle_id);
        gimbal_angle_cmd_pub_map_.erase(vehicle_id);
        camera_fov_cmd_pub_map_.erase(vehicle_id);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // STATUS: Status methods
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimTools::status_cb(const airsim_interfaces::msg::Status::SharedPtr msg)
    {
        is_connected_.store(msg->is_connected);
        is_running_.store(msg->is_running);
    }

    void AirsimTools::waitConnection() const
    {
        // Wait for AirSim to be connected
        RCLCPP_INFO(node_->get_logger(), "Waiting for AirSim connection...");
        while (!isConnected())
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(), "Waiting for AirSim connection interrupted. Shutting down node");
                rclcpp::shutdown();
                return;
            }
            std::this_thread::sleep_for(200ms);
        }
        RCLCPP_INFO(node_->get_logger(), "AirSim connected!");
    }

    bool AirsimTools::isConnected() const
    {
        return is_connected_.load();
    }

    void AirsimTools::waitRunning() const
    {
        // Wait for AirSim to be running
        while (!isRunning())
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_->get_logger(), "Waiting for AirSim running interrupted. Shutting down node");
                rclcpp::shutdown();
                return;
            }
            std::this_thread::sleep_for(200ms);
        }
    }

    bool AirsimTools::isRunning() const
    {
        return is_running_.load();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // GLOBAL CONTROL: Service-based control methods
    // ════════════════════════════════════════════════════════════════════════════

    bool AirsimTools::resetSimulation()
    {
        // Create request
        auto request = std::make_shared<Reset::Request>();

        // Send request and wait for response
        return RosUtils::sendRequestSync<Reset>(node_, reset_client_, request, 1000);
    }

    bool AirsimTools::runSimulation()
    {
        // Create request
        auto request = std::make_shared<Run::Request>();

        // Send request and wait for response
        return RosUtils::sendRequestSync<Run>(node_, run_client_, request, 1000);
    }

    bool AirsimTools::pauseSimulation()
    {
        // Create request
        auto request = std::make_shared<Pause::Request>();

        // Send request and wait for response
        return RosUtils::sendRequestSync<Pause>(node_, pause_client_, request, 1000);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // VEHICLE CONTROL: Service-based control methods
    // ════════════════════════════════════════════════════════════════════════════

    bool AirsimTools::takeoffVehicleGroup(const IDs& vehicle_ids)
    {
        // Create request
        auto request = std::make_shared<TakeoffGroup::Request>();
        request->vehicle_names = vehicle_ids;

        // Send request and wait for response
        return RosUtils::sendRequestSync<TakeoffGroup>(node_, takeoff_group_client_, request, 1000);
    }

    bool AirsimTools::landVehicleGroup(const IDs& vehicle_ids)
    {
        // Create request
        auto request = std::make_shared<LandGroup::Request>();
        request->vehicle_names = vehicle_ids;

        // Send request and wait for response
        return RosUtils::sendRequestSync<LandGroup>(node_, land_group_client_, request, 1000);
    }

    bool AirsimTools::hoverVehicleGroup(const IDs& vehicle_ids)
    {
        // Create request
        auto request = std::make_shared<HoverGroup::Request>();
        request->vehicle_names = vehicle_ids;

        // Send request and wait for response
        return RosUtils::sendRequestSync<HoverGroup>(node_, hover_group_client_, request, 1000);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // VEHICLE CONTROL: Publisher-based control methods
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimTools::setVehicleVelocity(const ID& vehicle_id, const float& vel_cmd_x, const float& vel_cmd_y, const float& vel_cmd_z, const float& vel_cmd_dt, const std::string& frame_id)
    {
        // Create message
        VelCmd msg;
        msg.header = RosUtils::createHeader(node_, frame_id);
        msg.vel_cmd_x = vel_cmd_x;
        msg.vel_cmd_y = vel_cmd_y;
        msg.vel_cmd_z = vel_cmd_z;
        msg.vel_cmd_dt = vel_cmd_dt;

        // Publish message
        vel_cmd_pub_map_[vehicle_id]->publish(msg);
    }

    void AirsimTools::setGimbalAngles(const ID& vehicle_id, const IDs& camera_ids, const std::vector<QuaternionMsg>& target_quats, const std::string& frame_id)
    {
        // Create message
        GimbalAngleCmd msg;
        msg.header = RosUtils::createHeader(node_, frame_id);
        msg.camera_names = camera_ids;
        msg.orientations = target_quats;

        // Publish message
        gimbal_angle_cmd_pub_map_[vehicle_id]->publish(msg);
    }

    void AirsimTools::setCameraFovs(const ID& vehicle_id, const IDs& camera_ids, const std::vector<float>& target_fovs, const std::string& frame_id)
    {
        // Create message
        CameraFovCmd msg;
        msg.header = RosUtils::createHeader(node_, frame_id);
        msg.camera_names = camera_ids;
        msg.fovs = target_fovs;

        // Publish message
        camera_fov_cmd_pub_map_[vehicle_id]->publish(msg);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // OBJECT CONTROL: Service-based control methods
    // ════════════════════════════════════════════════════════════════════════════

    bool AirsimTools::spawnObjectGroup(const IDs& object_ids, const std::vector<PoseMsg>& poses, const std::vector<float>& scales, const std::vector<TargetType>& object_types)
    {
        // Create request
        auto request = std::make_shared<SpawnObjectGroup::Request>();
        request->object_names = object_ids;
        request->poses = poses;
        request->scales = scales;

        for (size_t i = 0; i < object_ids.size(); ++i)
        {
            // Set object blueprint based on object type
            switch (object_types[i])
            {
            case TargetType::Human:
                request->blueprints.push_back("CubeBP");
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown object type: %d", static_cast<int>(object_types[i]));
                rclcpp::shutdown();
                return false;
            }
        }

        // Send request and wait for response
        return RosUtils::sendRequestSync<SpawnObjectGroup>(node_, spawn_object_group_client_, request, 100000);
    }

    bool AirsimTools::despawnObjectGroup(const IDs& object_ids)
    {
        // Create request
        auto request = std::make_shared<DespawnObjectGroup::Request>();
        request->object_names = object_ids;

        // Send request and wait for response
        return RosUtils::sendRequestSync<DespawnObjectGroup>(node_, despawn_object_group_client_, request, 1000);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // OBJECT CONTROL: Publisher-based control methods
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimTools::setObjectPoseGroup(const IDs& object_ids, const std::vector<PoseMsg>& poses, const std::string& frame_id)
    {
        // Create message
        ObjectPoseCmdGroup msg;
        msg.header = RosUtils::createHeader(node_, frame_id);
        msg.object_names = object_ids;
        msg.poses = poses;

        // Publish message
        object_pose_cmd_group_pub_->publish(msg);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // WINDOW CONTROL: Service-based control methods
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimTools::setWindowImageGroup(const IDs& window_ids, const IDs& vehicle_ids, const IDs& camera_ids, const std::vector<int>& crop_x, const std::vector<int>& crop_y, const std::vector<int>& crop_w, const std::vector<int>& crop_h)
    {
        // Create message
        WindowImageCmdGroup msg;
        msg.window_indices = getWindowIndices(window_ids);
        msg.vehicle_names = vehicle_ids;
        msg.camera_names = camera_ids;
        msg.crop_x = crop_x;
        msg.crop_y = crop_y;
        msg.crop_w = crop_w;
        msg.crop_h = crop_h;

        // Publish message
        window_image_cmd_group_pub_->publish(msg);
    }

} // namespace flychams::core