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
        // Window commands
        window_image_cmd_group_pub_ = node_->create_publisher<WindowImageCmdGroup>("/airsim/group_of_windows/image_cmd", 10);
        // Tracking commands
        add_target_group_client_ = node_->create_client<AddTargetGroup>("/airsim/group_of_targets/add");
        add_cluster_group_client_ = node_->create_client<AddClusterGroup>("/airsim/group_of_clusters/add");
        update_target_cmd_group_pub_ = node_->create_publisher<UpdateTargetCmdGroupMsg>("/airsim/group_of_targets/update_cmd", 10);
        update_cluster_cmd_group_pub_ = node_->create_publisher<UpdateClusterCmdGroupMsg>("/airsim/group_of_clusters/update_cmd", 10);
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
        add_target_group_client_.reset();
        add_cluster_group_client_.reset();
        // Destroy publishers
        vel_cmd_pub_map_.clear();
        gimbal_angle_cmd_pub_map_.clear();
        camera_fov_cmd_pub_map_.clear();
        window_image_cmd_group_pub_.reset();
        update_target_cmd_group_pub_.reset();
        update_cluster_cmd_group_pub_.reset();
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

    // ════════════════════════════════════════════════════════════════════════════
    // TRACKING CONTROL: Service-based control methods
    // ════════════════════════════════════════════════════════════════════════════

    bool AirsimTools::addTargetGroup(const IDs& target_ids, const std::vector<TargetType>& target_types, const std::vector<PointMsg>& positions, const bool& highlight, const std::vector<ColorMsg>& highlight_colors)
    {
        // Create request
        auto request = std::make_shared<AddTargetGroup::Request>();
        request->target_names = target_ids;
        request->positions = positions;
        request->highlight = highlight;
        request->highlight_color_rgba = highlight_colors;

        for (size_t i = 0; i < target_ids.size(); ++i)
        {
            // Set target type based on target type
            switch (target_types[i])
            {
            case TargetType::Cube:
                request->target_types.push_back("Cube");
                break;
            case TargetType::Human:
                request->target_types.push_back("Human");
                break;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown target type: %d", static_cast<int>(target_types[i]));
                request->target_types.push_back("Cube");
                break;
            }
        }

        // Send request and wait for response
        return RosUtils::sendRequestSync<AddTargetGroup>(node_, add_target_group_client_, request, 100000);
    }

    bool AirsimTools::addClusterGroup(const IDs& cluster_ids, const std::vector<PointMsg>& centers, const std::vector<float>& radii, const bool& highlight, const std::vector<ColorMsg>& highlight_colors)
    {
        // Create request
        auto request = std::make_shared<AddClusterGroup::Request>();
        request->cluster_names = cluster_ids;
        request->centers = centers;
        request->radii = radii;
        request->highlight = highlight;
        request->highlight_color_rgba = highlight_colors;

        // Send request and wait for response
        return RosUtils::sendRequestSync<AddClusterGroup>(node_, add_cluster_group_client_, request, 100000);
    }

    // ════════════════════════════════════════════════════════════════════════════
    // OBJECT CONTROL: Publisher-based control methods
    // ════════════════════════════════════════════════════════════════════════════

    void AirsimTools::updateTargetGroup(const IDs& target_ids, const std::vector<PointMsg>& positions)
    {
        // Create message
        UpdateTargetCmdGroup msg;
        msg.target_names = target_ids;
        msg.positions = positions;

        // Publish message
        update_target_cmd_group_pub_->publish(msg);
    }

    void AirsimTools::updateClusterGroup(const IDs& cluster_ids, const std::vector<PointMsg>& centers, const std::vector<float>& radii)
    {
        // Create message
        UpdateClusterCmdGroup msg;
        msg.cluster_names = cluster_ids;
        msg.centers = centers;
        msg.radii = radii;

        // Publish message
        update_cluster_cmd_group_pub_->publish(msg);
    }

} // namespace flychams::core