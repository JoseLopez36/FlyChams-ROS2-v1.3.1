#pragma once

// Config includes
#include "flychams_core/config/config_parser.hpp"
#include "flychams_core/config/config_types.hpp"

// Core includes
#include "flychams_core/utils/ros_utils.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Config Manager for handling configuration parsing and storing
     *
     * @details
     * This class provides utilities for managing the configuration of the
     * FlyChams system. It also provides utilities for parsing the configuration
     * into AirSim settings.json.
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class ConfigTools
    {
    public: // Types
        using SharedPtr = std::shared_ptr<ConfigTools>;

    private: // Data
        // Configuration
        ConfigPtr config_ptr_;

        // ROS components
        NodePtr node_;

    public: // Constructor/Destructor
        ConfigTools(NodePtr node)
            : node_(node)
        {
            // Parse the configuration file
            const std::string& config_path = RosUtils::getParameter<std::string>(node_, "config_source_file");
            config_ptr_ = ConfigParser::parseExcelFile(config_path);
        }

        ~ConfigTools()
        {
            shutdown();
        }

        void shutdown()
        {
            // Destroy config
            config_ptr_.reset();
            // Destroy node
            node_.reset();
        }

    public: // AirSim utilities
        void createAirsimSettings() const
        {
            const std::string& airsim_path = RosUtils::getParameter<std::string>(node_, "airsim_settings_destination_file");
            if (!ConfigParser::parseAirsimSettings(config_ptr_, airsim_path))
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to create AirSim settings.json");
                rclcpp::shutdown();
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "AirSim settings.json created successfully");
            }
        }

    public: // Raw getter methods

        const ConfigPtr getConfig() const
        {
            return config_ptr_;
        }

        const MissionConfigPtr getMission() const
        {
            return config_ptr_->mission;
        }

        const SimulationConfigPtr getSimulation() const
        {
            return config_ptr_->simulation;
        }

        const MapConfigPtr getMap() const
        {
            return config_ptr_->map;
        }

        const GroupConfigMap getGroups() const
        {
            return config_ptr_->groups;
        }

        const GroupConfigPtr getGroup(const std::string& group_id) const
        {
            return config_ptr_->groups.at(group_id);
        }

        const AgentConfigMap getAgents() const
        {
            return config_ptr_->agents;
        }

        const AgentConfigPtr getAgent(const std::string& agent_id) const
        {
            return config_ptr_->agents.at(agent_id);
        }

        const HeadConfigPtr getHead(const std::string& agent_id, const std::string& head_id) const
        {
            return config_ptr_->agents.at(agent_id)->heads.at(head_id);
        }

        const DroneConfigPtr getDrone(const std::string& agent_id) const
        {
            return config_ptr_->agents.at(agent_id)->drone;
        }

        const GimbalConfigPtr getGimbal(const std::string& agent_id, const std::string& head_id) const
        {
            return config_ptr_->agents.at(agent_id)->heads.at(head_id)->gimbal;
        }

        const GimbalLinkConfigPtr getGimbalLink(const std::string& agent_id, const std::string& head_id, const std::string& link_id) const
        {
            return config_ptr_->agents.at(agent_id)->heads.at(head_id)->gimbal->links.at(link_id);
        }

        const CameraConfigPtr getCamera(const std::string& agent_id, const std::string& head_id) const
        {
            return config_ptr_->agents.at(agent_id)->heads.at(head_id)->camera;
        }

    public: // Config utilities
        CameraParameters getCameraParameters(const std::string& agent_id, const std::string& head_id) const
        {
            // Extract head config
            const auto& head_ptr = getHead(agent_id, head_id);
            const auto& camera_ptr = getCamera(agent_id, head_id);

            // Extract camera parameters
            const auto& f_min = head_ptr->min_focal;
            const auto& f_max = head_ptr->max_focal;
            const auto& f_ref = camera_ptr->default_focal;
            const auto& width = static_cast<float>(camera_ptr->resolution(0));
            const auto& height = static_cast<float>(camera_ptr->resolution(1));
            const auto& sensor_width = camera_ptr->sensor_width;
            const auto& sensor_height = camera_ptr->sensor_height;

            // Extract ROI parameters
            const auto& s_min_pix = RosUtils::getParameterOr<float>(node_, "roi_params.s_min_pix", 200.0f); // [pix]
            const auto& kappa_s = RosUtils::getParameterOr<float>(node_, "roi_params.kappa_s", 0.8f);

            // Create and fill head parameters
            CameraParameters params;

            // Minimum and maximum focal distances
            params.f_min = f_min; // [m]
            params.f_max = f_max; // [m]

            // Default focal distance (taken as reference)
            params.f_ref = f_ref; // [m]

            // Maximum admissible apparent size of the object in the image (in pixels)
            // Assuming 90% of the half-height (or width if smaller)
            float s_max_pix = 0.5f * static_cast<float>(std::min(width, height)) * 0.9f;
            params.s_max_pix = s_max_pix; // [pix]

            // Minimum admissible apparent size (in pixels)
            // Provided externally
            params.s_min_pix = s_min_pix; // [pix]

            // Reference apparent size (in pixels)
            // Calculated using kappaS (half-width fraction) parameter
            float s_ref_pix = s_max_pix * kappa_s + s_min_pix * (1.0f - kappa_s); // [pix]
            params.s_ref_pix = s_ref_pix; // [pix]

            // Conversion to metric distances on the sensor surface
            float rho_x = sensor_width / width;     // [m/pix]
            float rho_y = sensor_height / height;   // [m/pix]
            float rho = std::sqrt(rho_x * rho_y);   // [m/pix]
            params.s_max = s_max_pix * rho;         // [m]
            params.s_min = s_min_pix * rho;         // [m]
            params.s_ref = s_ref_pix * rho;         // [m]

            // Other parameters
            params.sensor_width = sensor_width;
            params.sensor_height = sensor_height;

            return params;
        }
    };

} // namespace flychams::core