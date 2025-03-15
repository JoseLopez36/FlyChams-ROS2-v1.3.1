#include "rclcpp/rclcpp.hpp"

// Config includes
#include "flychams_core/config/config_parser.hpp"
#include "flychams_core/config/config_types.hpp"

// Core includes
#include "flychams_core/utils/ros_utils.hpp"

using namespace flychams::core;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Airsim settings program for creating the settings.json file
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-02
 * ════════════════════════════════════════════════════════════════
 */

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create temporary node
    auto node = rclcpp::Node::make_shared("airsim_settings_parser", options);

    // Parse the configuration file
    const std::string& config_path = RosUtils::getParameter<std::string>(node, "config_source_file");
    ConfigPtr config_ptr;
    try
    {
        config_ptr = ConfigParser::parseExcelFile(config_path);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error parsing config file: %s", e.what());
        rclcpp::shutdown();
    }

    const std::string& airsim_path = RosUtils::getParameter<std::string>(node, "airsim_settings_destination_file");
    if (!ConfigParser::parseAirsimSettings(config_ptr, airsim_path))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to create AirSim settings.json");
        rclcpp::shutdown();
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "AirSim settings.json created successfully");
    }

    // Shutdown
    rclcpp::shutdown();

    return 0;
}