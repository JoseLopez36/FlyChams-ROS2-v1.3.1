#include "rclcpp/rclcpp.hpp"

// Dashboard includes
#include "flychams_dashboard/gui/gui_controller.hpp"

// Core includes
#include "flychams_core/base/discoverer_node.hpp"

using namespace flychams::core;
using namespace flychams::dashboard;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief GUI node for the FlyingChameleons system
 *
 * @details
 * This class implements the GUI node for the FlyingChameleons system.
 * It uses the discoverer node to discover the different targets and then
 * creates a GUI controller for each target discovered.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-03-01
 * ════════════════════════════════════════════════════════════════
 */
class GuiNode : public DiscovererNode
{
public: // Constructor/Destructor
    GuiNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : DiscovererNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Initialize GUI controller
        gui_controller_ = std::make_shared<GuiController>(node_, config_tools_, ext_tools_, topic_tools_, tf_tools_);
    }

    void onShutdown() override
    {
        // Destroy GUI controller
        gui_controller_.reset();
    }

private: // Agent management
    void onAddAgent(const ID& agent_id) override
    {
        // Add agent to GUI controller
        gui_controller_->addAgent(agent_id);
    }

    void onRemoveAgent(const ID& agent_id) override
    {
        // Remove agent from GUI controller
        gui_controller_->removeAgent(agent_id);
    }

private: // Components
    // GUI controller
    GuiController::SharedPtr gui_controller_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create node
    auto node = std::make_shared<GuiNode>("gui_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}