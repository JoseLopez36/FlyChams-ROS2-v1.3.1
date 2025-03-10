#include "rclcpp/rclcpp.hpp"

// Core includes
#include "flychams_core/base/registrator_node.hpp"

using namespace flychams::core;

/**
 * ════════════════════════════════════════════════════════════════
 * @brief Agent registration node for registering the different agents
 * in the simulation
 *
 * @details
 * This class implements the agent registration node for registering the
 * different agents in the simulation. It uses the registrator node to
 * register the different agents, so that they can be discovered by the
 * different nodes.
 *
 * ════════════════════════════════════════════════════════════════
 * @author Jose Francisco Lopez Ruiz
 * @date 2025-02-28
 * ════════════════════════════════════════════════════════════════
 */
class ClusterRegistrationNode : public RegistratorNode
{
public: // Constructor/Destructor
    ClusterRegistrationNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : RegistratorNode(node_name, options)
    {
        // Nothing to do
    }

    void onInit() override
    {
        // Create cluster callback group
        CallbackGroupPtr callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Cycle through all agents in the config
        std::vector<ColorMsg> highlight_colors;
        for (const auto& [_, config_ptr] : config_tools_->getAgents())
        {
            // Extract macimum number of assignments per agent
            const int max_assign = config_ptr->max_assignments;

            // Cycle through all assignments and register clusters
            for (int i = 0; i < max_assign; i++)
            {
                // Generate cluster ID
                std::stringstream ss;
                ss << "CLUSTER" << std::setw(2) << std::setfill('0') << i;
                const ID cluster_id = ss.str();

                // Register cluster
                registerCluster(cluster_id);

                // Add cluster to vectors
                cluster_ids_.push_back(cluster_id);
                PointMsg zero_position;
                zero_position.x = 0.0f;
                zero_position.y = 0.0f;
                zero_position.z = 0.0f;
                cluster_centers_.push_back(zero_position);
                cluster_radii_.push_back(0.0f);
                // Cyan color
                ColorMsg color;
                color.r = 0.0f;
                color.g = 1.0f;
                color.b = 1.0f;
                color.a = 0.04f;
                highlight_colors.push_back(color);

                // Create cluster info subscriber
                auto options = rclcpp::SubscriptionOptions();
                options.callback_group = callback_group;
                cluster_info_subs_.push_back(topic_tools_->createClusterInfoSubscriber(cluster_id,
                    [this, i](const ClusterInfoMsg::SharedPtr msg)
                    {
                        this->clusterInfoCallback(i, msg);
                    }, options));
            }
        }

        // Add clusters to simulation
        ext_tools_->addClusterGroup(cluster_ids_, cluster_centers_, cluster_radii_, config_tools_->getSimulation()->draw_world_markers, highlight_colors);

        // Set cluster update timer
        const auto& update_rate = RosUtils::getParameter<float>(node_, "cluster_registration.cluster_update_rate");
        update_timer_ = RosUtils::createTimerByRate(node_, update_rate,
            std::bind(&ClusterRegistrationNode::onUpdate, this),
            callback_group);
    }

    void onShutdown() override
    {
        // Reset cluster update components
        cluster_centers_.clear();
        cluster_radii_.clear();
        cluster_info_subs_.clear();
        update_timer_.reset();
    }

private: // Methods
    void clusterInfoCallback(const int& i, const ClusterInfoMsg::SharedPtr msg)
    {
        // Update cluster metrics under lock
        cluster_centers_[i] = msg->center;
        cluster_radii_[i] = msg->radius;
    }

    void onUpdate()
    {
        // Update cluster group on simulation
        ext_tools_->updateClusterGroup(cluster_ids_, cluster_centers_, cluster_radii_);
    }

private: // Components
    // Target update
    std::vector<ID> cluster_ids_;
    std::vector<PointMsg> cluster_centers_;
    std::vector<float> cluster_radii_;
    std::vector<rclcpp::Subscription<ClusterInfoMsg>::SharedPtr> cluster_info_subs_;
    TimerPtr update_timer_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    // Create node options
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    // Create and initialize node
    auto node = std::make_shared<ClusterRegistrationNode>("cluster_registration_node", options);
    node->init();
    // Spin node
    rclcpp::spin(node);
    // Shutdown
    rclcpp::shutdown();
    return 0;
}