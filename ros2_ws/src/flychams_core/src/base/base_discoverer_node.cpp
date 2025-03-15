#include "flychams_core/base/base_discoverer_node.hpp"

namespace flychams::core
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    BaseDiscovererNode::BaseDiscovererNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : Node(node_name, options), node_name_(node_name)
    {
        // Nothing to do
    }

    void BaseDiscovererNode::init()
    {
        // Get node pointer
        node_ = this->shared_from_this();
        RCLCPP_INFO(node_->get_logger(), "Starting %s node...", node_name_.c_str());

        // Create tools
        config_tools_ = std::make_shared<ConfigTools>(node_);
        ext_tools_ = externalToolsFactory(node_, Framework::AirSim);
        topic_tools_ = std::make_shared<TopicTools>(node_);
        tf_tools_ = std::make_shared<TfTools>(node_);

        // Initialize discovery subscriber
        elements_.clear();
        discovery_sub_ = topic_tools_->createRegistrationSubscriber(
            std::bind(&BaseDiscovererNode::onDiscovery, this, std::placeholders::_1));

        // Call on init overridable method
        onInit();
        RCLCPP_INFO(node_->get_logger(), "%s node running", node_name_.c_str());
    }

    BaseDiscovererNode::~BaseDiscovererNode()
    {
        shutdown();
    }

    void BaseDiscovererNode::shutdown()
    {
        // Lock elements map
        std::lock_guard<std::mutex> lock(elements_mutex_);

        RCLCPP_INFO(node_->get_logger(), "Shutting down %s node...", node_name_.c_str());
        // Call on shutdown overridable method
        onShutdown();
        // Destroy discovery subscriber
        elements_.clear();
        discovery_sub_.reset();
        // Destroy tools
        config_tools_.reset();
        ext_tools_.reset();
        topic_tools_.reset();
        tf_tools_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // CALLBACKS: Callbacks for adding or removing elements
    // ════════════════════════════════════════════════════════════════════════════

    void BaseDiscovererNode::onDiscovery(const RegistrationMsg::SharedPtr msg)
    {
        // Lock elements map
        std::lock_guard<std::mutex> lock(elements_mutex_);

        // Add or remove elements
        for (const auto& element : msg->elements)
        {
            // Get element id and type
            const auto element_id = element.id;
            const auto element_type = static_cast<ElementType>(element.type);

            // Add or remove element
            if (elements_.find(element_id) == elements_.end())
            {
                elements_.insert({ element_id, element_type });

                // Call corresponding add callback
                switch (element_type)
                {
                case ElementType::Agent:
                    onAddAgent(element_id);
                    break;

                case ElementType::Target:
                    onAddTarget(element_id);
                    break;

                case ElementType::Cluster:
                    onAddCluster(element_id);
                    break;
                }
            }
            else
            {
                elements_.erase(element_id);

                // Call corresponding remove callback
                switch (element_type)
                {
                case ElementType::Agent:
                    onRemoveAgent(element_id);
                    break;

                case ElementType::Target:
                    onRemoveTarget(element_id);
                    break;

                case ElementType::Cluster:
                    onRemoveCluster(element_id);
                    break;
                }
            }
        }
    }

} // namespace flychams::core