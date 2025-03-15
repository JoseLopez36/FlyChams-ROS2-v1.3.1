#include "flychams_core/base/base_registrator_node.hpp"

namespace flychams::core
{
    // ════════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR: Constructor and destructor
    // ════════════════════════════════════════════════════════════════════════════

    BaseRegistratorNode::BaseRegistratorNode(const std::string& node_name, const rclcpp::NodeOptions& options)
        : Node(node_name, options), node_name_(node_name)
    {
        // Nothing to do
    }

    void BaseRegistratorNode::init()
    {
        // Get node pointer
        node_ = this->shared_from_this();
        RCLCPP_INFO(node_->get_logger(), "Starting %s node...", node_name_.c_str());

        // Create tools
        config_tools_ = std::make_shared<ConfigTools>(node_);
        ext_tools_ = externalToolsFactory(node_, Framework::AirSim);
        topic_tools_ = std::make_shared<TopicTools>(node_);
        tf_tools_ = std::make_shared<TfTools>(node_);

        // Initialize registration publisher
        elements_.clear();
        registration_pub_ = topic_tools_->createRegistrationPublisher();

        // Initialize update timer
        update_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() { publishRegistration(); });

        // Call on init overridable method
        onInit();
        RCLCPP_INFO(node_->get_logger(), "%s node running", node_name_.c_str());
    }

    BaseRegistratorNode::~BaseRegistratorNode()
    {
        shutdown();
    }

    void BaseRegistratorNode::shutdown()
    {
        // Lock elements map
        std::lock_guard<std::mutex> lock(elements_mutex_);

        RCLCPP_INFO(node_->get_logger(), "Shutting down %s node...", node_name_.c_str());
        // Call on shutdown overridable method
        onShutdown();
        // Destroy registration publisher
        elements_.clear();
        registration_pub_.reset();
        // Destroy tools
        config_tools_.reset();
        ext_tools_.reset();
        topic_tools_.reset();
        tf_tools_.reset();
    }

    // ════════════════════════════════════════════════════════════════════════════
    // REGISTRATION METHODS: Registration methods for adding or removing elements
    // ════════════════════════════════════════════════════════════════════════════

    void BaseRegistratorNode::registerElement(const ID& element_id, const ElementType& element_type)
    {
        // Lock elements map
        std::lock_guard<std::mutex> lock(elements_mutex_);

        // Add element to map (only if not already registered)
        if (elements_.find(element_id) != elements_.end())
            return;
        elements_.insert({ element_id, element_type });

        // Register element in tools
        switch (element_type)
        {
        case ElementType::Agent:
            ext_tools_->addVehicle(element_id);
            break;
        }

        RCLCPP_INFO(node_->get_logger(), "Element %s registered", element_id.c_str());
    }

    void BaseRegistratorNode::unregisterElement(const ID& element_id, const ElementType& element_type)
    {
        // Lock elements map
        std::lock_guard<std::mutex> lock(elements_mutex_);

        // Remove element from map (only if registered)
        if (elements_.find(element_id) == elements_.end())
            return;
        elements_.erase(element_id);

        // Unregister element in tools
        switch (element_type)
        {
        case ElementType::Agent:
            ext_tools_->removeVehicle(element_id);
            break;
        }

        RCLCPP_INFO(node_->get_logger(), "Element %s unregistered", element_id.c_str());
    }

    // ════════════════════════════════════════════════════════════════════════════
    // UPDATE METHODS: Update methods for publishing registration messages
    // ════════════════════════════════════════════════════════════════════════════

    void BaseRegistratorNode::publishRegistration()
    {
        // Lock elements map
        std::lock_guard<std::mutex> lock(elements_mutex_);

        // Create and publish registration message
        RegistrationMsg msg;
        for (const auto& [id, type] : elements_)
        {
            ElementMsg element;
            element.id = id;
            element.type = static_cast<uint8_t>(type);
            msg.elements.push_back(element);
        }
        registration_pub_->publish(msg);
    }

} // namespace flychams::core