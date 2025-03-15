#pragma once

// Standard includes
#include <mutex>
#include <unordered_map>

// Tools includes
#include "flychams_core/tools/config_tools.hpp"
#include "flychams_core/tools/external_tools.hpp"
#include "flychams_core/tools/topic_tools.hpp"
#include "flychams_core/tools/tf_tools.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Registrator node for registering the different elements
     * in the simulation
     *
     * @details
     * This class implements the registrator node for registering agents,
     * targets, and clusters with the help of the various tools. It serves
     * as a base class for the different nodes that need to register elements
     * dynamically in the simulation.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class BaseRegistratorNode : public rclcpp::Node
    {
    public: // Constructor/Destructor
        BaseRegistratorNode(const std::string& node_name, const rclcpp::NodeOptions& options);
        void init();
        virtual ~BaseRegistratorNode();
        void shutdown();

    public: // Types
        using SharedPtr = std::shared_ptr<BaseRegistratorNode>;

    protected: // Overridable methods
        virtual void onInit() {};
        virtual void onShutdown() {};

    protected: // Registration methods
        // Element registration
        void registerElement(const ID& element_id, const ElementType& element_type);
        void unregisterElement(const ID& element_id, const ElementType& element_type);

    private: // Update
        void publishRegistration();

    protected: // Components
        // Node
        NodePtr node_;
        const std::string node_name_;
        // Tools
        ConfigTools::SharedPtr config_tools_;
        ExternalTools::SharedPtr ext_tools_;
        TopicTools::SharedPtr topic_tools_;
        TfTools::SharedPtr tf_tools_;
        // Registered elements
        std::unordered_map<ID, ElementType> elements_;
        std::mutex elements_mutex_;
        // Registration publisher
        PublisherPtr<RegistrationMsg> registration_pub_;
        // Update timer
        TimerPtr update_timer_;
    };

} // namespace flychams::core