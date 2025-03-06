#pragma once

// Tools includes
#include "flychams_core/tools/config_tools.hpp"
#include "flychams_core/tools/external_tools.hpp"
#include "flychams_core/tools/topic_tools.hpp"
#include "flychams_core/tools/tf_tools.hpp"

// Utils includes
#include "flychams_core/utils/msg_conversions.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief Base module for all modules in the core
     *
     * @details
     * This class is the base class for all modules present in the
     * packages. It provides a common interface for all modules and a
     * set of utilities for the modules to use.
     *
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class BaseModule
    {
    public: // Constructor/Destructor
        BaseModule(core::NodePtr node, core::ConfigTools::SharedPtr config_tools, core::ExternalTools::SharedPtr ext_tools, core::TopicTools::SharedPtr topic_tools, core::TfTools::SharedPtr tf_tools)
            : node_(node), config_tools_(config_tools), ext_tools_(ext_tools), topic_tools_(topic_tools), tf_tools_(tf_tools)
        {
            // Nothing to do
        }
        void init()
        {
            // Call on init overridable method
            onInit();
        }
        virtual ~BaseModule()
        {
            shutdown();
        }
        void shutdown()
        {
            // Call on shutdown overridable method
            onShutdown();
            // Destroy tools
            config_tools_.reset();
            ext_tools_.reset();
            topic_tools_.reset();
            tf_tools_.reset();
            // Destroy node
            node_.reset();
        }

    public: // Types
        using SharedPtr = std::shared_ptr<BaseModule>;

    protected: // Overridable methods
        virtual void onInit() {};
        virtual void onShutdown() {};

    protected: // Components
        // Node
        core::NodePtr node_;
        // Tools
        core::ConfigTools::SharedPtr config_tools_;
        core::ExternalTools::SharedPtr ext_tools_;
        core::TopicTools::SharedPtr topic_tools_;
        core::TfTools::SharedPtr tf_tools_;
    };

} // namespace flychams::core