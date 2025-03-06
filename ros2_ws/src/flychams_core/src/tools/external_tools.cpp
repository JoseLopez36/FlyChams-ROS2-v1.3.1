#include "flychams_core/tools/external_tools.hpp"

// Derived classes includes
#include "flychams_core/tools/airsim_tools.hpp"

namespace flychams::core
{
    ExternalTools::SharedPtr externalToolsFactory(NodePtr node, const Framework& framework)
    {
        // Create communication utility based on framework
        if (framework == Framework::AirSim)
        {
            return std::make_shared<AirsimTools>(node);
        }
        else
        {
            throw std::runtime_error("Unknown framework: " + std::to_string(static_cast<int>(framework)));
        }
    }

} // namespace flychams::core