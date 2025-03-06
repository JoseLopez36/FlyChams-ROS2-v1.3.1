#pragma once

// Core includes
#include "flychams_core/types/core_types.hpp"
#include "flychams_core/types/ros_types.hpp"
#include "flychams_core/utils/ros_utils.hpp"

namespace flychams::core
{
    /**
     * ════════════════════════════════════════════════════════════════
     * @brief External Communication Manager for handling communication
     * with the external framework
     *
     * @details
     * This class provides utilities for managing the communication
     * with the external framework.
     * ════════════════════════════════════════════════════════════════
     * @author Jose Francisco Lopez Ruiz
     * @date 2025-02-28
     * ════════════════════════════════════════════════════════════════
     */
    class ExternalTools
    {
    public: // Constructors/Destructors
        virtual ~ExternalTools() = default;
        virtual void shutdown() = 0;

    public: // Types
        using SharedPtr = std::shared_ptr<ExternalTools>;

    public: // Vehicle adders (override)
        virtual void addVehicle(const ID& vehicle_id) = 0;
        virtual void removeVehicle(const ID& vehicle_id) = 0;

    public: // Status methods (override)
        virtual void waitConnection() const = 0;
        virtual bool isConnected() const = 0;
        virtual void waitRunning() const = 0;
        virtual bool isRunning() const = 0;

    public: // Global control methods (override)
        virtual bool resetSimulation() = 0;
        virtual bool runSimulation() = 0;
        virtual bool pauseSimulation() = 0;

    public: // Vehicle control methods (override)
        virtual bool takeoffVehicleGroup(const IDs& vehicle_ids) = 0;
        virtual bool landVehicleGroup(const IDs& vehicle_ids) = 0;
        virtual bool hoverVehicleGroup(const IDs& vehicle_ids) = 0;
        virtual void setVehicleVelocity(const ID& vehicle_id, const float& vel_cmd_x, const float& vel_cmd_y, const float& vel_cmd_z, const float& vel_cmd_dt, const std::string& frame_id) = 0;
        virtual void setGimbalAngles(const ID& vehicle_id, const IDs& camera_ids, const std::vector<QuaternionMsg>& target_quats, const std::string& frame_id) = 0;
        virtual void setCameraFovs(const ID& vehicle_id, const IDs& camera_ids, const std::vector<float>& target_fovs, const std::string& frame_id) = 0;

    public: // Object control methods (override)
        virtual bool spawnObjectGroup(const IDs& object_ids, const std::vector<PoseMsg>& poses, const std::vector<float>& scales, const std::vector<TargetType>& object_types) = 0;
        virtual bool despawnObjectGroup(const IDs& object_ids) = 0;
        virtual void setObjectPoseGroup(const IDs& object_ids, const std::vector<PoseMsg>& poses, const std::string& frame_id) = 0;

    public: // Window control methods (override)
        virtual void setWindowImageGroup(const IDs& window_ids, const IDs& vehicle_ids, const IDs& camera_ids, const std::vector<int>& crop_x, const std::vector<int>& crop_y, const std::vector<int>& crop_w, const std::vector<int>& crop_h) = 0;
    };

    ExternalTools::SharedPtr externalToolsFactory(NodePtr node, const Framework& framework);

} // namespace flychams::core