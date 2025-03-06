# flychams_infrastructure

ROS infrastructure for the Flying Chameleons (FlyChams) project, providing ROS-specific implementations of core interfaces.

## Overview

The `flychams_ros` package is the bridge between the pure domain models in `flychams_core` and the ROS ecosystem. It implements:

1. **Message converters** between domain models and ROS messages
2. **Transformation management** using tf2
3. **Topic management** for standardized topic naming and access (subscribers and publishers)
4. **Element management** for agents, heads, targets and clusters
5. **Parameter management** for configuration and settings

## Components

### Managers

- `transform_manager.hpp` - Manages transformations between frames using tf2, handles frame naming patterns, and provides utilities for transforming messages between coordinate frames
- `topic_manager.hpp` - Manages topic naming patterns and provides factory methods for creating publishers and subscribers with standardized topic names
- `agent_manager.hpp` - Handles agent lifecycle and receives and publish its state

### Message Converters

- `message_converters.hpp` - Converts between domain models and ROS messages
- `ros_utils.hpp` - Utility functions for ROS operations, such ad parameter handling or timers creation

### ROS Types

- `ros_types.hpp` - Type definitions for ROS messages, publishers, subscribers, and other ROS components
- Custom message definitions for all domain objects (Agent, Head, Target, Cluster)

## Usage

To use this package in your project:

1. Add a dependency in your `package.xml`:
   ```xml
   <depend>flychams_infrastructure</depend>
   ```

2. Add a dependency in your `CMakeLists.txt`:
   ```cmake
   find_package(flychams_infrastructure REQUIRED)
   target_link_libraries(your_target flychams_infrastructure::flychams_infrastructure)
   ```

3. Include the necessary headers in your code:
   ```cpp
   #include "flychams_infrastructure/ros/data_service_ros.hpp"
   #include "flychams_infrastructure/ros/message_converters.hpp"
   ```

4. Create a data service instance:
   ```cpp
   auto node = std::make_shared<rclcpp::Node>("your_node");
   auto data_service = std::make_shared<flychams::infrastructure::DataServiceROS>(node);
   ```

## Architecture

This package is part of the layered architecture:

1. **Core Layer** (`flychams_core`) - Data models and interfaces
2. **Infrastructure Layer** (`flychams_infrastructure`) - ROS-specific implementations
3. **Domain-Specific Packages** - Domain logic implementation
4. **Application Layer** - High-level nodes and launch files

The infrastructure layer handles all ROS-specific concerns, allowing the domain logic to focus on business rules and algorithms. 