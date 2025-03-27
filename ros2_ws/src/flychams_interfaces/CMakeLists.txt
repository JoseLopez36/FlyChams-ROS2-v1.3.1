cmake_minimum_required(VERSION 3.8)
project(flychams_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Registration messages
  "msg/Element.msg"
  "msg/Registration.msg"

  # Agent messages
  "msg/AgentState.msg"
  "msg/PositionGoal.msg"
  "msg/TrackingInfo.msg"
  "msg/TrackingGoal.msg"

  # Target messages
  "msg/TargetInfo.msg"

  # Cluster messages
  "msg/ClusterInfo.msg"

  # Metrics messages
  "msg/AgentMetrics.msg"
  "msg/TargetMetrics.msg"
  "msg/ClusterMetrics.msg"
  "msg/GlobalMetrics.msg"

  # Basic messages
  "msg/Crop.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
