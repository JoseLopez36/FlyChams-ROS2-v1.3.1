cmake_minimum_required(VERSION 3.8)
project(flychams_core)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(airsim_interfaces REQUIRED)
find_package(flychams_interfaces REQUIRED)

# Add external libraries
add_subdirectory(external/OpenXLSX ${CMAKE_CURRENT_BINARY_DIR}/OpenXLSX_build)
add_subdirectory(external/json ${CMAKE_CURRENT_BINARY_DIR}/json_build)

# Include directories
include_directories(
  include
)

# Declare the library
add_library(${PROJECT_NAME}
  src/config/config_parser.cpp
  src/tools/external_tools.cpp
  src/tools/airsim_tools.cpp
  src/base/registrator_node.cpp
  src/base/discoverer_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/json/single_include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
    OpenXLSX::OpenXLSX
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_geometry_msgs
  airsim_interfaces
  flychams_interfaces
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY external/OpenXLSX/OpenXLSX/headers/
  DESTINATION include/OpenXLSX
)
install(
  DIRECTORY external/OpenXLSX/OpenXLSX/external/
  DESTINATION include/external
)
install(
  DIRECTORY external/json/single_include/
  DESTINATION include
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install and export OpenXLSX
install(TARGETS OpenXLSX
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_geometry_msgs
  airsim_interfaces
  flychams_interfaces
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package() 