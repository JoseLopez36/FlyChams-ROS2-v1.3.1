cmake_minimum_required(VERSION 3.8)
project(flychams_bringup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flychams_core REQUIRED)

# Create bringup library
set(BRINGUP_SRC
  src/target/target_controller.cpp
)
add_library(${PROJECT_NAME} SHARED ${BRINGUP_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp flychams_core
)

# Link to required libraries
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
)

# Create executable for agent registration node
add_executable(agent_registration_node src/agent_registration_node.cpp)
target_link_libraries(agent_registration_node ${PROJECT_NAME})
ament_target_dependencies(agent_registration_node rclcpp flychams_core)

# Create executable for target registration node
add_executable(target_registration_node src/target_registration_node.cpp)
target_link_libraries(target_registration_node ${PROJECT_NAME})
ament_target_dependencies(target_registration_node rclcpp flychams_core)

# Create executable for cluster registration node
add_executable(cluster_registration_node src/cluster_registration_node.cpp)
target_link_libraries(cluster_registration_node ${PROJECT_NAME})
ament_target_dependencies(cluster_registration_node rclcpp flychams_core)

# Create executable for airsim settings node
add_executable(airsim_settings_node src/airsim_settings_node.cpp)
target_link_libraries(airsim_settings_node ${PROJECT_NAME})
ament_target_dependencies(airsim_settings_node rclcpp flychams_core)

# Install targets and headers
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS agent_registration_node target_registration_node cluster_registration_node airsim_settings_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export package
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  flychams_core
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()