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

# Create executable for registrator node
add_executable(registrator_node src/registrator_node.cpp)
ament_target_dependencies(registrator_node rclcpp flychams_core)

# Create executable for airsim settings parser
add_executable(airsim_settings_parser src/airsim_settings_parser.cpp)
ament_target_dependencies(airsim_settings_parser rclcpp flychams_core)

# Install targets and headers
install(TARGETS registrator_node airsim_settings_parser
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

ament_package()