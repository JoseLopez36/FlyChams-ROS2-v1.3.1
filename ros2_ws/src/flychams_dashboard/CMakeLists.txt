cmake_minimum_required(VERSION 3.8)
project(flychams_dashboard)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flychams_core REQUIRED)
find_package(OpenCV REQUIRED)

# Create dashboard library
set(DASHBOARD_SRC
  src/gui/gui_controller.cpp
  src/visualization/visualization_factory.cpp
  src/visualization/metrics_factory.cpp
  src/visualization/markers_factory.cpp
)
add_library(${PROJECT_NAME} SHARED ${DASHBOARD_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Export dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp flychams_core OpenCV
)

# Link to required libraries
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  ${OpenCV_LIBS}
)

# Create executable for GUI node
add_executable(gui_node src/gui_node.cpp)
target_link_libraries(gui_node ${PROJECT_NAME})
ament_target_dependencies(gui_node rclcpp flychams_core OpenCV)

# Create executable for visualization node
add_executable(visualization_node src/visualization_node.cpp)
target_link_libraries(visualization_node ${PROJECT_NAME})
ament_target_dependencies(visualization_node rclcpp flychams_core OpenCV)

# Install targets and headers
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS gui_node visualization_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Export package
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  flychams_core
  OpenCV
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()