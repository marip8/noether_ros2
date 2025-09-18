cmake_minimum_required(VERSION 3.5)
project(noether_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(noether_tpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Generate the messages
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(
  ${PROJECT_NAME}_msgs
  "msg/ToolPath.msg"
  "msg/ToolPaths.msg"
  "srv/PlanToolPath.srv"
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  pcl_msgs
)
rosidl_get_typesupport_target(msgs_target ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

# Library
add_library(${PROJECT_NAME} SHARED src/conversions.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} noether::noether_tpp ${msgs_target})
ament_target_dependencies(${PROJECT_NAME} pcl_conversions tf2_eigen)

# Tool path planning server
add_executable(${PROJECT_NAME}_tool_path_planning_server src/tool_path_planning_server.cpp)
target_link_libraries(${PROJECT_NAME}_tool_path_planning_server ${PROJECT_NAME} yaml-cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_tool_path_planning_server
  geometry_msgs
  pcl_msgs
  rclcpp
)

# Tool path planning client
add_executable(${PROJECT_NAME}_tool_path_planning_client src/tool_path_planning_client.cpp)
target_link_libraries(${PROJECT_NAME}_tool_path_planning_client ${PROJECT_NAME})
ament_target_dependencies(
  ${PROJECT_NAME}_tool_path_planning_client
  geometry_msgs
  pcl_msgs
  rclcpp
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION lib
)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

# Install the executable
install(TARGETS ${PROJECT_NAME}_tool_path_planning_server ${PROJECT_NAME}_tool_path_planning_client
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  builtin_interfaces
  geometry_msgs
  pcl_msgs
  pcl_conversions
  rosidl_default_runtime
  tf2_eigen
)
ament_package()
