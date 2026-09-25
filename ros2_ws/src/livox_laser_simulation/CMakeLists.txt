cmake_minimum_required(VERSION 3.8)
project(livox_laser_simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Gazebo rewrites package:// mesh URIs to model:// URIs when it converts the
# robot description to SDF. Export this package's share parent so those model
# URIs can be resolved at runtime.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/livox_laser_simulation.dsv.in"
)

find_library(GAZEBO_RAY_PLUGIN
  NAMES RayPlugin
  PATHS ${GAZEBO_LIBRARY_DIRS} /usr/lib/x86_64-linux-gnu/gazebo-11/plugins
)
if(NOT GAZEBO_RAY_PLUGIN)
  message(FATAL_ERROR "Gazebo RayPlugin library was not found")
endif()

add_library(livox_laser_simulation SHARED
  src/livox_points_plugin.cpp
  src/livox_ode_multiray_shape.cpp
)

target_include_directories(livox_laser_simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(livox_laser_simulation
  ament_index_cpp
  builtin_interfaces
  gazebo_dev
  gazebo_ros
  rclcpp
  sensor_msgs
)
target_link_libraries(livox_laser_simulation ${GAZEBO_RAY_PLUGIN})

install(TARGETS livox_laser_simulation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY meshes scan_mode
  DESTINATION share/${PROJECT_NAME}
)
install(FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(livox_laser_simulation)
ament_export_dependencies(
  ament_index_cpp builtin_interfaces gazebo_dev gazebo_ros rclcpp sensor_msgs)
ament_package()
