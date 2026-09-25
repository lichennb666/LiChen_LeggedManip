cmake_minimum_required(VERSION 3.8)
project(open3d_loc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(open3d_registration SHARED
  src/registration.cpp
  src/pointcloud_conversions.cpp
)
target_include_directories(open3d_registration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(open3d_registration
  Eigen3::Eigen
  Open3D::Open3D
)
ament_target_dependencies(open3d_registration builtin_interfaces sensor_msgs)

add_executable(global_localization_node src/global_localization_node.cpp)
target_link_libraries(global_localization_node
  Eigen3::Eigen
  Open3D::Open3D
  open3d_registration
)
ament_target_dependencies(global_localization_node
  geometry_msgs
  builtin_interfaces
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS open3d_registration global_localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(open3d_registration)
ament_export_dependencies(Eigen3 Open3D builtin_interfaces sensor_msgs)
ament_package()
