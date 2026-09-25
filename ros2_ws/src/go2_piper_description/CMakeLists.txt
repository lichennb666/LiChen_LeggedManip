cmake_minimum_required(VERSION 3.8)
project(go2_piper_description)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(LEGACY_URDF "${CMAKE_CURRENT_SOURCE_DIR}/../../../gazebo/go2_piper_description/urdf/go2_piper.urdf")
set(POLICY_CONTRACT "${CMAKE_CURRENT_SOURCE_DIR}/../go2_piper_wbc/config/policy_contract.yaml")
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/urdf")
set(GENERATED_URDF "${GENERATED_DIR}/go2_piper_ros2.urdf")
set(GENERATED_DEMO_URDF "${GENERATED_DIR}/go2_piper_demo.urdf")
set(GENERATED_SDF "${GENERATED_DIR}/go2_piper_ros2.sdf")
set(GENERATED_DEMO_SDF "${GENERATED_DIR}/go2_piper_demo.sdf")
set(STARTUP_POSE "${CMAKE_CURRENT_SOURCE_DIR}/config/startup_pose.json")
set(GENERATED_MESH_DIR "${CMAKE_CURRENT_BINARY_DIR}/meshes")

# Dereference the legacy workspace symlinks at configure time so the installed
# ROS 2 package contains complete visual meshes instead of depending on Gazebo
# resolving absolute host symlinks at runtime.
file(GLOB GO2_DAE_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../gazebo/go2_piper_description/meshes/go2_dae/*.dae")
file(GLOB PIPER_DAE_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../gazebo/go2_piper_description/meshes/piper_meshes/dae/*.dae")
file(MAKE_DIRECTORY "${GENERATED_MESH_DIR}/go2_dae" "${GENERATED_MESH_DIR}/piper_dae")
file(COPY ${GO2_DAE_FILES} DESTINATION "${GENERATED_MESH_DIR}/go2_dae")
file(COPY ${PIPER_DAE_FILES} DESTINATION "${GENERATED_MESH_DIR}/piper_dae")
add_custom_command(
  OUTPUT "${GENERATED_URDF}"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${GENERATED_DIR}"
  COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/convert_legacy_urdf.py"
          --input "${LEGACY_URDF}" --output "${GENERATED_URDF}"
          --contract "${POLICY_CONTRACT}" --startup-pose "${STARTUP_POSE}"
  DEPENDS "${LEGACY_URDF}" "${POLICY_CONTRACT}" "${STARTUP_POSE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/convert_legacy_urdf.py"
  VERBATIM
)
add_custom_command(
  OUTPUT "${GENERATED_DEMO_URDF}"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${GENERATED_DIR}"
  COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/convert_legacy_urdf.py"
          --input "${LEGACY_URDF}" --output "${GENERATED_DEMO_URDF}"
          --contract "${POLICY_CONTRACT}" --startup-pose "${STARTUP_POSE}" --demo-plugins
  DEPENDS "${LEGACY_URDF}" "${POLICY_CONTRACT}" "${STARTUP_POSE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/convert_legacy_urdf.py"
  VERBATIM
)
add_custom_target(generate_ros2_urdf ALL DEPENDS "${GENERATED_URDF}" "${GENERATED_DEMO_URDF}")

# Gazebo spawns the SDF (not the URDF) so that the robot can start directly in
# its policy natural pose via <initial_position>; the URDF is still used for
# robot_state_publisher and ros2_control description lookups.
add_custom_command(
  OUTPUT "${GENERATED_SDF}"
  COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_sdf.py"
          --input "${GENERATED_URDF}" --output "${GENERATED_SDF}" --pose "${STARTUP_POSE}"
  DEPENDS "${GENERATED_URDF}" "${STARTUP_POSE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_sdf.py"
  VERBATIM
)
add_custom_command(
  OUTPUT "${GENERATED_DEMO_SDF}"
  COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_sdf.py"
          --input "${GENERATED_DEMO_URDF}" --output "${GENERATED_DEMO_SDF}" --pose "${STARTUP_POSE}"
  DEPENDS "${GENERATED_DEMO_URDF}" "${STARTUP_POSE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_sdf.py"
  VERBATIM
)
add_custom_target(generate_ros2_sdf ALL DEPENDS "${GENERATED_SDF}" "${GENERATED_DEMO_SDF}")

install(FILES
  "${GENERATED_URDF}" "${GENERATED_DEMO_URDF}"
  "${GENERATED_SDF}" "${GENERATED_DEMO_SDF}"
  DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../../mujoco/robots/go2_piper/assets/"
  DESTINATION share/${PROJECT_NAME}/assets)
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../gazebo/go2_piper_description/meshes/livox_mid360.dae"
  DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY "${GENERATED_MESH_DIR}/"
  DESTINATION share/${PROJECT_NAME}/meshes)
ament_package()
