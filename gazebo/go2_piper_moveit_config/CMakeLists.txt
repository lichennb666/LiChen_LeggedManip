cmake_minimum_required(VERSION 3.0.2)
project(go2_piper_moveit_config)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
