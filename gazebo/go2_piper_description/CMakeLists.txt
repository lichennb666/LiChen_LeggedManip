cmake_minimum_required(VERSION 3.0.2)
project(go2_piper_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf meshes launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS object_detector.py full_station.py object_spawn.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
