cmake_minimum_required(VERSION 2.8.3)
project(bosdyn_spot)

set(CMAKE_CXX_STANDARD 17)

set(IGN_GAZEBO_VER 4)
find_package(ignition-gazebo${IGN_GAZEBO_VER} REQUIRED)

find_package(catkin REQUIRED)

catkin_package()

add_library(multi-joint-command-system src/multi_joint_command.cpp)
target_link_libraries(multi-joint-command-system
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::core
)

install(TARGETS multi-joint-command-system
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/clean_sdf scripts/high_precision_constants.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch meshes urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

