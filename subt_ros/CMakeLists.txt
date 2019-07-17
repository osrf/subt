cmake_minimum_required(VERSION 2.8.3)
project(subt_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2
  tf2_ros
  subt_msgs
)

find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs4 REQUIRED)
find_package(ignition-transport7 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Pose->TF broadcaster
add_executable(pose_tf_broadcaster
  src/pose_tf_broadcaster.cpp
)

target_link_libraries(pose_tf_broadcaster
  ${catkin_LIBRARIES}
)

add_executable(subt_ros_relay src/SubtRosRelay.cc)
target_link_libraries(subt_ros_relay
  PUBLIC
    ignition-math6::ignition-math6
    ignition-msgs4::ignition-msgs4
    ignition-transport7::ignition-transport7
    ${catkin_LIBRARIES}
)

install(TARGETS pose_tf_broadcaster subt_ros_relay
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
