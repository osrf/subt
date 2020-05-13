cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(subt_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  tf2
  tf2_ros
  topic_tools
  subt_msgs
  subt_communication_broker
)

find_package(ignition-common3 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs5 REQUIRED)
find_package(ignition-transport8 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    subt_communication_broker
    topic_tools
  )

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

add_executable(bridge_logger src/BridgeLogger.cc)
add_dependencies(bridge_logger ${catkin_EXPORTED_TARGETS})
target_link_libraries(bridge_logger ${catkin_LIBRARIES})

add_executable(subt_ros_relay src/SubtRosRelay.cc)
target_include_directories(subt_ros_relay
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(subt_ros_relay
  PUBLIC
    ignition-common3::ignition-common3
    ignition-math6::ignition-math6
    ignition-msgs5::ignition-msgs5
    ignition-transport8::ignition-transport8
    ${catkin_LIBRARIES}
)
add_dependencies(subt_ros_relay ${catkin_EXPORTED_TARGETS})

add_executable(set_pose_relay src/SetPoseRelay.cc)
target_include_directories(set_pose_relay
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(set_pose_relay
  PUBLIC
    ignition-common3::ignition-common3
    ignition-math6::ignition-math6
    ignition-msgs5::ignition-msgs5
    ignition-transport8::ignition-transport8
    ${catkin_LIBRARIES}
)
add_dependencies(set_pose_relay ${catkin_EXPORTED_TARGETS})

install(TARGETS pose_tf_broadcaster subt_ros_relay set_pose_relay bridge_logger
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rostopic_stats_logger.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
