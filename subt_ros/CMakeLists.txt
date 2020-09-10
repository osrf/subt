cmake_minimum_required(VERSION 2.8.3)
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

add_message_files(
  FILES
  CompetitionClock.msg
  CompetitionStats.msg
  Robot.msg
  Marsupial.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(ignition-common3 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs4 REQUIRED)
find_package(ignition-transport7 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    subt_communication_broker
    std_msgs
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

add_executable(optical_frame_publisher src/OpticalFramePublisher.cc)
target_link_libraries(optical_frame_publisher
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
    ignition-msgs4::ignition-msgs4
    ignition-transport7::ignition-transport7
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
    ignition-msgs4::ignition-msgs4
    ignition-transport7::ignition-transport7
    ${catkin_LIBRARIES}
)
add_dependencies(set_pose_relay ${catkin_EXPORTED_TARGETS})

install(TARGETS pose_tf_broadcaster subt_ros_relay set_pose_relay bridge_logger
  optical_frame_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rostopic_stats_logger.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
