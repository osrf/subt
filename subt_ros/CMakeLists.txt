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
  rosbag
)

add_message_files(
  FILES
  ArtifactReport.msg
  CompetitionClock.msg
  KinematicStates.msg
  Marsupial.msg
  RegionEvent.msg
  Robot.msg
  RobotEvent.msg
  RobotMetric.msg
  RunStatistics.msg
  RunStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(ignition-common3 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs6 REQUIRED)
find_package(ignition-transport9 REQUIRED)

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
target_link_libraries(bridge_logger
  PUBLIC
    ${catkin_LIBRARIES}
  PRIVATE
    ignition-common3::ignition-common3
)

add_executable(subt_ros_relay src/SubtRosRelay.cc)
target_include_directories(subt_ros_relay
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(subt_ros_relay
  PUBLIC
    ignition-common3::ignition-common3
    ignition-math6::ignition-math6
    ignition-msgs6::ignition-msgs6
    ignition-transport9::ignition-transport9
    ${catkin_LIBRARIES}
)
add_dependencies(subt_ros_relay ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(set_pose_relay src/SetPoseRelay.cc)
target_include_directories(set_pose_relay
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(set_pose_relay
  PUBLIC
    ignition-common3::ignition-common3
    ignition-math6::ignition-math6
    ignition-msgs6::ignition-msgs6
    ignition-transport9::ignition-transport9
    ${catkin_LIBRARIES}
)
add_dependencies(set_pose_relay ${catkin_EXPORTED_TARGETS})

add_executable(set_rate_relay src/SetRateRelay.cc)
target_include_directories(set_rate_relay
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(set_rate_relay
  PUBLIC
    ignition-common3::ignition-common3
    ignition-msgs6::ignition-msgs6
    ignition-transport9::ignition-transport9
    ${catkin_LIBRARIES}
)
add_dependencies(set_rate_relay ${catkin_EXPORTED_TARGETS})

install(TARGETS pose_tf_broadcaster subt_ros_relay set_pose_relay bridge_logger
  optical_frame_publisher set_rate_relay
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rostopic_stats_logger.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
