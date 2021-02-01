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

if(CATKIN_ENABLE_TESTING)
  add_executable(test_broker test/test_broker.cc)
  target_link_libraries(test_broker ${catkin_LIBRARIES} ignition-msgs6)
  add_dependencies(tests test_broker)

  find_package(rostest REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  include_directories(${rosgraph_msgs_INCLUDE_DIRS})
  include_directories(${GTEST_INCLUDE_DIRS})

  # we do not use add_rostest_gtest() because that doesn't allow for multiple
  # independent targets/binaries to be built... and it is desirable that the
  # tests are in separate binaries, because then rostest tears down all the
  # helper nodes between switching to the other test node... on the other hand,
  # keeping them all under one rostest command serializes the execution of the
  # tests (while three separate rostests would run in parallel).
  set(comms_types unicast multicast broadcast)
  set(comms_targets "")
  foreach(comms_type IN LISTS comms_types)
    set(target test_comms_client_${comms_type})
    add_executable(${target} EXCLUDE_FROM_ALL test/comms_relay_${comms_type}.cc)
    target_link_libraries(${target}
      ${catkin_LIBRARIES}
      ${rosgraph_msgs_LIBRARIES}
      ${GTEST_LIBRARIES}
      ignition-common3::ignition-common3
    )
    add_dependencies(tests ${target})
    list(APPEND comms_targets ${target})
  endforeach()

  add_rostest(test/comms_relay.test
    DEPENDENCIES subt_ros_relay test_broker ${comms_targets}
  )
endif()
