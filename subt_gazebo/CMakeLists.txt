cmake_minimum_required(VERSION 3.5.1)
project(subt_gazebo)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

set (SUBT_MAJOR_VERSION 0)
set (SUBT_MINOR_VERSION 1)
set (SUBT_PATCH_VERSION 0)
set (SUBT_VERSION_FULL
  ${SUBT_MAJOR_VERSION}.${SUBT_MINOR_VERSION}.${SUBT_PATCH_VERSION})

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_ros
    gazebo_plugins
    roscpp
    geometry_msgs
    std_msgs
    std_srvs
    subt_msgs
    message_generation
    subt_rf_interface
    subt_communication_model
    subt_communication_broker
    subt_gazebo_los_model
)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(gazebo REQUIRED)
find_package(ignition-common1 REQUIRED)
find_package(ignition-math4 REQUIRED)
find_package(ignition-msgs1 REQUIRED)
find_package(ignition-transport4 REQUIRED)

########################
## Message generation ##
########################

# Add here SubT ROS messages.

###########
## Build ##
###########

include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_srvs
    subt_rf_interface
    subt_communication_model
    subt_communication_broker
    subt_gazebo_los_model
  INCLUDE_DIRS
  include
  ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

# Create a static library with the Protobuf messages used internally.
set(protobuf_lib_name SubtProtobuf)
add_subdirectory(src/protobuf)
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS}
                            PROPERTIES GENERATED TRUE)
add_library(${protobuf_lib_name} STATIC ${PROTO_SOURCES})
add_dependencies(${protobuf_lib_name} protobuf_compilation)
set_target_properties(${protobuf_lib_name}
  PROPERTIES POSITION_INDEPENDENT_CODE ON COMPILE_FLAGS "-Wno-unused-parameter")

# Create the libJointMotionTimerPlugin.so library.
set(joint_motion_timer_plugin_name JointMotionTimerPlugin)
add_library(${joint_motion_timer_plugin_name} src/JointMotionTimerPlugin.cc)
target_link_libraries(${joint_motion_timer_plugin_name}
  ${catkin_LIBRARIES}
)
install(TARGETS ${joint_motion_timer_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libMotionTimerDetachPlugin.so library.
set(motion_timer_detach_plugin_name MotionTimerDetachPlugin)
add_library(${motion_timer_detach_plugin_name} src/MotionTimerDetachPlugin.cc)
target_link_libraries(${motion_timer_detach_plugin_name}
  ${joint_motion_timer_plugin_name}
  ${catkin_LIBRARIES}
)
install(TARGETS ${motion_timer_detach_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libMotionTimerLockPlugin.so library.
set(motion_timer_lock_plugin_name MotionTimerLockPlugin)
add_library(${motion_timer_lock_plugin_name} src/MotionTimerLockPlugin.cc)
target_link_libraries(${motion_timer_lock_plugin_name}
  ${joint_motion_timer_plugin_name}
  ${catkin_LIBRARIES}
)
install(TARGETS ${motion_timer_lock_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libGameLogicPlugin.so library.
set(game_logic_plugin_name GameLogicPlugin)
add_library(${game_logic_plugin_name} src/GameLogicPlugin.cc)
add_dependencies(${game_logic_plugin_name} subt_msgs_generate_messages_cpp)
target_include_directories(${game_logic_plugin_name}
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${game_logic_plugin_name}
  ${catkin_LIBRARIES}
  ${protobuf_lib_name}
)
install(TARGETS ${game_logic_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libCommsBrokerPlugin.so library.
set(comms_broker_plugin_name CommsBrokerPlugin)
add_library(${comms_broker_plugin_name}
  src/CommsBrokerPlugin.cc
)
target_include_directories(${comms_broker_plugin_name}
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${comms_broker_plugin_name}
  PUBLIC
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ignition-math4::ignition-math4
  ${ignition-transport4_LIBRARIES}
)
add_dependencies(${comms_broker_plugin_name}
  protobuf_compilation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${comms_broker_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libBaseStationPlugin.so library.
set(base_station_plugin_name BaseStationPlugin)
add_library(${base_station_plugin_name} src/BaseStationPlugin.cc)
target_include_directories(${base_station_plugin_name}
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${base_station_plugin_name}
  ${catkin_LIBRARIES}
  ignition-common1::ignition-common1
  ${protobuf_lib_name}
)
install(TARGETS ${base_station_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libRosFlashLightPlugin.so library.
set(ros_flashlight_plugin_name RosFlashLightPlugin)
add_library(${ros_flashlight_plugin_name} src/RosFlashLightPlugin.cc)
target_link_libraries(${ros_flashlight_plugin_name}
  ${catkin_LIBRARIES}
  LedPlugin
)
install(TARGETS ${ros_flashlight_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libLightVisualPlugin.so library.
set(light_visual_plugin_name LightVisualPlugin)
add_library(${light_visual_plugin_name} src/LightVisualPlugin.cc)
target_link_libraries(${light_visual_plugin_name}
  ${catkin_LIBRARIES}
)
install(TARGETS ${light_visual_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

###########
## Tests ##
###########

configure_file (test/test_config.h.in
  ${CATKIN_DEVEL_PREFIX}/include/test/test_config.h)

find_package(rostest REQUIRED)

add_rostest_gtest(test_score test/score.test test/score.cc)
target_include_directories(test_score
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(test_score
  ${protobuf_lib_name}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(test_score
  ${game_logic_plugin_name}
)

#############
## Install ##
#############

install(FILES cmake/${PROJECT_NAME}-extras.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install all the model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

install(PROGRAMS
  scripts/world_generator
  scripts/benchmark.launch.template
  scripts/benchmark.world.template
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
