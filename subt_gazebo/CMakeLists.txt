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
)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(gazebo REQUIRED)
find_package(ignition-transport4 REQUIRED)
find_package(ignition-msgs1 REQUIRED)

########################
## Message generation ##
########################

# Add here SubT ROS messages.

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    subt_msgs
    std_srvs
  INCLUDE_DIRS
    include
  LIBRARIES
    CommsClient
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

# Create the libGameLogicPlugin.so library.
set(game_logic_plugin_name GameLogicPlugin)
add_library(${game_logic_plugin_name} src/GameLogicPlugin.cc)
add_dependencies(${game_logic_plugin_name} subt_msgs_generate_messages_cpp)
target_link_libraries(${game_logic_plugin_name}
  ${catkin_LIBRARIES}
)
install(TARGETS ${game_logic_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libCommsBrokerPlugin.so library.
set(comms_broker_plugin_name CommsBrokerPlugin)
add_library(${comms_broker_plugin_name}
  src/Broker.cc
  src/CommsBrokerPlugin.cc
  src/CommsModel.cc
)
target_include_directories(${comms_broker_plugin_name}
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${comms_broker_plugin_name}
  ${catkin_LIBRARIES}
  ${protobuf_lib_name}
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

# Create the libCommsClient.so library.
set(comms_client_name CommsClient)
add_library(${comms_client_name} src/CommsClient.cc)
target_include_directories(${comms_client_name}
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${comms_client_name}
  ${catkin_LIBRARIES}
  ${protobuf_lib_name}
  ignition-transport4::ignition-transport4
  protobuf::libprotobuf
)
add_dependencies(${comms_client_name} protobuf_compilation)
install(TARGETS ${comms_client_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
  FILES
    include/${PROJECT_NAME}/CommonTypes.hh
    include/${PROJECT_NAME}/CommsClient.hh
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  FILES
    ${PROTO_HEADERS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/protobuf
)

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
