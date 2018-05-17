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
    std_msgs
    std_srvs
    message_generation
)

find_package(gazebo REQUIRED)

########################
## Message generation ##
########################

# Add here SubT messages.

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
    std_srvs
)

# Create the libGameLogicPlugin.so library.
set(game_logic_plugin_name GameLogicPlugin)
add_library(${game_logic_plugin_name} src/GameLogicPlugin.cc)
target_link_libraries(${game_logic_plugin_name}
  ${catkin_LIBRARIES}
  ${PROTO_SRC}
)
add_dependencies(game_logic_plugin_name protobuf_compilation)
install(TARGETS ${game_logic_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libCommsClient.so library.
add_subdirectory(src/protobuf)
set(comms_client_name CommsClient)
add_library(${comms_client_name} src/CommsClient.cc)
target_link_libraries(${comms_client_name}
  ${catkin_LIBRARIES}
)
install(TARGETS ${comms_client_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#############
## Install ##
#############

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)
