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
  INCLUDE_DIRS
    include
  LIBRARIES
    CommsClient
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

# Protobuf.
add_subdirectory(src/protobuf)
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS}
                            PROPERTIES GENERATED TRUE)

# Create the libGameLogicPlugin.so library.
set(game_logic_plugin_name GameLogicPlugin)
add_library(${game_logic_plugin_name} src/GameLogicPlugin.cc)
target_link_libraries(${game_logic_plugin_name}
  ${catkin_LIBRARIES}
)
install(TARGETS ${game_logic_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libCommsBrokerPlugin.so library.
set(comms_broker_plugin_name CommsBrokerPlugin)
add_library(${comms_broker_plugin_name} src/CommsBrokerPlugin.cc)
target_link_libraries(${comms_broker_plugin_name}
  ${catkin_LIBRARIES}
  ${PROTO_SOURCES}
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
include_directories(${PROJECT_BINARY_DIR}/include)
target_link_libraries(${comms_client_name}
  ${catkin_LIBRARIES}
  ${PROTO_SOURCES}
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
    ${PROTO_HEADERS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

###########
## Tests ##
###########

configure_file (test/test_config.h.in
  ${PROJECT_BINARY_DIR}/include/test/test_config.h)

catkin_add_gtest(test_comms test/comms.cc)
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(test_comms
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  gazebo_test_fixture
  boost_thread
  ${comms_client_name}
)
add_dependencies(test_comms
  ${comms_broker_plugin_name}
  ${comms_client_name}
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
