cmake_minimum_required(VERSION 3.5.1)
project(subt_ign)

configure_file (${PROJECT_SOURCE_DIR}/include/subt_ign/Config.hh.in ${PROJECT_BINARY_DIR}/include/subt_ign/Config.hh)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")
endif()

set (SUBT_MAJOR_VERSION 0)
set (SUBT_MINOR_VERSION 1)
set (SUBT_PATCH_VERSION 0)
set (SUBT_VERSION_FULL
  ${SUBT_MAJOR_VERSION}.${SUBT_MINOR_VERSION}.${SUBT_PATCH_VERSION})

find_package(catkin REQUIRED
  COMPONENTS
    rostest
    subt_rf_interface
    subt_communication_model
    subt_communication_broker
)

find_package(ignition-gazebo2 2.1 QUIET REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo2_VERSION_MAJOR})

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(ignition-common3 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs4 REQUIRED)
find_package(ignition-transport7 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
find_package(ignition-launch1 REQUIRED)
find_package(sdformat8 REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}/include
  ${CATKIN_DEVEL_PREFIX}/include
)

link_directories()

catkin_package(
  CATKIN_DEPENDS
    subt_rf_interface
    subt_communication_model
    subt_communication_broker
  INCLUDE_DIRS
    include
    ${CATKIN_DEVEL_PREFIX}/include
  LIBRARIES
    SubtProtobuf
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

# Create a static library with the Protobuf messages used internally.
set(protobuf_lib_name SubtProtobuf)
add_subdirectory(src/protobuf)
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS}
                            PROPERTIES GENERATED TRUE)
add_library(${protobuf_lib_name} SHARED ${PROTO_SOURCES})
add_dependencies(${protobuf_lib_name} protobuf_compilation)
set_target_properties(${protobuf_lib_name}
  PROPERTIES POSITION_INDEPENDENT_CODE ON COMPILE_FLAGS "-Wno-unused-parameter" )
target_link_libraries(${protobuf_lib_name}
  PRIVATE
  ignition-msgs4::ignition-msgs4
)
install(TARGETS ${protobuf_lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/protobuf/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/protobuf
  FILES_MATCHING PATTERN "*.h")

catkin_add_env_hooks(ign SHELLS bash
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
catkin_add_env_hooks(ign SHELLS zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

# Create the libGameLogicPlugin.so library.
set(game_logic_plugin_name GameLogicPlugin)
add_library(${game_logic_plugin_name} SHARED src/GameLogicPlugin.cc)
target_include_directories(${game_logic_plugin_name}
  PRIVATE
    ${PROTOBUF_INCLUDE_DIRS}
    ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${game_logic_plugin_name}
  PRIVATE
    ignition-gazebo${IGN_GAZEBO_VER}::core
    ignition-common3::ignition-common3
    ignition-launch1::ignition-launch1
    ignition-math6::ignition-math6
    ignition-msgs4::ignition-msgs4
    ignition-plugin1::loader
    ignition-transport7::ignition-transport7
    ${catkin_LIBRARIES}
    ${protobuf_lib_name}
)
install(TARGETS ${game_logic_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

set(artifact_validator_name ArtifactValidator)
add_library(${artifact_validator_name} SHARED src/ArtifactValidator.cc)
target_link_libraries(${artifact_validator_name}
  PRIVATE
    ignition-gazebo${IGN_GAZEBO_VER}::core
    ignition-common3::ignition-common3
    ignition-launch1::ignition-launch1
    ignition-math6::ignition-math6
    ignition-msgs4::ignition-msgs4
    ignition-plugin1::loader
    ignition-transport7::ignition-transport7
    ${catkin_LIBRARIES}
    ${protobuf_lib_name}
)
install(TARGETS ${artifact_validator_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libCommsBrokerPlugin.so library.
set(comms_broker_plugin_name CommsBrokerPlugin)
add_library(${comms_broker_plugin_name}
  src/CommsBrokerPlugin.cc
  src/VisibilityRfModel.cc
  src/VisibilityTable.cc
  src/SimpleDOTParser.cc
)
target_include_directories(${comms_broker_plugin_name}
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${comms_broker_plugin_name}
  PUBLIC
  ignition-common3::ignition-common3
  ignition-math6::ignition-math6
  ignition-msgs4::ignition-msgs4
  ignition-plugin1::loader
  ignition-launch1::ignition-launch1
  ignition-transport7::ignition-transport7
  sdformat8::sdformat8
  ${protobuf_lib_name}
  ${catkin_LIBRARIES}
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
   ignition-common3::ignition-common3
   ignition-plugin1::loader
   ignition-launch1::ignition-launch1
   ignition-transport7::ignition-transport7
   sdformat8::sdformat8
   ${catkin_LIBRARIES}
   ${protobuf_lib_name}
 )
 install(TARGETS ${base_station_plugin_name}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin)

# Create the libControllerPlugin.so library.
set(controller_plugin_name ControllerPlugin)
add_library(${controller_plugin_name} src/ControllerPlugin.cc)
target_include_directories(${controller_plugin_name}
   PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
 target_link_libraries(${controller_plugin_name}
   ignition-common3::ignition-common3
   ignition-plugin1::loader
   ignition-launch1::ignition-launch1
   ignition-transport7::ignition-transport7
   sdformat8::sdformat8
   ${catkin_LIBRARIES}
   ${protobuf_lib_name}
 )
 install(TARGETS ${controller_plugin_name}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin)

##################
## Launch files ##
##################

file(GLOB launch_files "launch/*.ign.in")
foreach(launch_file ${launch_files})
  get_filename_component(name ${launch_file} NAME_WE)
  configure_file(
    "${launch_file}"
    "${CMAKE_CURRENT_BINARY_DIR}/launch/${name}.ign" @ONLY
  )
endforeach()

###########
## Tests ##
###########

find_package(rostest REQUIRED)

# test_score was ported from gazebo9, but doesn't work as rostest
# since it needs ign launch. so just compile it, but don't
# declare it as a rostest
#add_rostest_gtest(test_score test/score.test test/score.cc)
if(GTEST_FOUND)
  add_executable(test_score EXCLUDE_FROM_ALL test/score.cc)
  target_link_libraries(test_score ${GTEST_LIBRARIES})
  if(TARGET tests)
    add_dependencies(tests test_score)
  endif()
endif()
target_include_directories(test_score
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(test_score
  ${protobuf_lib_name}
  ${catkin_LIBRARIES}
)
add_dependencies(test_score
  ${game_logic_plugin_name}
)

#add_rostest_gtest(max_test_score test/max_score.test test/max_score.cc)
if(GTEST_FOUND)
  add_executable(max_test_score EXCLUDE_FROM_ALL test/max_score.cc)
  target_link_libraries(max_test_score ${GTEST_LIBRARIES})
  if(TARGET tests)
    add_dependencies(tests max_test_score)
  endif()
endif()
target_include_directories(max_test_score
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(max_test_score
  ${protobuf_lib_name}
  ${catkin_LIBRARIES}
)
add_dependencies(max_test_score
  ${game_logic_plugin_name}
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/subt_docker.bash
  scripts/subt_docker_list.bash
  scripts/subt_docker_uri.bash
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

install(FILES cmake/${PROJECT_NAME}-extras.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

# Install all the launch files
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the images
install(DIRECTORY images/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/images)
