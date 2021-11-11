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
    roslib
    rostest
    subt_rf_interface
    subt_communication_model
    subt_communication_broker
    subt_ros
    rosbag
)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(ignition-common4 REQUIRED COMPONENTS graphics)
find_package(ignition-fuel_tools7 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-sensors6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
find_package(ignition-launch5 REQUIRED)
find_package(sdformat12 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL_PC REQUIRED fcl)
# find *absolute* paths to LIBFCL_INCLUDE_DIRS and LIBFCL_LIBRARIES
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_PC_INCLUDE_DIR} ${LIBFCL_PC_INCLUDE_DIRS})
find_library(LIBFCL_LIBRARIES fcl HINTS ${LIBFCL_PC_LIBRARY_DIRS})

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}/include
  ${CATKIN_DEVEL_PREFIX}/include
)

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
  ignition-msgs8::ignition-msgs8
)
install(TARGETS ${protobuf_lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/protobuf/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/protobuf
  FILES_MATCHING PATTERN "*.h")

catkin_add_env_hooks(30.subt_ign SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

add_library(SubtCommon
  SHARED
  src/Common.cc
  src/ConnectionValidatorPrivate.cc
  src/ConnectionHelper.cc
  src/ign_to_fcl.cc
  src/SdfParser.cc
  src/SimpleDOTParser.cc
  src/VisibilityRfModel.cc
  src/VisibilityTable.cc
)

target_include_directories(SubtCommon
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(SubtCommon
  PUBLIC
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-common4::ignition-common4
  ignition-common4::graphics
  ignition-transport11::ignition-transport11
  ignition-transport11::ignition-transport11-log
  ignition-sensors6::ignition-sensors6
  ignition-launch5::ignition-launch5
  ignition-fuel_tools7::ignition-fuel_tools7
  ${catkin_LIBRARIES}
  ${LIBFCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

list(APPEND SUBT_IGN_PLUGINS
  ArtifactValidator
  BaseStationPlugin
  CommsBrokerPlugin
  ConnectionValidator
  ControllerPlugin
  GameLogicPlugin
  GasEmitterDetectorPlugin
  LocalWindPlugin
  VisibilityPlugin
)

foreach(PLUGIN ${SUBT_IGN_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC SubtCommon SubtProtobuf)
endforeach()

# Create the libPlaybackEventRecorder.so library.
set(playback_event_recorder_name PlaybackEventRecorder)
add_library(${playback_event_recorder_name} SHARED
  src/PlaybackEventRecorder.cc
)

target_include_directories(${playback_event_recorder_name}
  PRIVATE
    ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${playback_event_recorder_name}
  PRIVATE
    ignition-gazebo${IGN_GAZEBO_VER}::core
    ignition-common4::ignition-common4
    ignition-launch5::ignition-launch5
    ignition-math6::ignition-math6
    ignition-msgs8::ignition-msgs8
    ignition-plugin1::loader
    ignition-transport11::ignition-transport11
    sdformat12::sdformat12
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

install(TARGETS ${playback_event_recorder_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Create the libPlaybackSensors.so library.
set(playback_sensors_name PlaybackSensors)
add_library(${playback_sensors_name} SHARED
  src/PlaybackSensors.cc
)

target_include_directories(${playback_sensors_name}
  PRIVATE
    ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${playback_sensors_name}
  PRIVATE
    ignition-gazebo${IGN_GAZEBO_VER}::core
    ignition-common4::ignition-common4
    ignition-math6::ignition-math6
    ignition-msgs8::ignition-msgs8
    ignition-plugin1::loader
    ignition-transport11::ignition-transport11
    ignition-sensors6::ignition-sensors6
    ignition-sensors6::camera
    ignition-rendering6::ignition-rendering6
    sdformat12::sdformat12
    ignition-gazebo${IGN_GAZEBO_VER}-rendering
    ${catkin_LIBRARIES}
)

install(TARGETS ${playback_sensors_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(
  TARGETS SubtProtobuf SubtCommon ${SUBT_IGN_PLUGINS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(validate_connections src/apps/validate_connections.cc)
target_link_libraries(validate_connections ConnectionValidator)

add_executable(path_tracer src/apps/path_tracer.cc)
target_link_libraries(path_tracer SubtCommon)

add_executable(validate_visibility_table src/apps/validate_visibility_table.cc)
target_link_libraries(validate_visibility_table SubtCommon)

# Create log_checker executable.
add_executable(log_checker src/apps/LogChecker.cc)
target_link_libraries(log_checker SubtCommon)

add_executable(dot_generator src/apps/dot_generator.cc)
target_link_libraries(dot_generator SubtCommon)

add_executable(level_generator src/apps/level_generator.cc)
target_link_libraries(level_generator SubtCommon)

add_executable(cave_generator src/apps/cave_generator.cc src/apps/world_generator_base.cc src/apps/world_generator_utils.cc)
target_link_libraries(cave_generator SubtCommon)

add_executable(urban_generator src/apps/urban_generator.cc src/apps/world_generator_base.cc src/apps/world_generator_utils.cc)
target_link_libraries(urban_generator SubtCommon)

add_executable(tunnel_generator src/apps/tunnel_generator.cc src/apps/world_generator_base.cc src/apps/world_generator_utils.cc)
target_link_libraries(tunnel_generator SubtCommon)

###########
## Tests ##
###########

find_package(rostest REQUIRED)
if (CATKIN_ENABLE_TESTING)
  configure_file(test/test_config.hh.in ${CMAKE_CURRENT_BINARY_DIR}/test_config.hh)

  # Score Test
  add_executable(test_score test/score.cc)
  target_link_libraries(test_score SubtCommon SubtProtobuf ${GTEST_LIBRARIES})
  add_dependencies(test_score GameLogicPlugin)

  # Max Score Test
  add_executable(max_test_score test/max_score.cc)
  target_link_libraries(max_test_score SubtCommon SubtProtobuf ${GTEST_LIBRARIES})
  add_dependencies(max_test_score GameLogicPlugin)

  # GasEmitterDetector Test
  catkin_add_gtest(gas_emitter_detector_TEST test/gas_emitter_detector.cc)
  target_include_directories(gas_emitter_detector_TEST PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(gas_emitter_detector_TEST SubtCommon)

  # Common Test
  catkin_add_gtest(common_TEST test/Common_TEST.cc)
  target_include_directories(common_TEST PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(common_TEST SubtCommon)
endif()


#############
## Install ##
#############

install(
  TARGETS
    validate_connections
    path_tracer
    validate_visibility_table
    log_checker
    dot_generator
    level_generator
    cave_generator
    tunnel_generator
    urban_generator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Unzip the cave_circuit_practice_02.dat.zip file
add_custom_target(unzip_dat ALL)
add_custom_command(TARGET unzip_dat PRE_BUILD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds
  COMMAND ${CMAKE_COMMAND} -E tar xvf cave_circuit_practice_02.dat.zip
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/worlds/cave_circuit_practice_02.dat.zip
COMMENT "Unpacking cave_circuit_practice_02.dat.zip"
VERBATIM)

# Unzip the cave_circuit_03.dat.zip file
add_custom_command(TARGET unzip_dat PRE_BUILD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds/cave_circuit/03
  COMMAND ${CMAKE_COMMAND} -E tar xvf cave_circuit_03.dat.zip
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/worlds/cave_circuit/03/cave_circuit_03.dat.zip
COMMENT "Unpacking cave_circuit_03.dat.zip"
VERBATIM)

# Unzip the cave_circuit_04.dat.zip file
add_custom_command(TARGET unzip_dat PRE_BUILD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds/cave_circuit/04
  COMMAND ${CMAKE_COMMAND} -E tar xvf cave_circuit_04.dat.zip
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/worlds/cave_circuit/04/cave_circuit_04.dat.zip
COMMENT "Unpacking cave_circuit_04.dat.zip"
VERBATIM)

# Unzip the cave_circuit_07.dat.zip file
add_custom_command(TARGET unzip_dat PRE_BUILD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds/cave_circuit/07/
  COMMAND ${CMAKE_COMMAND} -E tar xvf cave_circuit_07.dat.zip
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/worlds/cave_circuit/07/cave_circuit_07.dat.zip
  BYPRODUCTS ${CMAKE_CURRENT_SOURCE_DIR}/worlds/cave_circuit/07/cave_circuit_07.dat
COMMENT "Unpacking cave_circuit_07.dat.zip"
VERBATIM)

# Unzip the cave_circuit_08.dat.zip file
add_custom_command(TARGET unzip_dat PRE_BUILD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds/cave_circuit/08
  COMMAND ${CMAKE_COMMAND} -E tar xvf cave_circuit_08.dat.zip
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/worlds/cave_circuit/08/cave_circuit_08.dat.zip
COMMENT "Unpacking cave_circuit_08.dat.zip"
VERBATIM)

# Unzip the finals_practice_03.dat.zip file
add_custom_command(TARGET unzip_dat PRE_BUILD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds
  COMMAND ${CMAKE_COMMAND} -E tar xvf finals_practice_03.dat.zip
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/worlds/finals_practice_03.dat.zip
COMMENT "Unpacking finals_practice_03.dat.zip"
VERBATIM)

# Unzip the final_event_03.dat.zip file
add_custom_command(TARGET unzip_dat PRE_BUILD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds/final_event/03
  COMMAND ${CMAKE_COMMAND} -E tar xvf final_event_03.dat.zip
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/worlds/final_event/03/final_event_03.dat.zip
COMMENT "Unpacking final_event_03.dat.zip"
VERBATIM)

# Unzip the final_event/06/final_event_06.dat.zip file
add_custom_command(TARGET unzip_dat PRE_BUILD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds/final_event/06
  COMMAND ${CMAKE_COMMAND} -E tar xvf final_event_06.dat.zip
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/worlds/final_event/06/final_event_06.dat.zip
COMMENT "Unpacking final_event_06.dat.zip"
VERBATIM)

# Unzip the final_event_07.dat.zip file
add_custom_command(TARGET unzip_dat PRE_BUILD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/worlds/final_event/07
  COMMAND ${CMAKE_COMMAND} -E tar xvf final_event_07.dat.zip
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/worlds/final_event/07/final_event_07.dat.zip
COMMENT "Unpacking final_event_07.dat.zip"
VERBATIM)

install(PROGRAMS
  scripts/subt_docker.bash
  scripts/subt_docker_list.bash
  scripts/subt_docker_uri.bash
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

install(FILES cmake/${PROJECT_NAME}-extras.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the images
install(DIRECTORY images/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/images)
