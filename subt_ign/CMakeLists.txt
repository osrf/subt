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

find_package(ignition-gazebo3 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(ignition-common3 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs5 REQUIRED)
find_package(ignition-transport8 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
find_package(ignition-launch2 REQUIRED)
find_package(sdformat9 REQUIRED)

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
  ignition-msgs5::ignition-msgs5
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
    ignition-launch2::ignition-launch2
    ignition-math6::ignition-math6
    ignition-msgs5::ignition-msgs5
    ignition-plugin1::loader
    ignition-transport8::ignition-transport8
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
    ignition-launch2::ignition-launch2
    ignition-math6::ignition-math6
    ignition-msgs5::ignition-msgs5
    ignition-plugin1::loader
    ignition-transport8::ignition-transport8
    ${catkin_LIBRARIES}
    ${protobuf_lib_name}
)
install(TARGETS ${artifact_validator_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

set(connection_validator_name ConnectionValidator)
add_library(${connection_validator_name} SHARED
  src/ConnectionValidator.cc
  src/ConnectionValidatorPrivate.cc
  src/ConnectionHelper.cc
  src/SimpleDOTParser.cc)
target_link_libraries(${connection_validator_name}
  PRIVATE
    ignition-gazebo${IGN_GAZEBO_VER}::core
    ignition-common3::ignition-common3
    ignition-launch2::ignition-launch2
    ignition-math6::ignition-math6
    ignition-msgs5::ignition-msgs5
    ignition-plugin1::loader
    ignition-transport8::ignition-transport8
    ${catkin_LIBRARIES}
    ${protobuf_lib_name}
)
install(TARGETS ${connection_validator_name}
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
  ignition-msgs5::ignition-msgs5
  ignition-plugin1::loader
  ignition-launch2::ignition-launch2
  ignition-transport8::ignition-transport8
  sdformat9::sdformat9
  ${protobuf_lib_name}
  ${catkin_LIBRARIES}
)
install(TARGETS ${comms_broker_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libVisibilityPlugin.so library.
set(visibility_plugin_name VisibilityPlugin)
add_library(${visibility_plugin_name} SHARED
  src/VisibilityPlugin.cc
  src/VisibilityTable.cc
  src/SimpleDOTParser.cc
)
target_include_directories(${visibility_plugin_name}
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${visibility_plugin_name}
  PRIVATE
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-common3::ignition-common3
  ignition-math6::ignition-math6
  ignition-msgs5::ignition-msgs5
  ignition-plugin1::loader
  ignition-launch2::ignition-launch2
  ignition-transport8::ignition-transport8
  sdformat9::sdformat9
  ${protobuf_lib_name}
  ${catkin_LIBRARIES}
)
install(TARGETS ${visibility_plugin_name}
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
   ignition-launch2::ignition-launch2
   ignition-transport8::ignition-transport8
   sdformat9::sdformat9
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
   ignition-launch2::ignition-launch2
   ignition-transport8::ignition-transport8
   sdformat9::sdformat9
   ${catkin_LIBRARIES}
   ${protobuf_lib_name}
 )
 install(TARGETS ${controller_plugin_name}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin)

# Create the libGasEmitterDetectorPlugin.so library.
set(gas_emitter_detector_plugin_name GasEmitterDetectorPlugin)
add_library(${gas_emitter_detector_plugin_name} SHARED src/GasEmitterDetectorPlugin.cc)
target_include_directories(${gas_emitter_detector_plugin_name}
  PRIVATE
    ${PROTOBUF_INCLUDE_DIRS}
    ${CATKIN_DEVEL_PREFIX}/include)
target_link_libraries(${gas_emitter_detector_plugin_name}
  PRIVATE
    ignition-gazebo${IGN_GAZEBO_VER}::core
    ignition-common3::ignition-common3
    ignition-launch2::ignition-launch2
    ignition-math6::ignition-math6
    ignition-msgs5::ignition-msgs5
    ignition-plugin1::loader
    ignition-transport8::ignition-transport8
    ${catkin_LIBRARIES}
    ${protobuf_lib_name}
)
install(TARGETS ${gas_emitter_detector_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Create log_checker executable.
add_executable(log_checker src/LogChecker.cc)
target_link_libraries(log_checker
  ignition-transport8::ignition-transport8
  ignition-transport8::ignition-transport8-log)
install(TARGETS log_checker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(dot_generator src/dot_generator.cc src/ConnectionHelper.cc)
target_link_libraries(dot_generator
  ignition-math6::ignition-math6
  ignition-common3::ignition-common3
  sdformat9::sdformat9)
target_include_directories(dot_generator
  PRIVATE ${CATKIN_DEVEL_PREFIX}/include)
install(TARGETS dot_generator
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

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

if (CATKIN_ENABLE_TESTING)
  configure_file(test/test_config.hh.in ${CMAKE_CURRENT_BINARY_DIR}/test_config.hh)

  catkin_add_gtest(gas_emitter_detector test/gas_emitter_detector.cc)

  target_include_directories(
    gas_emitter_detector
    PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
  )

  target_link_libraries(gas_emitter_detector
    ignition-gazebo${IGN_GAZEBO_VER}::core
    ignition-common3::ignition-common3
  )
endif()


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
