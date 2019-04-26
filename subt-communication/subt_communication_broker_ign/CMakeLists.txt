cmake_minimum_required(VERSION 2.8.3)
project(subt_communication_broker_ign)

set(PKG_DEPENDS
  roscpp subt_communication_model subt_rf_interface
  )
set(BUILD_DEPENDS
  ${PKG_DEPENDS}
  )
set(RUN_DEPENDS
  ${PKG_DEPENDS}
  )

#######################################
## Find dependencies
#######################################

find_package(catkin REQUIRED ${BUILD_DEPENDS})

find_package(ignition-common3 REQUIRED)
find_package(ignition-msgs3 REQUIRED)
find_package(ignition-transport6 REQUIRED)

include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
  ${ignition-transport6_INCLUDE_DIRS}
  ${ignition-msgs3_INCLUDE_DIRS}
  ${ignition-common3_INCLUDE_DIRS}
  )

set(project_libs
  ${catkin_LIBRARIES}
  ${ignition-transport6_LIBRARIES}
  ${ignition-msgs3_LIBRARIES}
  ${ignition-common3_LIBRARIES})

#######################################
## Declare Catkin package and installation
#######################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${RUN_DEPENDS}
  LIBRARIES subt_communication_broker_ign subt_communication_client_ign SubtCommsIgnProtobuf
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
    )

set(protobuf_lib_name SubtCommsIgnProtobuf)
add_subdirectory(src/protobuf)
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS}
                            PROPERTIES GENERATED TRUE)
add_library(${protobuf_lib_name} SHARED ${PROTO_SOURCES})
add_dependencies(${protobuf_lib_name} ${PROJECT_NAME}_protobuf_compilation)
set_target_properties(${protobuf_lib_name}
  PROPERTIES POSITION_INDEPENDENT_CODE ON COMPILE_FLAGS "-Wno-unused-parameter")

install(
  FILES
    ${PROTO_HEADERS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/protobuf
)

add_library(subt_communication_broker_ign src/subt_communication_broker.cpp)
target_link_libraries(subt_communication_broker_ign ${project_libs} ${protobuf_lib_name})
add_dependencies(subt_communication_broker_ign ${protobuf_lib_name})

add_library(subt_communication_client_ign src/subt_communication_client.cpp)
target_link_libraries(subt_communication_client_ign ${project_libs} ${protobuf_lib_name})
add_dependencies(subt_communication_client_ign ${protobuf_lib_name})

install(TARGETS subt_communication_broker_ign subt_communication_client_ign SubtCommsIgnProtobuf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )


if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_unit_test tests/unit_test.cpp)
  target_link_libraries(${PROJECT_NAME}_unit_test
    subt_communication_broker_ign
    subt_communication_client_ign
    ${project_libs}
    ${protobuf_lib_name}
    ${GTEST_LIBRARIES})
endif()
