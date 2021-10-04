cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(subt_communication_broker)

set(PKG_DEPENDS
  roscpp subt_communication_model subt_rf_interface subt_msgs
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

find_package(ignition-common4 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
  ${ignition-transport11_INCLUDE_DIRS}
  ${ignition-msgs8_INCLUDE_DIRS}
  ${ignition-common4_INCLUDE_DIRS}
  )

set(project_libs
  ${catkin_LIBRARIES}
  ${ignition-transport11_LIBRARIES}
  ${ignition-msgs8_LIBRARIES}
  ${ignition-common4_LIBRARIES})

#######################################
## Declare Catkin package and installation
#######################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${RUN_DEPENDS}
  LIBRARIES subt_communication_broker subt_communication_client SubtCommsProtobuf
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

set(protobuf_lib_name SubtCommsProtobuf)
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

add_library(subt_communication_broker src/subt_communication_broker.cpp)
target_link_libraries(subt_communication_broker ${project_libs} ${protobuf_lib_name})
add_dependencies(subt_communication_broker ${protobuf_lib_name})

add_library(subt_communication_client src/subt_communication_client.cpp)
target_link_libraries(subt_communication_client ${project_libs} ${protobuf_lib_name})
add_dependencies(subt_communication_client ${protobuf_lib_name} ${catkin_EXPORTED_TARGETS})

install(TARGETS subt_communication_broker subt_communication_client SubtCommsProtobuf
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
    subt_communication_broker
    subt_communication_client
    ${project_libs}
    ${protobuf_lib_name}
    ${GTEST_LIBRARIES})
endif()
