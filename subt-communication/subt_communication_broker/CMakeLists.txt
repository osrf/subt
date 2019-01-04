cmake_minimum_required(VERSION 2.8.3)
project(subt_communication_broker)

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

find_package(ignition-common1 REQUIRED)
find_package(ignition-msgs1 REQUIRED)
find_package(ignition-transport4 REQUIRED)

include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
  ${ignition-transport4_INCLUDE_DIRS}
  ${ignition-msgs1_INCLUDE_DIRS}
  ${ignition-common1_INCLUDE_DIRS}
  )

set(project_libs
  ${catkin_LIBRARIES}
  ${ignition-transport4_LIBRARIES}
  ${ignition-msgs1_LIBRARIES}
  ${ignition-common1_LIBRARIES})

#######################################
## Declare Catkin package and installation
#######################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${RUN_DEPENDS}
  LIBRARIES subt_communication_broker subt_communication_client
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
    )

set(protobuf_lib_name SubtCommsProtobuf)
add_subdirectory(src/protobuf)
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS}
                            PROPERTIES GENERATED TRUE)
add_library(${protobuf_lib_name} STATIC ${PROTO_SOURCES})
add_dependencies(${protobuf_lib_name} protobuf_compilation)
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
add_dependencies(subt_communication_client ${protobuf_lib_name})

install(TARGETS subt_communication_broker subt_communication_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )


if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(unit_test tests/unit_test.cpp)
  target_link_libraries(unit_test
    subt_communication_broker
    subt_communication_client
    ${project_libs}
    ${protobuf_lib_name}
    ${GTEST_LIBRARIES})
endif()

