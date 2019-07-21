cmake_minimum_required(VERSION 2.8.3)
project(subt_communication_model)

set(PKG_DEPENDS
  subt_rf_interface
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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ignition-common3_INCLUDE_DIRS}
  )

set(project_libs
  ${catkin_LIBRARIES}
  ${ignition-common3_LIBRARIES})

# Set this project to use C++11
set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (APPLE)
  set(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}")
endif (APPLE)

#######################################
## Declare Catkin package and installation
#######################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME} src/subt_communication_model.cpp)
target_link_libraries(${PROJECT_NAME} ${project_libs})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_unit_test tests/unit_test.cpp)
  target_link_libraries(${PROJECT_NAME}_unit_test
    ${PROJECT_NAME}
    ${project_libs}
    ${GTEST_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )



