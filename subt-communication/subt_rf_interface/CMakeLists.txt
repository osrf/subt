cmake_minimum_required(VERSION 2.8.3)
project(subt_rf_interface)

set(PKG_DEPENDS
  roscpp
  geometry_msgs
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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

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
  LIBRARIES subt_rf_range_model
  )

add_library(subt_rf_range_model src/subt_rf_model.cpp)
target_link_libraries(subt_rf_range_model ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(unit_test test/unit_test.cpp)
  target_link_libraries(unit_test
    subt_rf_range_model
    ${GTEST_LIBRARIES})
endif()
