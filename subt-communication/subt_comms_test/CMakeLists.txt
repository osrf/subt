cmake_minimum_required(VERSION 2.8.3)
project(subt_comms_test)

set(PKG_DEPENDS
  gazebo_ros
  subt_communication_broker
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
find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)


#######################################
## Declare Catkin package and installation
#######################################

catkin_package(
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
  )

add_executable(subt_comms_test src/subt_comms_test.cpp)
target_link_libraries(subt_comms_test
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES})
