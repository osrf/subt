cmake_minimum_required(VERSION 2.8.3)
project(subt_gazebo_los_model)

set(PKG_DEPENDS
  roscpp
  geometry_msgs
  subt_rf_interface
  gazebo_ros
  gazebo_plugins
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
find_package(ignition-common1 REQUIRED)
find_package(ignition-math4 REQUIRED)

include_directories(
  include
  ${CATKIN_DEVEL_PREFIX}/include  
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
  LIBRARIES subt_gazebo_los_model
  )

# Create the libVisibilityPlugin.so library.
set(visibility_plugin_name VisibilityPlugin)
add_library(${visibility_plugin_name}
  src/VisibilityPlugin.cc)
target_link_libraries(${visibility_plugin_name}
  ${catkin_LIBRARIES}
)
install(TARGETS ${visibility_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(subt_gazebo_los_model
  src/SimpleDOTParser.cc
  src/VisibilityTable.cc)
target_link_libraries(subt_gazebo_los_model
  PUBLIC
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ignition-math4::ignition-math4
  )

# catkin_add_gtest(simpleDOTParser test/simpleDOTParser.cc)
# target_link_libraries(simpleDOTParser
#   ${comms_broker_plugin_name}
# )
