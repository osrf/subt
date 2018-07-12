cmake_minimum_required(VERSION 3.5.1)
project(subt_example)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

set (SUBT_MAJOR_VERSION 0)
set (SUBT_MINOR_VERSION 1)
set (SUBT_PATCH_VERSION 0)
set (SUBT_VERSION_FULL
  ${SUBT_MAJOR_VERSION}.${SUBT_MINOR_VERSION}.${SUBT_PATCH_VERSION})

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    subt_gazebo
    roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

########################
## Message generation ##
########################

# Add here SubT ROS messages.

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

## Declare a C++ executable
add_executable(subt_example_node src/subt_example_node.cc)
add_dependencies(subt_example_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(subt_example_node ${catkin_LIBRARIES})

###########
## Tests ##
###########


#############
## Install ##
#############

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install all the urdf files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

# Install a python script
catkin_install_python(PROGRAMS scripts/teleop_twist_keyboard.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS subt_example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
