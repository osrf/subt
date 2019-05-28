cmake_minimum_required(VERSION 3.5.1)
project(subt_example)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)



catkin_package(
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(teleop_node src/teleop_node.cc)
add_dependencies(teleop_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_node ${catkin_LIBRARIES})

###########
## Tests ##
###########


#############
## Install ##
#############

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
# Install all config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install a python script
catkin_install_python(PROGRAMS scripts/cmdvel_converter_rpy.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark executables and/or libraries for installation
install(TARGETS teleop_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
