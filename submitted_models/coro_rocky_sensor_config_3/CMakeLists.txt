cmake_minimum_required(VERSION 2.8.3)
project(coro_rocky_sensor_config_3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_runtime
  std_msgs
  geometry_msgs
)


find_package(ignition-gazebo4 REQUIRED)
find_package(ignition-transport9 REQUIRED)
find_package(ignition-common3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch meshes urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

