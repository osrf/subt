cmake_minimum_required(VERSION 2.8.3)
project(coro_karen_sensor_config_1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_runtime
  std_msgs
  geometry_msgs
)


find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(Eigen3 REQUIRED)

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

add_library(ackermannlocal SHARED
  src/AckermannSteering.cc
  src/SpeedLimiter.cc
)

target_link_libraries(ackermannlocal
  PUBLIC ${catkin_LIBRARIES}
  PRIVATE ignition-plugin1::ignition-plugin1
  PRIVATE ignition-gazebo6::ignition-gazebo6
  PRIVATE ignition-common4::ignition-common4
  PRIVATE ignition-transport11::ignition-transport11
  PRIVATE Eigen3::Eigen
)


install(TARGETS ackermannlocal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY launch meshes urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

