cmake_minimum_required(VERSION 2.8.3)
project(coro_karen_sensor_config_1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_runtime
  std_msgs
  geometry_msgs
)


find_package(ignition-gazebo4 REQUIRED)
find_package(ignition-transport9 REQUIRED)
find_package(ignition-common3 REQUIRED)
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
  PRIVATE ignition-gazebo4::ignition-gazebo4
  PRIVATE ignition-common3::ignition-common3
  PRIVATE ignition-transport9::ignition-transport9
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

