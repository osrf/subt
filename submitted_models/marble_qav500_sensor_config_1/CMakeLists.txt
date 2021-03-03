cmake_minimum_required(VERSION 3.1.2)

project(marble_qav500_sensor_config_1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
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

add_message_files(DIRECTORY msg FILES
  ControlReference.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime message_generation roscpp std_msgs geometry_msgs
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_library(MRSMultirotorController SHARED
  src/MRSMultirotorController.cc
  src/Common.cc
  src/LeeVelocityController.cc
  src/BacaSE3Controller.cc
)

target_link_libraries(MRSMultirotorController
  PUBLIC ${catkin_LIBRARIES}
  PRIVATE ignition-plugin1::ignition-plugin1
  PRIVATE ignition-gazebo4::ignition-gazebo4
  PRIVATE ignition-common3::ignition-common3
  PRIVATE ignition-transport9::ignition-transport9
  PRIVATE Eigen3::Eigen
)

install(DIRECTORY launch meshes urdf materials worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS MRSMultirotorController
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
