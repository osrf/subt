cmake_minimum_required(VERSION 3.1.2)

project(ctu_cras_norlab_x500_sensor_config_1)

find_package(catkin REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-common4 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(MRSMultirotorController SHARED
  src/MRSMultirotorController.cpp
  src/Common.cpp
  src/SE3Controller.cpp
)

target_link_libraries(MRSMultirotorController
  PUBLIC ${catkin_LIBRARIES}
  PRIVATE ignition-plugin1::ignition-plugin1
  PRIVATE ignition-gazebo6::ignition-gazebo6
  PRIVATE ignition-common4::ignition-common4
  PRIVATE ignition-transport11::ignition-transport11
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
