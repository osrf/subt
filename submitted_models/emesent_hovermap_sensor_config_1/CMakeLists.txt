cmake_minimum_required(VERSION 2.8.3)
project(emesent_hovermap_sensor_config_1)

set(CMAKE_CXX_STANDARD 17)
set(IGN_PLUGIN_VER 0)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-common4 REQUIRED)
ign_find_package(ignition-plugin0 REQUIRED COMPONENTS register)

find_package(catkin REQUIRED)

catkin_package()

add_library(GimbalControlPlugin src/ign_gimbal.cpp)
target_link_libraries(GimbalControlPlugin PRIVATE ignition-gazebo6::core ignition-common4::ignition-common4)

install(TARGETS GimbalControlPlugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
