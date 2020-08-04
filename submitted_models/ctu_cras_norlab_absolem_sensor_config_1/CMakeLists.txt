cmake_minimum_required(VERSION 2.8.3)
project(ctu_cras_norlab_absolem_sensor_config_1)

find_package(ignition-gazebo2 REQUIRED)
find_package(ignition-common3 REQUIRED)

find_package(catkin REQUIRED)

catkin_package()

add_library(laser_rotate_plugin src/laser_rotate_plugin.cpp)
target_link_libraries(laser_rotate_plugin PRIVATE ignition-gazebo2::core ignition-common3::ignition-common3)

install(TARGETS laser_rotate_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY launch meshes textures urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

