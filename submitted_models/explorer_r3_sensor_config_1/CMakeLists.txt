cmake_minimum_required(VERSION 2.8.3)

project(explorer_r3_sensor_config_1)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch materials meshes urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

