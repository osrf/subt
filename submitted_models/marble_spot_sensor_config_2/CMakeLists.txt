cmake_minimum_required(VERSION 2.8.3)
project(marble_spot_sensor_config_2)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes textures urdf config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/print_robot_urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
