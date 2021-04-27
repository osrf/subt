cmake_minimum_required(VERSION 2.8.3)
project(ctu_cras_norlab_spot_sensor_config_1)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch meshes textures urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/print_robot_urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)