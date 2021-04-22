cmake_minimum_required(VERSION 3.10)
project(ctu_cras_norlab_marv_sensor_config_2)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  scripts/print_robot_urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
