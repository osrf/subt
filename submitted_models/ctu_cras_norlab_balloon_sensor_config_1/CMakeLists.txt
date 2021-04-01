cmake_minimum_required(VERSION 3.10)
project(ctu_cras_norlab_balloon_sensor_config_1)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  scripts/print_robot_urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
