cmake_minimum_required(VERSION 2.8.3)
project(ctu_cras_norlab_husky_sensor_config_3)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch meshes materials urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/print_robot_urdf scripts/yaml_to_xacro_args
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
