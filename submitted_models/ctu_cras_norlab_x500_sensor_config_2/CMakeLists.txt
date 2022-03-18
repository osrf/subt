cmake_minimum_required(VERSION 3.1.2)

project(ctu_cras_norlab_x500_sensor_config_2)

find_package(catkin REQUIRED)


catkin_package()

install(DIRECTORY launch urdf worlds meshes materials
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
