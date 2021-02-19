cmake_minimum_required(VERSION 2.8.3)
project(nasa_perseverance_sensor_config_1)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch resources worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config model.xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

