cmake_minimum_required(VERSION 2.8.3)
project(coro_mike_sensor_config_2)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
