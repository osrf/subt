cmake_minimum_required(VERSION 2.8.3)
project(cerberus_anymal_c_ignition_gazebo_plugin_1)

find_package(catkin REQUIRED)

catkin_package()

install(FILES lib/libAnymalCControlPlugin1.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
