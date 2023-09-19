cmake_minimum_required(VERSION 2.8.3)
project(jhuapl_uav_dynamics_ignition_gazebo_plugin_1)

find_package(catkin REQUIRED)

catkin_package()

install(FILES lib/libjhuapl_uav_dynamics.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
