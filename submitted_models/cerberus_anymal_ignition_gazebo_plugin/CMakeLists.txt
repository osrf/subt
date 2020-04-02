cmake_minimum_required(VERSION 2.8.3)
project(cerberus_anymal_ignition_gazebo_plugin)

find_package(catkin REQUIRED)

catkin_package()

install(FILES lib/libAnymalControlPlugin.so
  DESTINATION lib)
